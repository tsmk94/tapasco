/*
 * Copyright (c) 2014-2020 Embedded Systems and Applications, TU Darmstadt.
 *
 * This file is part of TaPaSCo
 * (see https://github.com/esa-tu-darmstadt/tapasco).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

use snafu::ResultExt;
use std::ffi::CString;
use std::os::unix::io::FromRawFd;
use std::fs::{File, OpenOptions, read_link};
use std::os::unix::io::AsRawFd;
use std::sync::{Mutex, Arc};
use vfio_bindings::bindings::vfio::*;
use config::Config;

pub const IOMMU_PAGESIZE: u64 = 4096;
// TODO: Is this offset 0x0008_0000_0000 correct or should it be 0x8000_0000?
pub const HP_OFFS: u64 = 0x0008_0000_0000; // AXI Offset IP block between PE and PS

// VFIO ioctl import
//
// Magic ioctl numbers and ioctl arguments are taken from :
// https://elixir.bootlin.com/linux/latest/source/include/uapi/linux/vfio.h
ioctl_none_bad!(
    vfio_get_api_version,
    request_code_none!(VFIO_TYPE, VFIO_BASE)
);

ioctl_write_int_bad!(
    vfio_check_extension,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 1)
);

ioctl_read_bad!(
    vfio_group_get_status,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 3),
    vfio_group_status
);

ioctl_write_int_bad!(
    vfio_set_iommu,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 2)
);

ioctl_write_ptr_bad!(
    vfio_group_set_container,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 4),
    i32
);

ioctl_write_ptr_bad!(
    vfio_group_get_device_fd,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 6),
    u8
);

ioctl_read_bad!(
    vfio_device_get_info,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 7),
    vfio_device_info
);

ioctl_readwrite_bad!(
    vfio_device_get_region_info,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 8),
    vfio_region_info
);

ioctl_read_bad!(
    vfio_iommu_get_info,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 12),
    vfio_iommu_type1_info
);

ioctl_write_ptr_bad!(
    vfio_iommu_map_dma,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 13),
    vfio_iommu_type1_dma_map
);

ioctl_readwrite_bad!(
    vfio_iommu_unmap_dma,
    request_code_none!(VFIO_TYPE, VFIO_BASE + 14),
    vfio_iommu_type1_dma_unmap
);

#[derive(Debug, Snafu)]
pub enum Error {
    #[snafu(display("Could not open {}: {}", file, source))]
    VfioOpen {
        source: std::io::Error,
        file: String
    },

    #[snafu(display("Make sure the vfio_platform driver is loaded: {}, {}", file, source))]
    VfioNoGroup {
        source: std::io::Error,
        file: String
    },

    #[snafu(display("Could not parse configuration {}", source))]
    ConfigError { source: config::ConfigError },

    #[snafu(display("IOCTL {} failed", name))]
    IoctlError { name: String },

    #[snafu(display("IOMMU mapping for iova=0x{:x} not found", iova))]
    MappingError { iova: u64 },
}

/// Instance of an SMMU mapping
///
/// iova: virtual memory address used by PL
/// size: Size of the mapped memory region
#[derive(Debug)]
pub struct VfioMapping {
    pub iova: u64,
    pub size: u64
}

/// Instance of the current VFIO context
#[derive(Debug)]
pub struct VfioDev {
    container: File,
    device: File,
    pub mappings: Mutex<Vec<VfioMapping>>
}
impl VfioDev {
    pub fn get_region_size(&self, iova: u64) -> Result<u64, Error> {
        let m = self.mappings.lock().unwrap();
        match m.iter().find(|x| x.iova == iova) {
            Some(e) => Ok(e.size),
            None => Err(Error::MappingError { iova })
        }
    }
}
impl Default for VfioDev {
    // only used for testing
    fn default() -> Self {
        Self {
            container: File::open("/dev/null").unwrap(),
            device: File::open("/dev/null").unwrap(),
            mappings: Mutex::new(Vec::new())
        }
    }
}

pub const fn to_page_boundary(x: u64) -> u64 {
    x - (x % IOMMU_PAGESIZE)
}

// get VFIO group number of tapasco platform device from sysfs
fn get_vfio_group(settings: &Arc<Config>) -> Result<i32, Error> {
    let dev_path = settings
        .get_string("tlkm.vfio_device")
        .context(ConfigSnafu)?;
    let iommu_group_path = read_link(&dev_path)
        .context(VfioNoGroupSnafu {file: &dev_path} )?;
    let iommu_group = iommu_group_path.file_name().unwrap().to_str().unwrap();

    Ok(iommu_group.parse().unwrap())
}

pub fn init_vfio(settings: Arc<Config>) -> Result<VfioDev, Error> {
    trace!("Initializing VFIO");
    let container_path = "/dev/vfio/vfio";
    let container = OpenOptions::new()
        .read(true)
        .write(true)
        .open(container_path)
        .context(VfioOpenSnafu { file: container_path })?;

    let mut ret = unsafe { vfio_get_api_version(container.as_raw_fd()) }.unwrap();
    if ret != VFIO_API_VERSION as i32 {
        error!("VFIO version is {} should be {}", ret, VFIO_API_VERSION);
        return Err(Error::IoctlError{ name: "vfio_get_api_version".to_string() });
    }
    trace!("VFIO version is {}, okay!", ret);

    ret = unsafe { vfio_check_extension(container.as_raw_fd(), VFIO_TYPE1_IOMMU as i32) }.unwrap();
    if ret <= 0 {
        error!("VFIO_TYPE1_IOMMU not supported");
        return Err(Error::IoctlError{ name: "vfio_check_extension".to_string() });
    }
    trace!("VFIO_TYPE1_IOMMU okay!");

    let group_path = format!("/dev/vfio/{}", get_vfio_group(&settings)?);
    let group = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&group_path)
        .context(VfioNoGroupSnafu {file: &group_path} )?;

    let mut group_status = vfio_group_status {
        argsz: std::mem::size_of::<vfio_group_status>() as u32,
        flags: 0,
    };
    ret = unsafe { vfio_group_get_status(group.as_raw_fd(), &mut group_status) }.unwrap();
    if ret < 0 {
        return Err(Error::IoctlError{ name: "vfio_group_get_status".to_string() });
    } else if (group_status.flags & VFIO_GROUP_FLAGS_VIABLE) != VFIO_GROUP_FLAGS_VIABLE {
        error!("VFIO group is not viable\n");
        return Err(Error::IoctlError{ name: "vfio_group_get_status".to_string() });
    }
    trace!("VFIO group is okay\n");

    ret = unsafe { vfio_group_set_container(group.as_raw_fd(), &container.as_raw_fd()) }.unwrap();
    if ret < 0 {
        return Err(Error::IoctlError{ name: "vfio_group_set_container".to_string() });
    }
    trace!("VFIO set container okay\n");

    ret = unsafe { vfio_set_iommu(container.as_raw_fd(), VFIO_TYPE1_IOMMU as i32) }.unwrap();
    if ret < 0 {
        return Err(Error::IoctlError{ name: "vfio_set_iommu".to_string() });
    }
    trace!("vfio_set_iommu okay\n");

    let dev_name = CString::new("tapasco").unwrap();
    let dev_fd = unsafe { vfio_group_get_device_fd(
        group.as_raw_fd(),
        dev_name.as_ptr() as *const u8
    ) }.unwrap();
    if dev_fd < 0 {
        return Err(Error::IoctlError{ name: "vfio_group_get_device_fd".to_string() });
    }
    trace!("vfio_group_get_device_fd okay: fd={}\n", dev_fd);

    Ok(VfioDev{
        container,
        device: unsafe { File::from_raw_fd(dev_fd) },
        mappings: Mutex::new(Vec::new())
    })
}

pub fn vfio_get_info(dev: &VfioDev) -> Result<vfio_iommu_type1_info, Error> {
    let mut iommu_info = vfio_iommu_type1_info {
        argsz: std::mem::size_of::<vfio_iommu_type1_info>() as u32,
        flags: 0,
        iova_pgsizes: 0,
    };
    let ret = unsafe { vfio_iommu_get_info(dev.container.as_raw_fd(), &mut iommu_info) }.unwrap();
    if ret < 0 {
        Err(Error::IoctlError{ name: "vfio_iommu_get_info".to_string() })
    } else {
        trace!("flags={}, Pagesize bitvector=0x{:x}!\n", iommu_info.flags, iommu_info.iova_pgsizes);
        Ok(iommu_info)
    }
}

pub fn vfio_get_region_info(dev: &VfioDev) -> Result<vfio_device_info, Error> {
    let mut dev_info = vfio_device_info {
        argsz: std::mem::size_of::<vfio_device_info>() as u32,
        flags: 0,
        num_regions: 0,
        num_irqs: 0,
    };
    let ret = unsafe { vfio_device_get_info(dev.device.as_raw_fd(), &mut dev_info) }.unwrap();
    if ret < 0 {
        return Err(Error::IoctlError{ name: "vfio_device_get_info".to_string() });
    }
    trace!("VFIO device has {} regions\n", dev_info.num_regions);

    // get info for all regions
    for r in 0..dev_info.num_regions {
        let mut reg_info = vfio_region_info {
            argsz: std::mem::size_of::<vfio_region_info>() as u32,
            flags: 0,
            index: r,
            cap_offset: 0,
            size: 0,
            offset: 0,
        };
        let ret = unsafe { vfio_device_get_region_info(dev.device.as_raw_fd(), &mut reg_info) }.unwrap();
        if ret < 0 {
            return Err(Error::IoctlError{ name: "vfio_device_get_region_info".to_string() });
        }
        trace!("Region {}: sz=0x{:x}, offs=0x{:x}\n", r, reg_info.size, reg_info.offset);
    }
    Ok(dev_info)
}

pub fn vfio_dma_map(dev: &VfioDev, size: u64, iova: u64, vaddr: u64) -> Result<(), Error> {
    let dma_map_src = vfio_iommu_type1_dma_map {
        argsz: std::mem::size_of::<vfio_iommu_type1_dma_map>() as u32,
        flags: VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        vaddr,
        iova,
        size,
    };

    let ret = unsafe { vfio_iommu_map_dma(dev.container.as_raw_fd(), &dma_map_src) }.unwrap();
    if ret < 0 {
        Err(Error::IoctlError{ name: "vfio_iommu_map_dma".to_string() })
    } else {
        trace!("vfio_iommu_map_dma: va=0x{:x} -> iova=0x{:x}, size=0x{:x}\n", vaddr, iova, size);
        Ok(())
    }
}

pub fn vfio_dma_unmap(dev: &VfioDev, iova: u64, size: u64) -> Result<(), Error> {
    let mut dma_unmap = vfio_iommu_type1_dma_unmap {
        argsz: std::mem::size_of::<vfio_iommu_type1_dma_unmap>() as u32,
        flags: 0,
        iova,
        size,
    };

    let ret = unsafe { vfio_iommu_unmap_dma(dev.container.as_raw_fd(), &mut dma_unmap) }.unwrap();
    if ret < 0 {
        Err(Error::IoctlError{ name: "vfio_iommu_unmap_dma".to_string() })
    } else {
        trace!("vfio_iommu_unmap_dma: iova=0x{:x}, size=0x{:x}\n", iova, size);
        Ok(())
    }
}
