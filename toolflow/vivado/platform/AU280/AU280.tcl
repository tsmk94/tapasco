# Copyright (c) 2014-2021 Embedded Systems and Applications, TU Darmstadt.
#
# This file is part of TaPaSCo
# (see https://github.com/esa-tu-darmstadt/tapasco).
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

namespace eval platform {
  set platform_dirname "AU280"
  set pcie_width "x16"

  if { [::tapasco::vivado_is_newer "2020.1"] == 0 } {
    puts "Vivado [version -short] is too old to support AU280."
    exit 1
  }

  source $::env(TAPASCO_HOME_TCL)/platform/pcie/pcie_base.tcl

  if {[tapasco::is_feature_enabled "HBM"]} {

    proc get_ignored_segments { } {
      set hbmInterfaces [hbm::get_hbm_interfaces]
      set ignored [list]
      set numInterfaces [llength $hbmInterfaces]
      if {[expr $numInterfaces % 2] == 1} {
        set max_mem_index [expr $numInterfaces + 1]
      } else {
        set max_mem_index $numInterfaces
      }
      for {set i 0} {$i < $numInterfaces} {incr i} {
        for {set j 0} {$j < $max_mem_index} {incr j} {
          set axi_index [format %02s $i]
          set mem_index [format %02s $j]
          lappend ignored "/hbm/hbm_0/SAXI_${axi_index}/HBM_MEM${mem_index}"
        }
      }
      return $ignored
    }

  }

  if {[tapasco::is_feature_enabled "QDMA"]} {
    proc create_subsystem_intc {} {

      puts "Creating interrupt subsystem ..."

      # create hierarchical interface ports
      set s_axi [create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 "S_INTC"]
      set usr_irq [create_bd_intf_pin -mode Master -vlnv xilinx.com:display_eqdma:usr_irq_rtl:1.0 "usr_irq"]

      # create hierarchical ports: clocks and resets
      set aclk [tapasco::subsystem::get_port "host" "clk"]
      set p_aresetn [tapasco::subsystem::get_port "host" "rst" "peripheral" "resetn"]
      set design_aclk [tapasco::subsystem::get_port "design" "clk"]
      set design_aresetn [tapasco::subsystem::get_port "design" "rst" "peripheral" "resetn"]

      # create hierarchical ports: interrupts
      set int_in [::tapasco::ip::create_interrupt_in_ports]
      set int_list [::tapasco::ip::get_interrupt_list]
      set int_mapping [list]

      puts "Starting mapping of interrupts $int_list"

      set int_design_total 0
      set int_design 0
      set int_host 0

      set design_concats_last [tapasco::ip::create_xlconcat "int_cc_design_0" 32]
      set design_concats [list $design_concats_last]

      foreach {name clk} $int_list port $int_in {
        puts "Connecting ${name} (Clk: ${clk}) to ${port}"
        if {$clk == "host"} {
          error "QDMA does not support host interrupts"
        } elseif {$clk == "design"} {
          if { $int_design >= 32 } {
            set n [llength $design_concats]
            set design_concats_last [tapasco::ip::create_xlconcat "int_cc_design_${n}" 32]

            lappend design_concats $design_concats_last

            set int_design 0
          }
          connect_bd_net ${port} [get_bd_pins ${design_concats_last}/In${int_design}]

          lappend int_mapping [expr 4 + $int_design_total]

          incr int_design
          incr int_design_total
        } else {
          error "Memory interrupts are not supported"
        }
      }

      ::tapasco::ip::set_interrupt_mapping $int_mapping

      if {[llength $design_concats] > 1} {
        set cntr 0
        set design_concats_last [tapasco::ip::create_xlconcat "int_cc_design_merge" [llength $design_concats]]
        foreach con $design_concats {
          connect_bd_net [get_bd_pins $con/dout] [get_bd_pins ${design_concats_last}/In${cntr}]
          incr cntr
        }
      }

      # create QDMA interrupt controller
      set qdma_intr_ctrl [tapasco::ip::create_qdma_intr_ctrl "qdma_intr_ctrl"]

      connect_bd_intf_net $s_axi [get_bd_intf_pins $qdma_intr_ctrl/S_AXI]
      connect_bd_intf_net [get_bd_intf_pins $qdma_intr_ctrl/usr_irq] $usr_irq
      connect_bd_net [get_bd_pins $design_concats_last/dout] [get_bd_pins $qdma_intr_ctrl/interrupt_design]

      connect_bd_net $aclk [get_bd_pins $qdma_intr_ctrl/S_AXI_aclk]
      connect_bd_net $design_aclk [get_bd_pins $qdma_intr_ctrl/design_clk]
      connect_bd_net $p_aresetn [get_bd_pins $qdma_intr_ctrl/S_AXI_aresetn]
      connect_bd_net $design_aresetn [get_bd_pins $qdma_intr_ctrl/design_rst]
      save_bd_design
    }
  }

  if {[tapasco::is_feature_enabled "QDMA"]} {
    proc create_subsystem_memory {} {

      puts "Creating memory subsystem ..."

      # create hierarchical interface ports
      set s_axi_mem [create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 "S_MEM_0"]
      set s_axi_qdma [create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 "S_MEM_QDMA"]

      # create hierarchical ports: clocks
      set ddr_aclk [create_bd_pin -type "clk" -dir "O" "ddr_aclk"]
      set design_clk [create_bd_pin -type "clk" -dir "O" "design_aclk"]
      set pcie_aclk [tapasco::subsystem::get_port "host" "clk"]
      set design_aclk [tapasco::subsystem::get_port "design" "clk"]

      # create hierarchical ports: resets
      set ddr_aresetn [create_bd_pin -type "rst" -dir "O" "ddr_aresetn"]
      set design_aresetn [create_bd_pin -type "rst" -dir "O" "design_aresetn"]
      set pcie_p_aresetn [tapasco::subsystem::get_port "host" "rst" "peripheral" "resetn"]
      set ddr_ic_aresetn [tapasco::subsystem::get_port "mem" "rst" "interconnect"]
      set ddr_p_aresetn  [tapasco::subsystem::get_port "mem" "rst" "peripheral" "resetn"]
      set design_p_aresetn [tapasco::subsystem::get_port "design" "rst" "peripheral" "resetn"]

      set mig [create_mig_core "mig"]

      set mig_ic [tapasco::ip::create_axi_sc "mig_ic" 2 1]
      tapasco::ip::connect_sc_default_clocks $mig_ic "mem"

      # AXI connections
      connect_bd_intf_net [get_bd_intf_pins mig_ic/M00_AXI] [get_bd_intf_pins -regexp mig/(C0_DDR4_)?S_AXI]
      connect_bd_intf_net $s_axi_mem [get_bd_intf_pins $mig_ic/S00_AXI]
      connect_bd_intf_net $s_axi_qdma [get_bd_intf_pins $mig_ic/S01_AXI]

      # create clocking wizard
      set design_clk_wiz [tapasco::ip::create_clk_wiz design_clk_wiz]
      set_property -dict [list CONFIG.CLK_OUT1_PORT {design_clk} \
                          CONFIG.USE_SAFE_CLOCK_STARTUP {false} \
                          CONFIG.CLKOUT1_REQUESTED_OUT_FREQ [tapasco::get_design_frequency] \
                          CONFIG.USE_LOCKED {true} \
                          CONFIG.USE_RESET {true} \
                          CONFIG.RESET_TYPE {ACTIVE_LOW} \
                          CONFIG.RESET_PORT {resetn} \
                          CONFIG.PRIM_SOURCE {No_buffer} \
                          ] $design_clk_wiz

      # connect clocks and resets
      connect_bd_net $ddr_p_aresetn [get_bd_pins -regexp mig/(c0_ddr4_)?aresetn]

      set ddr_clk [get_bd_pins -regexp mig/(c0_ddr4_)?ui_clk]
      connect_bd_net $ddr_clk $ddr_aclk [get_bd_pins $design_clk_wiz/clk_in1]
      connect_bd_net [get_bd_pins $design_clk_wiz/resetn] [get_bd_pins -regexp $mig/((mmcm_locked)|(c0_init_calib_complete))]
      connect_bd_net [get_bd_pins $design_clk_wiz/design_clk] $design_clk
      connect_bd_net [get_bd_pins $design_clk_wiz/locked] $design_aresetn

      if {[get_property CONFIG.POLARITY [get_bd_pins -regexp mig/(c0_ddr4_)?ui_clk_sync_rst]] == "ACTIVE_HIGH"} {
        set ddr_rst_inverter [tapasco::ip::create_logic_vector "ddr_rst_inverter"]
        set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not} CONFIG.LOGO_FILE {data/sym_notgate.png}] [get_bd_cells $ddr_rst_inverter]
        connect_bd_net [get_bd_pins $ddr_rst_inverter/Op1] [get_bd_pins -regexp mig/(c0_ddr4_)?ui_clk_sync_rst]
        connect_bd_net [get_bd_pins $ddr_rst_inverter/Res] $ddr_aresetn
      } else {
        connect_bd_net $ddr_aresetn [get_bd_pins -regexp mig/(c0_ddr4_)?ui_clk_sync_rst]
      }
      save_bd_design
    }
  }

  proc create_mig_core {name} {
    puts "Creating MIG core for DDR ..."
    set s_axi_host [create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 "S_MEM_CTRL"]

    set mig [tapasco::ip::create_us_ddr ${name}]
    apply_bd_automation -rule xilinx.com:bd_rule:board -config { Board_Interface {ddr4_sdram_c1 ( DDR4 SDRAM C1 ) } Manual_Source {Auto}}  [get_bd_intf_pins $mig/C0_DDR4]
    apply_bd_automation -rule xilinx.com:bd_rule:board -config { Board_Interface {sysclk1 ( 100 MHz System differential clock1 ) } Manual_Source {Auto}}  [get_bd_intf_pins $mig/C0_SYS_CLK]
    apply_bd_automation -rule xilinx.com:bd_rule:board -config { Board_Interface {resetn ( FPGA Resetn ) } Manual_Source {New External Port (ACTIVE_HIGH)}}  [get_bd_pins $mig/sys_rst]


    connect_bd_intf_net [get_bd_intf_pins $mig/C0_DDR4_S_AXI_CTRL] $s_axi_host

    set const [tapasco::ip::create_constant constz 1 0]
    make_bd_pins_external $const

    set constraints_fn "$::env(TAPASCO_HOME_TCL)/platform/AU280/board.xdc"
    read_xdc $constraints_fn
    set_property PROCESSING_ORDER EARLY [get_files $constraints_fn]

    set inst [current_bd_instance -quiet .]
    current_bd_instance -quiet

    set m_si [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 host/M_MEM_CTRL]

    set num_mi_old [get_property CONFIG.NUM_MI [get_bd_cells host/out_ic]]
    set num_mi [expr "$num_mi_old + 1"]
    set_property -dict [list CONFIG.NUM_MI $num_mi] [get_bd_cells host/out_ic]
    connect_bd_intf_net $m_si [get_bd_intf_pins host/out_ic/[format "M%02d_AXI" $num_mi_old]]

    current_bd_instance -quiet $inst

    return $mig
  }

  if {[tapasco::is_feature_enabled "QDMA"]} {
    proc create_subsystem_host {} {
      set device_type [get_property ARCHITECTURE [get_parts -of_objects [current_project]]]
      puts "Device type is $device_type"

      puts "Creating host subsystem ..."

      # create hierarchical ports
      set m_arch [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 "M_ARCH"]
      set m_intc [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 "M_INTC"]
      set m_tapasco [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 "M_TAPASCO"]
      set m_mem_qdma [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 "M_MEM_QDMA"]
      set m_desc_gen [create_bd_intf_pin -mode Master -vlnv xilinx.com:interface:aximm_rtl:1.0 "M_DESC_GEN"]
      set s_desc_gen [create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 "S_DESC_GEN"]
      set pcie_aclk [create_bd_pin -type "clk" -dir "O" "pcie_aclk"]
      set pcie_aresetn [create_bd_pin -type "rst" -dir "O" "pcie_aresetn"]
      set usr_irq [create_bd_intf_pin -mode Slave -vlnv xilinx.com:display_eqdma:usr_irq_rtl:1.0 "usr_irq"]

      set pcie_aclk_in [tapasco::subsystem::get_port "host" "clk"]
      set pcie_p_aresetn [tapasco::subsystem::get_port "host" "rst" "peripheral" "resetn"]

      puts "Creating QDMA ..."
      set qdma [tapasco::ip::create_qdma qdma_0]

      apply_bd_automation -rule xilinx.com:bd_rule:qdma -config { axi_clk {Maximum Data Width} axi_intf {AXI_MM} bar_size {Disable} lane_width {X16} link_speed {8.0 GT/s (PCIe Gen 3)}}  [get_bd_cells $qdma]

      set_property -dict [list CONFIG.axist_bypass_en {true} \
        CONFIG.dsc_byp_mode {Descriptor_bypass_and_internal} \
        CONFIG.pf0_bar0_type_qdma {AXI_Bridge_Master} \
        CONFIG.pf0_bar0_scale_qdma {Megabytes} \
        CONFIG.pf0_bar0_size_qdma {64} \
        CONFIG.pf0_bar2_type_qdma {DMA} \
        CONFIG.pf0_bar2_size_qdma {256} \
        CONFIG.pf1_bar0_type_qdma {AXI_Bridge_Master} \
        CONFIG.pf1_bar0_scale_qdma {Megabytes} \
        CONFIG.pf1_bar0_size_qdma {64} \
        CONFIG.pf1_bar2_type_qdma {DMA} \
        CONFIG.pf1_bar2_size_qdma {256} \
        CONFIG.pf2_bar0_type_qdma {AXI_Bridge_Master} \
        CONFIG.pf2_bar0_scale_qdma {Megabytes} \
        CONFIG.pf2_bar0_size_qdma {64} \
        CONFIG.pf2_bar2_type_qdma {DMA} \
        CONFIG.pf2_bar2_size_qdma {256} \
        CONFIG.pf3_bar0_type_qdma {AXI_Bridge_Master} \
        CONFIG.pf3_bar0_scale_qdma {Megabytes} \
        CONFIG.pf3_bar0_size_qdma {64} \
        CONFIG.pf3_bar2_type_qdma {DMA} \
        CONFIG.pf3_bar2_size_qdma {256} \
        CONFIG.csr_axilite_slave {false} \
        CONFIG.en_bridge_slv {true} \
        CONFIG.axibar_notranslate {true} \
        CONFIG.vdm_en {1} \
        CONFIG.pf0_device_id {7038} \
        CONFIG.PF0_MSIX_CAP_TABLE_SIZE_qdma {01F} \
        CONFIG.PF0_MSIX_CAP_TABLE_BIR_qdma {BAR_3:2} \
        CONFIG.PF1_MSIX_CAP_TABLE_BIR_qdma {BAR_3:2} \
        CONFIG.PF2_MSIX_CAP_TABLE_BIR_qdma {BAR_3:2} \
        CONFIG.PF3_MSIX_CAP_TABLE_BIR_qdma {BAR_3:2} \
        CONFIG.PF0_MSIX_CAP_PBA_BIR_qdma {BAR_3:2} \
        CONFIG.PF1_MSIX_CAP_PBA_BIR_qdma {BAR_3:2} \
        CONFIG.PF2_MSIX_CAP_PBA_BIR_qdma {BAR_3:2} \
        CONFIG.PF3_MSIX_CAP_PBA_BIR_qdma {BAR_3:2} \
        CONFIG.adv_int_usr {true} \
        CONFIG.en_pcie_drp {true}] [get_bd_cells qdma_0]

      set qdma_conf [tapasco::ip::create_qdma_configurator qdma_conf_0]
      connect_bd_net $pcie_aclk_in [get_bd_pins $qdma_conf/clk] [get_bd_pins $qdma/drp_clk]
      connect_bd_net $pcie_p_aresetn [get_bd_pins $qdma_conf/resetn]
      connect_bd_intf_net [get_bd_intf_pins $qdma_conf/drp] [get_bd_intf_pins $qdma/drp]
      connect_bd_intf_net [get_bd_intf_pins $qdma_conf/msix_vector_ctrl] [get_bd_intf_pins $qdma/msix_vector_ctrl]

      # create AXI Smartconnect for Slave Bridge, and Dummy Master to make Vivado happy
      set in_ic [tapasco::ip::create_axi_sc "in_ic" 2 1]
      tapasco::ip::connect_sc_default_clocks $in_ic "host"
      set dummy_master [tapasco::ip::create_axi_dummy_master "dummy_master"]
      connect_bd_net $pcie_aclk_in [get_bd_pins $dummy_master/M_AXI_aclk]
      connect_bd_net $pcie_p_aresetn [get_bd_pins $dummy_master/M_AXI_aresetn]

      # create AXI Smartconnect for Master bridge
      set out_ic [tapasco::ip::create_axi_sc "out_ic" 1 4]
      tapasco::ip::connect_sc_default_clocks $out_ic "host"

      connect_bd_intf_net [get_bd_intf_pins $qdma/M_AXI_BRIDGE] \
        [get_bd_intf_pins -of_objects $out_ic -filter "VLNV == [tapasco::ip::get_vlnv aximm_intf] && MODE == Slave"]

      # create QDMA descriptor generator
      set desc_gen [tapasco::ip::create_qdma_descriptor_gen desc_gen_0]

      connect_bd_net $pcie_aclk_in [get_bd_pins $desc_gen/aclk]
      connect_bd_net $pcie_p_aresetn [get_bd_pins $desc_gen/resetn]
      connect_bd_net [get_bd_pins $desc_gen/dma_resetn] [get_bd_pins $qdma/soft_reset_n]
      connect_bd_intf_net [get_bd_intf_pins $desc_gen/c2h_byp_in] [get_bd_intf_pins $qdma/c2h_byp_in_mm]
      connect_bd_intf_net [get_bd_intf_pins $desc_gen/h2c_byp_in] [get_bd_intf_pins $qdma/h2c_byp_in_mm]
      connect_bd_intf_net [get_bd_intf_pins $qdma/tm_dsc_sts] [get_bd_intf_pins $desc_gen/tm_dsc_sts]
      connect_bd_intf_net [get_bd_intf_pins $qdma/qsts_out] [get_bd_intf_pins $desc_gen/qsts_out]
      connect_bd_intf_net [get_bd_intf_pins $qdma/c2h_byp_out] [get_bd_intf_pins $desc_gen/c2h_byp_out]
      connect_bd_intf_net [get_bd_intf_pins $qdma/h2c_byp_out] [get_bd_intf_pins $desc_gen/h2c_byp_out]
      connect_bd_intf_net $usr_irq [get_bd_intf_pins $qdma/usr_irq]

      # create AXI connections
      connect_bd_intf_net [get_bd_intf_pins $qdma/M_AXI] $m_mem_qdma
      connect_bd_intf_net [get_bd_intf_pins -of_objects $out_ic -filter {NAME == M00_AXI}] $m_arch
      connect_bd_intf_net [get_bd_intf_pins -of_objects $out_ic -filter {NAME == M01_AXI}] $m_intc
      connect_bd_intf_net [get_bd_intf_pins -of_objects $out_ic -filter {NAME == M02_AXI}] $m_tapasco
      connect_bd_intf_net [get_bd_intf_pins -of_objects $out_ic -filter {NAME == M03_AXI}] $m_desc_gen
      connect_bd_intf_net [get_bd_intf_pins $dummy_master/M_AXI] [get_bd_intf_pins $in_ic/S00_AXI]
      connect_bd_intf_net [get_bd_intf_pins -of_object $in_ic -filter { MODE == Master }] \
        [get_bd_intf_pins $qdma/S_AXI_BRIDGE]
      connect_bd_intf_net $s_desc_gen [get_bd_intf_pins $desc_gen/S_AXI_CTRL]

      # connect PCIe clock and reset
      connect_bd_net [get_bd_pins $qdma/axi_aclk] $pcie_aclk
      connect_bd_net [get_bd_pins $qdma/axi_aresetn] $pcie_aresetn
    }
  } else {
    proc create_pcie_core {} {
      puts "Creating AXI PCIe Gen3 bridge ..."

      set pcie_core [tapasco::ip::create_axi_pcie3_0_usp axi_pcie3_0]

      apply_bd_automation -rule xilinx.com:bd_rule:board -config { Board_Interface {pci_express_x16 ( PCI Express ) } Manual_Source {Auto}}  [get_bd_intf_pins $pcie_core/pcie_mgt]
      apply_bd_automation -rule xilinx.com:bd_rule:board -config { Board_Interface {pcie_perstn ( PCI Express ) } Manual_Source {New External Port (ACTIVE_LOW)}}  [get_bd_pins $pcie_core/sys_rst_n]

      apply_bd_automation -rule xilinx.com:bd_rule:xdma -config { accel {1} auto_level {IP Level} axi_clk {Maximum Data Width} axi_intf {AXI Memory Mapped} bar_size {Disable} bypass_size {Disable} c2h {4} cache_size {32k} h2c {4} lane_width {X16} link_speed {8.0 GT/s (PCIe Gen 3)}}  [get_bd_cells $pcie_core]

      # enable second BAR with 256 kB to have matching BAR configuration between BlueDMA and QDMA versions
      set pcie_properties [list \
        CONFIG.functional_mode {AXI_Bridge} \
        CONFIG.mode_selection {Advanced} \
        CONFIG.pcie_blk_locn {PCIE4C_X1Y0} \
        CONFIG.pl_link_cap_max_link_width {X16} \
        CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} \
        CONFIG.axi_addr_width {64} \
        CONFIG.pipe_sim {true} \
        CONFIG.pf0_revision_id {01} \
        CONFIG.pf0_base_class_menu {Memory_controller} \
        CONFIG.pf0_sub_class_interface_menu {Other_memory_controller} \
        CONFIG.pf0_interrupt_pin {NONE} \
        CONFIG.pf0_msi_enabled {false} \
        CONFIG.SYS_RST_N_BOARD_INTERFACE {pcie_perstn} \
        CONFIG.PCIE_BOARD_INTERFACE {pci_express_x16} \
        CONFIG.pf0_msix_enabled {true} \
        CONFIG.c_m_axi_num_write {32} \
        CONFIG.pf0_msix_impl_locn {External} \
        CONFIG.pf0_bar0_size {64} \
        CONFIG.pf0_bar0_scale {Megabytes} \
        CONFIG.pf0_bar0_64bit {true} \
        CONFIG.pf0_bar2_enabled {true} \
        CONFIG.pf0_bar2_size {256} \
        CONFIG.pf0_bar2_scale {Kilobytes} \
        CONFIG.pf0_bar2_64bit {true} \
        CONFIG.axi_data_width {512_bit} \
        CONFIG.pf0_device_id {7038} \
        CONFIG.pf0_class_code_base {05} \
        CONFIG.pf0_class_code_sub {80} \
        CONFIG.pf0_class_code_interface {00} \
        CONFIG.xdma_axilite_slave {true} \
        CONFIG.coreclk_freq {500} \
        CONFIG.plltype {QPLL1} \
        CONFIG.pf0_msix_cap_table_size {83} \
        CONFIG.pf0_msix_cap_table_offset {20000} \
        CONFIG.pf0_msix_cap_table_bir {BAR_1:0} \
        CONFIG.pf0_msix_cap_pba_offset {28000} \
        CONFIG.pf0_msix_cap_pba_bir {BAR_1:0} \
        CONFIG.bar_indicator {BAR_1:0} \
        CONFIG.bar0_indicator {0}
        ]

      if {[catch {set_property -dict $pcie_properties $pcie_core}]} {
        error "ERROR: Failed to configure PCIe bridge. This may be related to the format settings of your OS for numbers. Please check that it is set to 'United States' (see AR# 51331)"
      }
      set_property -dict $pcie_properties $pcie_core


      tapasco::ip::create_msixusptrans "MSIxTranslator" $pcie_core

      return $pcie_core
    }
  }

  # Checks if the optional register slice given by the name is enabled (based on regslice feature and default value)
  proc is_regslice_enabled {name default} {
    if {[tapasco::is_feature_enabled "Regslice"]} {
      set regslices [tapasco::get_feature "Regslice"]
      if  {[dict exists $regslices $name]} {
          return [dict get $regslices $name]
        } else {
          return $default
        }
    } else {
      return $default
    }
  }

  # Inserts a new register slice between given master and slave (for SLR crossing)
  proc insert_regslice {name default master slave clock reset subsystem} {
    if {[is_regslice_enabled $name $default]} {
      set regslice [tapasco::ip::create_axi_reg_slice $subsystem/regslice_${name}]
      set_property -dict [list CONFIG.REG_AW {15} CONFIG.REG_AR {15} CONFIG.REG_W {15} CONFIG.REG_R {15} CONFIG.REG_B {15} CONFIG.USE_AUTOPIPELINING {1}] $regslice
      delete_bd_objs [get_bd_intf_nets -of_objects [get_bd_intf_pins $master]]
      connect_bd_intf_net [get_bd_intf_pins $master] [get_bd_intf_pins $regslice/S_AXI]
      connect_bd_intf_net [get_bd_intf_pins $regslice/M_AXI] [get_bd_intf_pins $slave]
      connect_bd_net [get_bd_pins $clock] [get_bd_pins $regslice/aclk]
      connect_bd_net [get_bd_pins $reset] [get_bd_pins $regslice/aresetn]
    }
  }

  # Insert optional register slices
  proc insert_regslices {} {
    insert_regslice "host_memctrl" true "/host/M_MEM_CTRL" "/memory/S_MEM_CTRL" "/clocks_and_resets/mem_clk" "/clocks_and_resets/mem_interconnect_aresetn" ""
    insert_regslice "arch_mem" false "/arch/M_MEM_0" "/memory/S_MEM_0" "/clocks_and_resets/design_clk" "/clocks_and_resets/design_interconnect_aresetn" ""
    if {[tapasco::is_feature_enabled "QDMA"]} {
      insert_regslice "host_dma" true "/host/M_MEM_QDMA" "/memory/S_MEM_QDMA" "/clocks_and_resets/host_clk" "/clocks_and_resets/host_interconnect_aresetn" ""
    } else {
      insert_regslice "host_dma" true "/host/M_DMA" "/memory/S_DMA" "/clocks_and_resets/host_clk" "/clocks_and_resets/host_interconnect_aresetn" ""
      insert_regslice "dma_migic" false "/memory/dma/m32_axi" "/memory/mig_ic/S00_AXI" "/memory/mem_clk" "/memory/mem_peripheral_aresetn" "/memory"
      insert_regslice "dma_host" true "/memory/M_HOST" "/host/S_HOST" "/clocks_and_resets/host_clk" "/clocks_and_resets/host_interconnect_aresetn" ""
    }
    insert_regslice "host_arch" true "/host/M_ARCH" "/arch/S_ARCH" "/clocks_and_resets/design_clk" "/clocks_and_resets/design_interconnect_aresetn" ""
    insert_regslice "l2_cache" [tapasco::is_feature_enabled "Cache"] "/memory/cache_l2_0/M0_AXI" "/memory/mig/C0_DDR4_S_AXI" "/clocks_and_resets/mem_clk" "/clocks_and_resets/mem_peripheral_aresetn" "/memory"

    insert_regslice "host_mmu" [tapasco::is_feature_enabled "SVM"] "/host/M_MMU" "/memory/S_MMU" "/clocks_and_resets/host_clk" "/clocks_and_resets/host_interconnect_aresetn" ""

    if {[is_regslice_enabled "pe" false]} {
      set ips [get_bd_cells /arch/target_ip_*]
      foreach ip $ips {
        set masters [tapasco::get_aximm_interfaces $ip]
        foreach master $masters {
          set slave [get_bd_intf_pins -filter {MODE == Slave} -of_objects [get_bd_intf_nets -of_objects $master]]
          insert_regslice [get_property NAME $ip] true $master $slave "/arch/design_clk" "/arch/design_interconnect_aresetn" "/arch"
        }
      }
    }
  }

  namespace eval AU280 {
        namespace export addressmap

        proc addressmap {args} {
            # add ECC config to platform address map
            set args [lappend args "M_MEM_CTRL" [list 0x40000 0x10000 0 "PLATFORM_COMPONENT_ECC"]]
            if {[tapasco::is_feature_enabled "QDMA"]} {
              set args [lappend args "M_MEM_QDMA" [list 0 0 [expr "1 << 64"] ""]]
              set args [lappend args "M_DESC_GEN" [list 0x00010000 0x10000 0      "PLATFORM_COMPONENT_DMA0"]]
            }
            return $args
        }
    }


  tapasco::register_plugin "platform::AU280::addressmap" "post-address-map"

  tapasco::register_plugin "platform::insert_regslices" "post-platform"

}
