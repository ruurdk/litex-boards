#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2023 Ruurd Keizer <ruurdk@hotmail.com>
# SPDX-License-Identifier: BSD-2-Clause
import logging
from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.io import DDROutput

from litex_boards.platforms import microsoft_storeypeak
# from litex_boards.platforms import AltDDR3

from litex.soc.cores.clock.intel_stratix5 import StratixVPLL
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.interconnect.avalon import AvalonMM2Wishbone
from litex.soc.interconnect import wishbone

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_mempll = ClockDomain()        
        self.cd_avalon = ClockDomain()

        mempll_freq = 125e6
        avalon_freq = 200e6
        # # #
        # Clk 
        clk125 = platform.request("clk125")
        
        # No PLL, just hook up
        #self.comb += self.cd_sys.clk.eq(clk125)    

        # PLL
        self.pll = pll = StratixVPLL(speedgrade="-I3")
        # Reset
        self.comb += pll.reset.eq(self.rst)        
        pll.register_clkin(clk125, 125e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_mempll, mempll_freq)

        platform.add_period_constraint(self.cd_sys, 1e9/sys_clk_freq)
        platform.add_period_constraint(self.cd_mempll, 1e9/mempll_freq)
        platform.add_period_constraint(self.cd_avalon.clk, 1e9/avalon_freq)

        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

from migen import *  
from litex.soc.interconnect.csr import *

# Altera UniPHY generated DDR IP wrapper -----------------------------------------------------------
class UniPHYDDR(Module, AutoCSR):
    def __init__(self, platform, ddr_ip_name="altera_mem_if_ddr3", genname = "ddram", address_width = 26, data_width = 512, status_leds = True):
        self.platform = platform        
        self.ddr__ip_name = ddr_ip_name
        self.genname = genname
        self.address_width = address_width
        self.data_width = data_width

        self.init_done = CSRStatus()

        ddram = platform.request("ddram")
        leds = Signal(8)
        memclk = ClockSignal("mempll")  

        self.avalon_clock         = Signal()
        self.avalon_address       = Signal(self.address_width)
        self.avalon_byteenable    = Signal(self.data_width//8)
        self.avalon_read          = Signal()
        self.avalon_readdata      = Signal(self.data_width)
        self.avalon_burstcount    = Signal(8) # 7?
        self.avalon_write         = Signal()
        self.avalon_writedata     = Signal(self.data_width)
        self.avalon_ready         = Signal()
        self.avalon_readdatavalid = Signal()
        self.avalon_burstbegin    = Signal()
        self.avalon_waitrequest   = Signal()

        self.afi_half_clk = Signal()
        self.afi_reset_export_n = Signal()
        self.afi_reset_n = Signal()
        
        # # #        
        ddr3 = Instance("ddram",
            i_pll_ref_clk         = memclk,
            i_global_reset_n      = ~ResetSignal(),
            i_soft_reset_n        = ~ResetSignal(),
            o_afi_clk             = self.avalon_clock,
            o_afi_half_clk        = self.afi_half_clk,
            o_afi_reset_n         = self.afi_reset_export_n,
            o_afi_reset_export_n  = self.afi_reset_n,

            o_mem_a               = ddram.a,
            o_mem_ba              = ddram.ba,
            o_mem_ck              = ddram.clk_p,
            o_mem_ck_n            = ddram.clk_n,
            o_mem_cke             = ddram.cke,
            o_mem_cs_n            = ddram.cs_n,
            o_mem_dm              = ddram.dm,
            o_mem_ras_n           = ddram.ras_n,
            o_mem_cas_n           = ddram.cas_n,
            o_mem_we_n            = ddram.we_n,
            o_mem_reset_n         = ddram.reset_n,
            io_mem_dq             = ddram.dq,
            io_mem_dqs            = ddram.dqs_p,
            io_mem_dqs_n          = ddram.dqs_n,
            o_mem_odt             = ddram.odt,
            i_oct_rzqin           = ddram.oct_rzqin,

            o_avl_ready           = self.avalon_ready,
            i_avl_burstbegin      = self.avalon_burstbegin,
            i_avl_addr            = self.avalon_address,
            o_avl_rdata_valid     = self.avalon_readdatavalid,
            o_avl_rdata           = self.avalon_readdata,
            i_avl_wdata           = self.avalon_writedata,
            i_avl_be              = self.avalon_byteenable,
            i_avl_read_req        = self.avalon_read,
            i_avl_write_req       = self.avalon_write,
            i_avl_size            = self.avalon_burstcount,

            o_local_init_done     = leds[5],
            o_local_cal_success   = leds[6],
            o_local_cal_fail      = leds[7],
            # o_pll_mem_clk         = 
            # o_pll_write_clk       = 
            o_pll_locked          = leds[4],
            # o_pll_capture0_clk    = 
            # o_pll_capture1_clk    = 
        )
        self.specials += ddr3

        self.comb += [
            self.avalon_burstbegin.eq(self.avalon_write & self.avalon_read),
            self.avalon_waitrequest.eq(~self.avalon_ready),
            ClockSignal("avalon").eq(self.avalon_clock),                         
        ]

        if (status_leds):
            self.comb += Cat([platform.request("leds", l) for l in range(8)]).eq(leds)                    

        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"DUAL-REGIONAL CLOCK\" -to ddram:ddram|ddram_pll0:pll0|pll_avl_clk -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"DUAL-REGIONAL CLOCK\" -to ddram:ddram|ddram_pll0:pll0|pll_config_clk -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"GLOBAL CLOCK\" -to ddram:ddram|ddram_pll0:pll0|afi_clk -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"DUAL-REGIONAL CLOCK\" -to ddram:ddram|ddram_pll0:pll0|pll_hr_clk -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL \"GLOBAL CLOCK\" -to ddram:ddram|ddram_pll0:pll0|pll_p2c_read_clk -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_reset:ureset|phy_reset_n -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[0] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[1] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[2] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[3] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[4] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[5] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[6] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[7] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_read_datapath:uread_datapath|reset_n_fifo_wraddress[8] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_s0:s0|rw_manager_ddr3:sequencer_rw_mgr_inst|rw_manager_generic:rw_mgr_inst|rw_manager_core:rw_mgr_core_inst|rw_soft_reset_n -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|altera_reset_controller:rst_controller|altera_reset_synchronizer:alt_rst_sync_uq1|altera_reset_synchronizer_int_chain_out -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name GLOBAL_SIGNAL OFF -to ddram:ddram|ddram_dmaster:if_csr_m0|altera_reset_controller:rst_controller|altera_reset_synchronizer:alt_rst_sync_uq1|altera_reset_synchronizer_int_chain_out -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name ENABLE_BENEFICIAL_SKEW_OPTIMIZATION_FOR_NON_GLOBAL_CLOCKS ON -to ddram:ddram -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name PLL_COMPENSATION_MODE DIRECT -to ddram:ddram|ddram_pll0:pll0|fbout -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name MAX_FANOUT 4 -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|ddram_p0_new_io_pads:uio_pads|ddram_p0_simple_ddio_out:wrdata_en_qr_to_hr|dataout_r[*][*] -tag __ddram_p0")
        platform.add_platform_command("set_instance_assignment -name FORM_DDR_CLUSTERING_CLIQUE ON -to ddram:ddram|ddram_p0:p0|ddram_p0_memphy:umemphy|*qr_to_hr* -tag __ddram_p0")
        platform.add_platform_command("set_global_assignment -name UNIPHY_SEQUENCER_DQS_CONFIG_ENABLE ON")
        platform.add_platform_command("set_global_assignment -name UNIPHY_TEMP_VER_CODE 1671073000")

    def add_sources(self, platform):
       platform.add_source_dir(os.path.join(os.getcwd(), self.genname))

    def do_finalize(self):
        self.add_sources(self.platform)

# Wishbone wrapper
class UniPhyDDR_Wishbone(Module):
    def __init__(self, platform, genname = "ddram"):
        # Instance the UniPHY controller
        ddravl = UniPHYDDR(platform, genname)        
        self.submodules += ddravl        

        # Instance a AvalonMM <==> Wishbone bridge
        bridge = AvalonMM2Wishbone(data_width = ddravl.data_width, avalon_address_width = ddravl.address_width)
        a2w_avl = bridge.a2w_avl
        self.submodules += bridge

        # Directly expose wishbone end of bridge
        self.wb = bridge.a2w_wb

        # Connect UniPHY Avalon to one end of the bridge
        self.comb += {
            a2w_avl.address.eq(ddravl.avalon_address),
            a2w_avl.writedata.eq(ddravl.avalon_writedata),
            a2w_avl.readdata.eq(ddravl.avalon_readdata),
            a2w_avl.readdatavalid.eq(ddravl.avalon_readdatavalid),
            a2w_avl.byteenable.eq(ddravl.avalon_byteenable),
            a2w_avl.read.eq(ddravl.avalon_read),
            a2w_avl.write.eq(ddravl.avalon_write),
            a2w_avl.waitrequest.eq(ddravl.avalon_waitrequest),
            a2w_avl.burstbegin.eq(ddravl.avalon_burstbegin),
            a2w_avl.burstcount.eq(ddravl.avalon_burstcount),
        }


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, 
                 sys_clk_freq=125e6, 
                 with_led_chaser=False, 
                 with_video_terminal=False, 
                 with_uartbone=False, 
                 with_ddram=False, 
                 **kwargs):
        platform = microsoft_storeypeak.Platform()
        self.platform = platform

        self.logger = logging.getLogger("BaseSoC")

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)
    

        # SoCCore ----------------------------------------------------------------------------------
        kwargs["uart_name"] = "serial"
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Microsoft Storey Peak", **kwargs)
        
        # UARTbone ---------------------------------------------------------------------------------
        if with_uartbone:
            self.add_uartbone(name="serial", baudrate=kwargs["uart_baudrate"])        
               
        # DDR SDRAM - wire up generated IP - assume it's there already for now in a folder with name "ddram"
        if with_ddram:
            self.submodules.ddr = ddr = ClockDomainsRenamer("mempll")(UniPhyDDR_Wishbone(platform))

            # Link ddr channel 0 as main RAM
            self.bus.add_slave("main_ram", ddr.wb, SoCRegion(origin=0x4000_0000, size=0x4000_0000, cached=True)) # 1GB.

       
        # Leds -------------------------------------------------------------------------------------        
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("leds"),
                sys_clk_freq = sys_clk_freq)        
        # Exceptions
        
        """
        set_false_path -to {inst_system|pcie1_status_amm|status_reg[*]}
        set_false_path -to {inst_system|pcie2_status_amm|status_reg[*]}

        set_false_path -from {system:inst_system|system_pio_pcie_npor:pio_pcie_npor|data_out}
        """
# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=microsoft_storeypeak.Platform, description="LiteX SoC on Microsoft Storey Peak.")
    parser.add_target_argument("--sys-clk-freq",        default=200e6, type=float, help="System clock frequency.")
    parser.add_argument("--with-uartbone",              action="store_true", help="Enable UARTbone support.")
    parser.add_argument("--with-ddram",                 action="store_true", help="Enable DDR3L RAM support.")

    args = parser.parse_args()
    
    soc = BaseSoC(
        sys_clk_freq        = args.sys_clk_freq,    
        with_uartbone       = args.with_uartbone,
        with_ddram          = args.with_ddram,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

# DDR3 ip generation command TODO move this to build script
# Note: USE WSL2 with Ubuntu 18 or you get weird errors.
# ip-generate altera_mem_if_ddr3_emif  --file-set=QUARTUS_SYNTH --output-name=ddram --component-system-param=DEVICE_FAMILY="Stratix V" --component-param=CALIBRATION_MODE="Quick" --component-param=TIMING_BOARD_DERATE_METHOD="AUTO" --component-param=MEM_TWTR="6" --component-param=STARVE_LIMIT="10" --component-param=SPEED_GRADE="2" --component-param=TIMING_TQSH="0.4" --component-param=OCT_SHARING_MODE="None" --component-param=MEM_TRFC_NS="260.0" --component-param=MEM_TRCD_NS="13.75" --component-param=TIMING_BOARD_SKEW_WITHIN_DQS="0.02" --component-param=C2P_WRITE_CLOCK_ADD_PHASE="0.0" --component-param=TIMING_TIS="140" --component-param=TIMING_BOARD_SKEW_CKDQS_DIMM_MIN="-0.01" --component-param=TIMING_BOARD_MAX_CK_DELAY="0.6" --component-param=DISCRETE_FLY_BY="true" --component-param=MEM_RTT_NOM="RZQ/4" --component-param=MEM_BANKADDR_WIDTH="3" --component-param=MEM_SRT="Normal" --component-param=TIMING_BOARD_SKEW_CKDQS_DIMM_MAX="0.01" --component-param=CTL_ECC_ENABLED="true" --component-param=AUTO_POWERDN_EN="false" --component-param=ENABLE_ABS_RAM_MEM_INIT="false" --component-param=TIMING_BOARD_DQ_TO_DQS_SKEW="0.0" --component-param=CFG_REORDER_DATA="false" --component-param=CTL_SELF_REFRESH_EN="false" --component-param=CORE_DEBUG_CONNECTION="EXPORT" --component-param=TIMING_TDS="40" --component-param=MEM_TRAS_NS="37.5" --component-param=REF_CLK_FREQ="125.0" --component-param=MEM_BT="Sequential" --component-param=FORCE_SHADOW_REGS="AUTO" --component-param=TIMING_TDH="110" --component-param=MEM_DQ_PER_DQS="8" --component-param=MEM_ROW_ADDR_WIDTH="16" --component-param=MEM_TREFI_US="7.8" --component-param=PINGPONGPHY_EN="false" --component-param=MEM_TCL="11" --component-param=TIMING_TDQSCK="225" --component-param=DLL_SHARING_MODE="None" --component-param=CTL_USR_REFRESH_EN="false" --component-param=TIMING_BOARD_MAX_DQS_DELAY="0.6" --component-param=TIMING_BOARD_SKEW_BETWEEN_DQS="0.02" --component-param=MEM_TWR_NS="15.0" --component-param=MEM_FORMAT="DISCRETE" --component-param=CTL_LOOK_AHEAD_DEPTH="4" --component-param=SOPC_COMPAT_RESET="false" --component-param=MEM_VENDOR="Hynix" --component-param=AVL_MAX_SIZE="64" --component-param=MEM_TMRD_CK="4" --component-param=MEM_TRRD_NS="6.25" --component-param=CTL_CSR_CONNECTION="INTERNAL_JTAG" --component-param=DEVICE_DEPTH="1" --component-param=MEM_VOLTAGE="1.35V DDR3L" --component-param=AUTO_PD_CYCLES="0" --component-param=CTL_CSR_ENABLED="true" --component-param=AC_PACKAGE_DESKEW="false" --component-param=MEM_IF_DM_PINS_EN="true" --component-param=USER_DEBUG_LEVEL="1" --component-param=PACKAGE_DESKEW="false" --component-param=PHY_ONLY="false" --component-param=MEM_COL_ADDR_WIDTH="10" --component-param=MEM_CK_PHASE="0.0" --component-param=TIMING_BOARD_AC_TO_CK_SKEW="0.0" --component-param=ADDR_ORDER="0" --component-param=MEM_CK_WIDTH="1" --component-param=MEM_ATCL="Disabled" --component-param=TIMING_TDQSS="0.25" --component-param=TIMING_TDQSQ="150" --component-param=BYTE_ENABLE="true" --component-param=MEM_VERBOSE="true" --component-param=MEM_ASR="Manual" --component-param=MEM_CLK_FREQ_MAX="1066.667" --component-param=SKIP_MEM_INIT="true" --component-param=MEM_WTCL="8" --component-param=RATE="Quarter" --component-param=MEM_TFAW_NS="30.0" --component-param=POWER_OF_TWO_BUS="true" --component-param=TIMING_TDSS="0.2" --component-param=TIMING_TIH="210" --component-param=PLL_LOCATION="Top_Bottom" --component-param=MEM_TRTP_NS="7.5" --component-param=PLL_SHARING_MODE="None" --component-param=MEM_RTT_WR="RZQ/2" --component-param=TIMING_TQH="0.38" --component-param=ENABLE_EXPORT_SEQ_DEBUG_BRIDGE="false" --component-param=MEM_DRV_STR="RZQ/6" --component-param=MEM_PD="DLL off" --component-param=TIMING_BOARD_AC_SKEW="0.02" --component-param=TIMING_TDSH="0.2" --component-param=CTL_AUTOPCH_EN="false" --component-param=EXPORT_AFI_HALF_CLK="false" --component-param=MEM_TRP_NS="13.75" --component-param=TIMING_BOARD_ISI_METHOD="AUTO" --component-param=ADD_EFFICIENCY_MONITOR="false" --component-param=MEM_CLK_FREQ="800.0" --component-param=MEM_MIRROR_ADDRESSING="0" --component-param=CTL_ECC_AUTO_CORRECTION_ENABLED="true" --component-param=MEM_TINIT_US="500" --component-param=MEM_DQ_WIDTH="72" 
# This is the no virtual JTAG version (to make sure we can use JTAG to SOC)
# ip-generate altera_mem_if_ddr3_emif  --file-set=QUARTUS_SYNTH --output-name=ddram --component-system-param=DEVICE_FAMILY="Stratix V" --component-param=CALIBRATION_MODE="Quick" --component-param=TIMING_BOARD_DERATE_METHOD="AUTO" --component-param=MEM_TWTR="6" --component-param=STARVE_LIMIT="10" --component-param=SPEED_GRADE="2" --component-param=TIMING_TQSH="0.4" --component-param=OCT_SHARING_MODE="None" --component-param=MEM_TRFC_NS="260.0" --component-param=MEM_TRCD_NS="13.75" --component-param=TIMING_BOARD_SKEW_WITHIN_DQS="0.02" --component-param=C2P_WRITE_CLOCK_ADD_PHASE="0.0" --component-param=TIMING_TIS="140" --component-param=TIMING_BOARD_SKEW_CKDQS_DIMM_MIN="-0.01" --component-param=TIMING_BOARD_MAX_CK_DELAY="0.6" --component-param=DISCRETE_FLY_BY="true" --component-param=MEM_RTT_NOM="RZQ/4" --component-param=MEM_BANKADDR_WIDTH="3" --component-param=MEM_SRT="Normal" --component-param=TIMING_BOARD_SKEW_CKDQS_DIMM_MAX="0.01" --component-param=CTL_ECC_ENABLED="true" --component-param=AUTO_POWERDN_EN="false" --component-param=ENABLE_ABS_RAM_MEM_INIT="false" --component-param=TIMING_BOARD_DQ_TO_DQS_SKEW="0.0" --component-param=CFG_REORDER_DATA="false" --component-param=CTL_SELF_REFRESH_EN="false" --component-param=CORE_DEBUG_CONNECTION="EXPORT" --component-param=TIMING_TDS="40" --component-param=MEM_TRAS_NS="37.5" --component-param=REF_CLK_FREQ="125.0" --component-param=MEM_BT="Sequential" --component-param=FORCE_SHADOW_REGS="AUTO" --component-param=TIMING_TDH="110" --component-param=MEM_DQ_PER_DQS="8" --component-param=MEM_ROW_ADDR_WIDTH="16" --component-param=MEM_TREFI_US="7.8" --component-param=PINGPONGPHY_EN="false" --component-param=MEM_TCL="11" --component-param=TIMING_TDQSCK="225" --component-param=DLL_SHARING_MODE="None" --component-param=CTL_USR_REFRESH_EN="false" --component-param=TIMING_BOARD_MAX_DQS_DELAY="0.6" --component-param=TIMING_BOARD_SKEW_BETWEEN_DQS="0.02" --component-param=MEM_TWR_NS="15.0" --component-param=MEM_FORMAT="DISCRETE" --component-param=CTL_LOOK_AHEAD_DEPTH="4" --component-param=SOPC_COMPAT_RESET="false" --component-param=MEM_VENDOR="Hynix" --component-param=AVL_MAX_SIZE="64" --component-param=MEM_TMRD_CK="4" --component-param=MEM_TRRD_NS="6.25" --component-param=CTL_CSR_CONNECTION="EXPORT" --component-param=DEVICE_DEPTH="1" --component-param=MEM_VOLTAGE="1.35V DDR3L" --component-param=AUTO_PD_CYCLES="0" --component-param=CTL_CSR_ENABLED="false" --component-param=AC_PACKAGE_DESKEW="false" --component-param=MEM_IF_DM_PINS_EN="true" --component-param=USER_DEBUG_LEVEL="0" --component-param=PACKAGE_DESKEW="false" --component-param=PHY_ONLY="false" --component-param=MEM_COL_ADDR_WIDTH="10" --component-param=MEM_CK_PHASE="0.0" --component-param=TIMING_BOARD_AC_TO_CK_SKEW="0.0" --component-param=ADDR_ORDER="0" --component-param=MEM_CK_WIDTH="1" --component-param=MEM_ATCL="Disabled" --component-param=TIMING_TDQSS="0.25" --component-param=TIMING_TDQSQ="150" --component-param=BYTE_ENABLE="true" --component-param=MEM_VERBOSE="true" --component-param=MEM_ASR="Manual" --component-param=MEM_CLK_FREQ_MAX="1066.667" --component-param=SKIP_MEM_INIT="true" --component-param=MEM_WTCL="8" --component-param=RATE="Quarter" --component-param=MEM_TFAW_NS="30.0" --component-param=POWER_OF_TWO_BUS="true" --component-param=TIMING_TDSS="0.2" --component-param=TIMING_TIH="210" --component-param=PLL_LOCATION="Top_Bottom" --component-param=MEM_TRTP_NS="7.5" --component-param=PLL_SHARING_MODE="None" --component-param=MEM_RTT_WR="RZQ/2" --component-param=TIMING_TQH="0.38" --component-param=ENABLE_EXPORT_SEQ_DEBUG_BRIDGE="false" --component-param=MEM_DRV_STR="RZQ/6" --component-param=MEM_PD="DLL off" --component-param=TIMING_BOARD_AC_SKEW="0.02" --component-param=TIMING_TDSH="0.2" --component-param=CTL_AUTOPCH_EN="false" --component-param=EXPORT_AFI_HALF_CLK="false" --component-param=MEM_TRP_NS="13.75" --component-param=TIMING_BOARD_ISI_METHOD="AUTO" --component-param=ADD_EFFICIENCY_MONITOR="false" --component-param=MEM_CLK_FREQ="800.0" --component-param=MEM_MIRROR_ADDRESSING="0" --component-param=CTL_ECC_AUTO_CORRECTION_ENABLED="true" --component-param=MEM_TINIT_US="500" --component-param=MEM_DQ_WIDTH="72" 
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()






