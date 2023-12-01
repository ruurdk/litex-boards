#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2023 Ruurd Keizer <ruurdk@hotmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litex.build.generic_platform import Pins, IOStandard, Subsignal, Misc
from litex.build.altera import AlteraPlatform
from litex.build.altera.programmer import USBBlaster

# IOs ----------------------------------------------------------------------------------------------

_io = [

    # clocks
    ("clk125", 0, Pins("M23"), IOStandard("SSTL-135")),
    ("clk_pcie1", 0, 
        Subsignal("p", Pins("AF6")),
        Subsignal("n", Pins("AF5")),
        IOStandard("HCSL")
    ),
    ("clk_pcie2", 0, 
        Subsignal("p", Pins("AF34")),
        Subsignal("n", Pins("AF35")),
        IOStandard("HCSL")
    ),
    ("clk_qsfp_osc", 0, 
        Subsignal("p", Pins("T7")),
        Subsignal("n", Pins("T6")),
        IOStandard("LVDS")
    ),

#    ("clk_ddr", 0, 
#        Subsignal("p", Pins("J23")),
#        Subsignal("n", Pins("J24")),
#        IOStandard("DIFFERENTIAL 1.35-V SSTL"), 
#        Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")
 #   )

    # leds
    ("leds", 7, Pins("A8"), IOStandard("2.5 V")),
    ("leds", 6, Pins("B8"), IOStandard("2.5 V")),
    ("leds", 5, Pins("C8"), IOStandard("2.5 V")),
    ("leds", 4, Pins("C9"), IOStandard("2.5 V")),
    ("leds", 3, Pins("C10"), IOStandard("2.5 V")),
    ("leds", 2, Pins("B10"), IOStandard("2.5 V")),
    ("leds", 1, Pins("A10"), IOStandard("2.5 V")),
    ("leds", 0, Pins("A11"), IOStandard("2.5 V")),

    
    #enable to do UART over SDA/SCL pins of QSFP0  
    ("serial", 0,
        Subsignal("tx", Pins("AB24")), # scl - qsfp0
        Subsignal("rx", Pins("AC24")), # sda - qsfp0
        IOStandard("3.3-V LVCMOS")
    ),

    # I2C (OSC3 - used for QSFP. Also labelled 'IDT' in other sources)
    ("i2c_qsfp_osc", 0,
        Subsignal("scl", 0, Pins("N7")),
        Subsignal("sda", 0, Pins("P7")),
        IOStandard("2.5 V"),
        Misc("SLEWRATE 0"),
        Misc("CURRENT_STRENGTH_NEW 8MA")
    ),
    
    # I2C (QSFP0)
    ("i2c_qsfp0", 0,
        Subsignal("scl", 0, Pins("AB24")),
        Subsignal("sda", 0, Pins("AC24")),
        IOStandard("2.5 V"),
        Misc("SLEWRATE 0"),
        Misc("CURRENT_STRENGTH_NEW 8MA")
    ),

    # I2C (QSFP1)
    ("i2c_qsfp1", 0,
        Subsignal("scl", 0, Pins("AA25")),
        Subsignal("sda", 0, Pins("AB25")),
        IOStandard("2.5 V"),
        Misc("SLEWRATE 0"),
        Misc("CURRENT_STRENGTH_NEW 8MA")
    ),

    # I2C (therm) - local and remote temp sensor (also labelled 'MON' in other sources)
    ("i2c_therm", 0,
        Subsignal("scl", 0, Pins("AW26")),
        Subsignal("sda", 0, Pins("AV26")),
        IOStandard("2.5 V"),
        Misc("SLEWRATE 0"),
        Misc("CURRENT_STRENGTH_NEW 8MA"),
        Misc("WEAK_PULL_UP_RESISTOR ON")
    ),

    # QSFP 0
    ("qsfp0", 0,
        Subsignal("tx_p", Pins("U4 R4 N4 L4")),
        Subsignal("tx_n", Pins("U3 R3 N3 L3")),
        Subsignal("rx_p", Pins("V2 T2 P2 M2")),
        Subsignal("rx_n", Pins("V1 T1 P1 M1")),
        IOStandard("1.5-V PCML")
    ),

    # QSFP 1
    ("qsfp1", 0,
        Subsignal("tx_p", Pins("J4 G4 E4 C4")),
        Subsignal("tx_n", Pins("J3 G3 E3 C3")),
        Subsignal("rx_p", Pins("K2 H2 F2 D2")),
        Subsignal("rx_n", Pins("K1 H1 F1 D1")),
        IOStandard("1.5-V PCML")
    ),

    # PCIe 1
    ("pcie1", 0,
        Subsignal("rst_n", Pins("AB28"), IOStandard("2.5 V")),  # perstn
        #Subsignal("clk_p", Pins("AF6"), IOStandard("HCSL")),    # clk_pcie1 defined under clocks
        #Subsignal("clk_n", Pins("AF5"), IOStandard("HCSL")),                  
        Subsignal("tx_p", Pins("AU4 AR4 AN4 AL4 AG4 AE4 AC4 AA4"), IOStandard("1.5-V PCML")),
        Subsignal("tx_n", Pins("AU3 AR3 AN3 AL3 AG3 AE3 AC3 AA3"), IOStandard("1.5-V PCML")),
        Subsignal("rx_p", Pins("AV2 AT2 AP2 AM2 AH2 AF2 AD2 AB2"), IOStandard("1.5-V PCML")),
        Subsignal("rx_n", Pins("AV1 AT1 AP1 AM1 AH1 AF1 AD1 AB1"), IOStandard("1.5-V PCML")),
    ),

    # PCIe 2
    ("pcie2", 0,
        Subsignal("rst_n", Pins("AC28"), IOStandard("2.5 V")),  # perstn
        #Subsignal("clk_p", Pins("AF34"), IOStandard("HCSL")),   # clk_pcie2 defined under clocks
        #Subsignal("clk_n", Pins("AF35"), IOStandard("HCSL")),
        Subsignal("tx_p", Pins("AU36 AR36 AN36 AL36 AG36 AE36 AC36 AA36"), IOStandard("1.5-V PCML")),
        Subsignal("tx_n", Pins("AU37 AR37 AN37 AL37 AG37 AE37 AC37 AA37"), IOStandard("1.5-V PCML")),
        Subsignal("rx_p", Pins("AV38 AT38 AP38 AM38 AH38 AF38 AD38 AB38"), IOStandard("1.5-V PCML")),
        Subsignal("rx_n", Pins("AV39 AT39 AP39 AM39 AH39 AF39 AD39 AB39"), IOStandard("1.5-V PCML")),
    ),
    
    # DDR3 memory
    ("ddram", 0,
        Subsignal("a", Pins(
            "J27 J21 J29 L28 P26 M26 N25 P25", 
            "N22 N26 K27 L27 N27 M27 N21 K28"), 
            IOStandard("SSTL-135"), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("ba", Pins(
            "J28 K21 L26"), 
            IOStandard("SSTL-135"), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("ras_n", Pins("L21"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("cas_n", Pins("L24"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("we_n", Pins("P23"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("dm", Pins(
            "N33 H34 C34 E30 D28 E27 D25 D22 D21"), 
            IOStandard("SSTL-135"), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITH CALIBRATION\"")),
        Subsignal("dq", Pins(
            "R32 P32 M33 T31 N34 P34 L34 L33 F32",
            "G33 G32 K33 J33 G34 K34 J34 D33 C33",
            "B32 A32 A34 A35 A36 A37 G31 H31 E31",
            "F30 C30 D30 A31 B31 E28 H28 G28 C28",
            "B28 A28 B29 A29 G26 F26 H26 D27 B26",
            "A26 C26 C27 G24 F24 G25 D24 C24 C25",
            "B25 A25 H23 G23 H22 C22 B22 A22 B23",
            "A23 F20 E20 G20 C20 C21 E21 B20 A20"), 
            IOStandard("SSTL-135"), 
            Misc("INPUT_TERMINATION \"PARALLEL 40 OHM WITH CALIBRATION\""), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITH CALIBRATION\"")),
        Subsignal("dqs_p", Pins(
            "N32 F33 E34 D31 H29 G27 E24 F23 G21"), 
            IOStandard("DIFFERENTIAL 1.35-V SSTL"), 
            Misc("INPUT_TERMINATION \"PARALLEL 40 OHM WITH CALIBRATION\""), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITH CALIBRATION\"")),
        Subsignal("dqs_n", Pins(
            "M32 E33 D34 C31 G29 F27 E25 E23 F21"), 
            IOStandard("DIFFERENTIAL 1.35-V SSTL"), 
            Misc("INPUT_TERMINATION \"PARALLEL 40 OHM WITH CALIBRATION\""), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITH CALIBRATION\"")),
        Subsignal("clk_p", Pins("J23"), IOStandard("DIFFERENTIAL 1.35-V SSTL"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("clk_n", Pins("J24"), IOStandard("DIFFERENTIAL 1.35-V SSTL"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("cke", Pins("K24"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("odt", Pins("M21"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("cs_n", Pins("N23"), IOStandard("SSTL-135"), Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITHOUT CALIBRATION\"")),
        Subsignal("reset_n", Pins("L20"), 
            IOStandard("SSTL-135"), 
            Misc("OUTPUT_TERMINATION \"SERIES 40 OHM WITH CALIBRATION\""), 
            Misc("BOARD_MODEL_FAR_PULLUP_R OPEN"), 
            Misc("BOARD_MODEL_NEAR_PULLUP_R OPEN"), 
            Misc("BOARD_MODEL_FAR_PULLDOWN_R OPEN"), 
            Misc("BOARD_MODEL_NEAR_PULLDOWN_R OPEN")),
        Subsignal("oct_rzqin", Pins("B34"), IOStandard("SSTL-135")),
        Misc("PACKAGE_SKEW_COMPENSATION OFF"),
    )

]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    default_clk_name   = "clk125"
    default_clk_period = 1e9/125e6

    def __init__(self, toolchain="quartus"):
        AlteraPlatform.__init__(self, "5SGSMD5K2F40I3L", _io, toolchain="quartus")

        # TODO: possibly overload AlteraToolchain to change the build function and add IP generation there.

        self.add_platform_command("set_global_assignment -name DEVICE 5SGSMD5K2F40I3L")
        self.add_platform_command("set_global_assignment -name FAMILY \"Stratix V\"")
        self.add_platform_command("set_global_assignment -name TOP_LEVEL_ENTITY microsoft_storeypeak")        
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_PIN_COUNT Any")
        self.add_platform_command("set_global_assignment -name STRATIXV_CONFIGURATION_SCHEME \"Passive Serial\"")
        self.add_platform_command("set_global_assignment -name STRATIX_OPTIMIZATION_TECHNIQUE BALANCED")

        self.add_platform_command("set_global_assignment -name RESERVE_DATA0_AFTER_CONFIGURATION \"As input tri-stated\"")
        self.add_platform_command("set_global_assignment -name RESERVE_DATA1_AFTER_CONFIGURATION \"As input tri-stated\"")
        self.add_platform_command("set_global_assignment -name RESERVE_FLASH_NCE_AFTER_CONFIGURATION \"As input tri-stated\"")
        self.add_platform_command("set_global_assignment -name RESERVE_DCLK_AFTER_CONFIGURATION \"Use as programming pin\"")
        self.add_platform_command("set_global_assignment -name RESERVE_ALL_UNUSED_PINS_WEAK_PULLUP \"As input tri-stated with weak pull-up\"")
        self.add_platform_command("set_global_assignment -name RESERVE_NCEO_AFTER_CONFIGURATION \"Use as regular IO\"")

        self.add_platform_command("set_global_assignment -name USE_DLL_FREQUENCY_FOR_DQS_DELAY_CHAIN ON")
        self.add_platform_command("set_global_assignment -name OPTIMIZE_MULTI_CORNER_TIMING ON")

        self.add_platform_command("set_global_assignment -name IP_SEARCH_PATHS ../ddram")

    def create_programmer(self):
        return USBBlaster()

    def do_finalize(self, fragment):
        AlteraPlatform.do_finalize(self, fragment)
        # Clock constraints
        self.add_period_constraint(self.lookup_request("clk_125", 0, loose=True), 1e9/125e6)        
        self.add_period_constraint(self.lookup_request("clk_qsfp_osc", 0, loose=True), 1e9/645e6)   
        self.add_period_constraint(self.lookup_request("clk_pcie1", 0, loose=True), 1e9/100e6)      
        self.add_period_constraint(self.lookup_request("clk_pcie2", 0, loose=True), 1e9/100e6)      

        # Generate PLL clock in STA
        self.toolchain.additional_sdc_commands.append("derive_pll_clocks")
        # Calculates clock uncertainties
        self.toolchain.additional_sdc_commands.append("derive_clock_uncertainty")

