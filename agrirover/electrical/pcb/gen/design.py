"""AgriRover GH-1 control & power carrier board (ARC-1, rev A) — design source.

This module is the single source of truth for the board netlist. The
schematic (.kicad_sch), the board (.kicad_pcb), the BOM and the netlist
checks are all generated from the tables below. Pure Python, no deps.

Component tuple: (ref, lib_id, value, footprint, {pin: net}, mpn, manufacturer, description)
A pin mapped to "NC" gets a no-connect flag.
"""

FP = dict(
    R="Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
    C="Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",
    C1206="Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder",
    LED="LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder",
    SOD="Diode_SMD:D_SOD-123",
    SMB="Diode_SMD:D_SMB",
    DPAK="Package_TO_SOT_SMD:TO-252-2",
    SOT23="Package_TO_SOT_SMD:SOT-23",
    SOT223="Package_TO_SOT_SMD:SOT-223-3_TabPin2",
    SOIC8="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
    SHUNT="Resistor_SMD:R_2512_6332Metric",
    FUSE="Fuse:Fuseholder_Blade_Mini_Keystone_3568",
    TSR="Converter_DCDC:Converter_DCDC_TRACO_TSR-1_THT",
    TANT="Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B",
    ELEC="Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
    MSTB2="Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal",
    XH2="Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
    XH3="Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical",
    XH4="Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
    QWIIC="Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
    H1x3="Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    H1x4="Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
    H2x3="Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
    H2x6="Connector_PinHeader_2.54mm:PinHeader_2x06_P2.54mm_Vertical",
    S1x20="Connector_PinSocket_2.54mm:PinSocket_1x20_P2.54mm_Vertical",
    HOLE="MountingHole:MountingHole_3.2mm_M3",
)

# WeAct "Black Pill" STM32F411CEU6 header order, USB connector at the top.
# !! Transcribed from memory of the V3.x silkscreen and NOT verified against a
# !! physical board in this project. Check before ordering the PCB; a change
# !! only requires editing these two lists and re-running build.sh.
BLACKPILL_LEFT = ["B12", "B13", "B14", "B15", "A8", "A9", "A10", "A11", "A12", "A15",
                  "B3", "B4", "B5", "B6", "B7", "B8", "B9", "5V", "GND", "3V3"]
BLACKPILL_RIGHT = ["5V", "GND", "3V3", "B10", "B2", "B1", "B0", "A7", "A6", "A5",
                   "A4", "A3", "A2", "A1", "A0", "NRST", "C15", "C14", "C13", "VBAT"]

# MCU pin -> net (see docs/04_electrical.md for the peripheral mapping rationale)
MCU_NETS = {
    "A0": "BUMP", "A1": "MOTOR_EN", "A2": "UART_TX", "A3": "UART_RX",
    "A4": "M1_OCM", "A5": "M2_OCM", "A6": "ENCR_A", "A7": "ENCR_B",
    "A8": "I2C_SCL", "A9": "DBG_TX", "A10": "DBG_RX", "A15": "ENCL_A",
    "B0": "VMOT_SENSE", "B1": "M_EN", "B3": "ENCL_B", "B4": "I2C_SDA", "B5": "ESTOP_SENSE",
    "B6": "M1_PWM1", "B7": "M1_PWM2", "B8": "M2_PWM1", "B9": "M2_PWM2", "B10": "M_DIAG",
    "B12": "SPI_NSS", "B13": "SPI_SCK", "B14": "SPI_MISO", "B15": "SPI_MOSI",
    "5V": "+5V", "GND": "GND",
}


def _row(names):
    return {str(i + 1): MCU_NETS.get(n, "NC") for i, n in enumerate(names)}


def _r(ref, val, a, b, mpn="", d="resistor 0805 1 %"):
    return (ref, "Device:R", val, FP["R"], {"1": a, "2": b}, mpn, "Yageo / any", d)


def _c(ref, val, a, b, fp="C", d="MLCC X7R"):
    return (ref, "Device:C", val, FP[fp], {"1": a, "2": b}, "", "any", d)


COMPONENTS = [
    # ------------------------------------------------------------ power input & protection
    ("J1", "Connector_Generic:Conn_01x02", "BATTERY 12.8V", FP["MSTB2"], {"1": "BAT_RAW", "2": "GND"},
     "1757242", "Phoenix Contact", "MSTBA 2,5/2-G-5,08 header, 12 A"),
    ("F1", "Device:Fuse", "10A mini blade", FP["FUSE"], {"1": "BAT_RAW", "2": "BAT_FUSED"},
     "3568", "Keystone", "mini blade fuse holder + 10 A fuse (Littelfuse 0297010)"),
    ("Q1", "Device:Q_PMOS_GDS", "DMP4015SK3", FP["DPAK"], {"1": "Q1_G", "2": "BAT_FUSED", "3": "BAT_PROT"},
     "DMP4015SK3-13", "Diodes Inc.", "P-MOSFET -40 V, reverse-polarity protection"),
    _r("R1", "10k", "Q1_G", "GND"),
    ("D1", "Device:D_Zener", "12V", FP["SOD"], {"1": "BAT_PROT", "2": "Q1_G"}, "MMSZ5242B", "onsemi / Diodes", "zener 12 V gate clamp"),
    ("R2", "Device:R", "5mR 1% 2W", FP["SHUNT"], {"1": "BAT_PROT", "2": "VBAT"}, "WSL25125L000FEA18", "Vishay", "current shunt 2512"),
    ("U2", "Sensor_Energy:INA219AxD", "INA219AIDR", FP["SOIC8"],
     {"8": "BAT_PROT", "7": "VBAT", "5": "+3V3", "6": "GND", "3": "I2C_SDA", "4": "I2C_SCL", "1": "GND", "2": "GND"},
     "INA219AIDR", "Texas Instruments", "bus voltage / current monitor, I2C 0x40"),
    _c("C1", "100n", "+3V3", "GND"),
    ("D2", "Device:D_TVS", "SMBJ16A", FP["SMB"], {"1": "VBAT", "2": "GND"}, "SMBJ16A", "Littelfuse", "TVS 600 W, unidirectional (pad 1 = cathode)"),
    _c("C2", "10u 25V", "VBAT", "GND", "C1206"),
    # ------------------------------------------------------------ fused outputs
    ("F2", "Device:Fuse", "5A mini blade", FP["FUSE"], {"1": "VBAT", "2": "VSBC"}, "3568", "Keystone", "SBC buck feed fuse"),
    ("J6", "Connector_Generic:Conn_01x02", "SBC DC/DC OUT", FP["MSTB2"], {"1": "VSBC", "2": "GND"}, "1757242", "Phoenix Contact", ""),
    ("F3", "Device:Fuse", "3A mini blade", FP["FUSE"], {"1": "VBAT", "2": "VAUX"}, "3568", "Keystone", "auxiliary feed fuse"),
    ("J7", "Connector_Generic:Conn_01x02", "AUX OUT", FP["MSTB2"], {"1": "VAUX", "2": "GND"}, "1757242", "Phoenix Contact", ""),
    # ------------------------------------------------------------ motor power switch / E-stop chain
    ("Q2", "Device:Q_PMOS_GDS", "DMP4015SK3", FP["DPAK"], {"1": "MOT_GATE", "2": "VMOT", "3": "VBAT"},
     "DMP4015SK3-13", "Diodes Inc.", "P-MOSFET motor-rail high-side switch"),
    _r("R3", "100k", "VBAT", "MOT_GATE"),
    _c("C3", "2u2 25V", "VBAT", "MOT_GATE", "C1206", "MLCC X7R, sets ~0.3 s turn-off delay"),
    ("D3", "Device:D_Zener", "12V", FP["SOD"], {"1": "VBAT", "2": "MOT_GATE"}, "MMSZ5242B", "onsemi / Diodes", "zener 12 V gate clamp"),
    _r("R4", "10k", "MOT_GATE", "ESTOP_A"),
    ("J14", "Connector_Generic:Conn_01x04", "ESTOP 2xNC", FP["XH4"],
     {"1": "ESTOP_A", "2": "ESTOP_B", "3": "ESTOP_SENSE", "4": "GND"}, "B4B-XH-A", "JST", "E-stop: pole 1 = pins 1-2, pole 2 = pins 3-4"),
    ("Q3", "Transistor_FET:2N7002", "2N7002", FP["SOT23"], {"1": "MOTEN_G", "2": "GND", "3": "ESTOP_B"}, "2N7002", "onsemi / Nexperia", "N-MOSFET, MCU motor-rail enable"),
    _r("R5", "1k", "MOTOR_EN", "MOTEN_G"),
    _r("R6", "100k", "MOTEN_G", "GND"),
    _r("R7", "10k", "+3V3", "ESTOP_SENSE"),
    _c("C4", "100n", "ESTOP_SENSE", "GND"),
    ("J5", "Connector_Generic:Conn_01x02", "VMOT OUT", FP["MSTB2"], {"1": "VMOT", "2": "GND"}, "1757242", "Phoenix Contact", "to motor-driver carriers"),
    ("C5", "Device:C_Polarized", "470u 25V", FP["ELEC"], {"1": "VMOT", "2": "GND"}, "EEU-FR1E471", "Panasonic", "low-ESR electrolytic"),
    _c("C6", "100n 50V", "VMOT", "GND"),
    _r("R8", "100k", "VMOT", "VMOT_SENSE"),
    _r("R9", "22k", "VMOT_SENSE", "GND"),
    _c("C7", "100n", "VMOT_SENSE", "GND"),
    ("D4", "Device:LED", "RED (VMOT)", FP["LED"], {"1": "GND", "2": "VMOT_LED"}, "", "any", "LED 0805"),
    _r("R10", "4k7", "VMOT", "VMOT_LED"),
    # ------------------------------------------------------------ logic supplies
    ("U3", "Regulator_Switching:TSR_1-2450", "TSR 1-2450", FP["TSR"], {"1": "VBAT", "2": "GND", "3": "+5V"},
     "TSR 1-2450", "Traco Power", "5 V 1 A switching regulator, SIP-3"),
    _c("C8", "10u 25V", "+5V", "GND", "C1206"),
    ("U4", "Regulator_Linear:AMS1117-3.3", "AMS1117-3.3", FP["SOT223"], {"3": "+5V", "1": "GND", "2": "+3V3"},
     "AMS1117-3.3", "AMS / equivalent (LM1117, AZ1117)", "3.3 V 1 A LDO"),
    ("C9", "Device:C_Polarized", "22u 10V", FP["TANT"], {"1": "+3V3", "2": "GND"}, "T491B226K010AT", "KEMET", "tantalum, LDO output"),
    _c("C10", "100n", "+3V3", "GND"),
    ("D5", "Device:LED", "GREEN (5V)", FP["LED"], {"1": "GND", "2": "PWR_LED"}, "", "any", "LED 0805"),
    _r("R11", "1k", "+5V", "PWR_LED"),
    # ------------------------------------------------------------ MCU module sockets
    ("A1", "Connector_Generic:Conn_01x20", "BlackPill F411 (left row)", FP["S1x20"], _row(BLACKPILL_LEFT),
     "4877", "WeAct (via Adafruit)", "STM32F411CEU6 module, socketed"),
    ("A2", "Connector_Generic:Conn_01x20", "BlackPill F411 (right row)", FP["S1x20"], _row(BLACKPILL_RIGHT),
     "", "", "second socket strip of A1"),
    # ------------------------------------------------------------ encoders
    ("J2", "Connector_Generic:Conn_01x04", "ENC LEFT", FP["XH4"], {"1": "+5V", "2": "GND", "3": "ENCL_A_IN", "4": "ENCL_B_IN"}, "B4B-XH-A", "JST", ""),
    _r("R12", "1k", "ENCL_A_IN", "ENCL_A"), _r("R13", "1k", "ENCL_B_IN", "ENCL_B"),
    _c("C11", "1n", "ENCL_A", "GND"), _c("C12", "1n", "ENCL_B", "GND"),
    ("J3", "Connector_Generic:Conn_01x04", "ENC RIGHT", FP["XH4"], {"1": "+5V", "2": "GND", "3": "ENCR_A_IN", "4": "ENCR_B_IN"}, "B4B-XH-A", "JST", ""),
    _r("R14", "1k", "ENCR_A_IN", "ENCR_A"), _r("R15", "1k", "ENCR_B_IN", "ENCR_B"),
    _c("C13", "1n", "ENCR_A", "GND"), _c("C14", "1n", "ENCR_B", "GND"),
    # ------------------------------------------------------------ motor driver interface (2 x TB9051FTG carriers)
    ("J4", "Connector_Generic:Conn_02x06_Odd_Even", "MOTOR DRIVER IF", FP["H2x6"],
     {"1": "+3V3", "2": "GND", "3": "M1_PWM1", "4": "M1_PWM2", "5": "M2_PWM1", "6": "M2_PWM2",
      "7": "M_EN", "8": "M_DIAG", "9": "M1_OCM_IN", "10": "M2_OCM_IN", "11": "+5V", "12": "GND"}, "", "any", "2x6 pin header 2.54 mm"),
    _r("R16", "100k", "M_EN", "GND"), _r("R17", "10k", "+3V3", "M_DIAG"),
    _r("R18", "1k", "M1_OCM_IN", "M1_OCM"), _c("C15", "100n", "M1_OCM", "GND"),
    _r("R19", "1k", "M2_OCM_IN", "M2_OCM"), _c("C16", "100n", "M2_OCM", "GND"),
    # ------------------------------------------------------------ I2C sensor bus
    ("J8", "Connector_Generic:Conn_01x04", "QWIIC 1", FP["QWIIC"], {"1": "GND", "2": "+3V3", "3": "I2C_SDA", "4": "I2C_SCL", "MP": "GND"}, "SM04B-SRSS-TB", "JST", "Qwiic / STEMMA QT"),
    ("J9", "Connector_Generic:Conn_01x04", "QWIIC 2", FP["QWIIC"], {"1": "GND", "2": "+3V3", "3": "I2C_SDA", "4": "I2C_SCL", "MP": "GND"}, "SM04B-SRSS-TB", "JST", "Qwiic / STEMMA QT"),
    ("J10", "Connector_Generic:Conn_01x04", "I2C HDR", FP["H1x4"], {"1": "+3V3", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA"}, "", "any", "IMU breakout header"),
    _r("R20", "2k2", "+3V3", "I2C_SCL"), _r("R21", "2k2", "+3V3", "I2C_SDA"),
    # ------------------------------------------------------------ serial / expansion
    ("J11", "Connector_Generic:Conn_01x03", "SBC UART", FP["XH3"], {"1": "GND", "2": "UART_TX_O", "3": "UART_RX_I"}, "B3B-XH-A", "JST", "3.3 V UART to the SBC"),
    _r("R22", "330R", "UART_TX", "UART_TX_O"), _r("R23", "330R", "UART_RX", "UART_RX_I"),
    ("J12", "Connector_Generic:Conn_01x03", "DEBUG UART", FP["H1x3"], {"1": "GND", "2": "DBG_TX", "3": "DBG_RX"}, "", "any", ""),
    ("J13", "Connector_Generic:Conn_02x03_Odd_Even", "SPI EXP", FP["H2x3"],
     {"1": "+3V3", "2": "GND", "3": "SPI_SCK", "4": "SPI_MISO", "5": "SPI_MOSI", "6": "SPI_NSS"}, "", "any", "expansion (e.g. MCP2515 CAN)"),
    ("J15", "Connector_Generic:Conn_01x02", "BUMPER NC", FP["XH2"], {"1": "BUMP", "2": "GND"}, "B2B-XH-A", "JST", "series NC bumper switches"),
    _r("R24", "10k", "+3V3", "BUMP"), _c("C17", "100n", "BUMP", "GND"),
    # ------------------------------------------------------------ mechanical / ERC helpers
    ("H1", "Mechanical:MountingHole", "M3", FP["HOLE"], {}, "", "", ""),
    ("H2", "Mechanical:MountingHole", "M3", FP["HOLE"], {}, "", "", ""),
    ("H3", "Mechanical:MountingHole", "M3", FP["HOLE"], {}, "", "", ""),
    ("H4", "Mechanical:MountingHole", "M3", FP["HOLE"], {}, "", "", ""),
    ("#FLG01", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "GND"}, "", "", ""),
    ("#FLG02", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VBAT"}, "", "", ""),
]

# Net classes drive track widths in the router (mm).
HI_NETS = {"BAT_RAW", "BAT_FUSED", "BAT_PROT", "VBAT", "VMOT"}     # battery / motor current path
PWR_NETS = {"+5V", "+3V3", "VSBC", "VAUX"}
WIDTH = {"hi": 2.5, "pwr": 0.6, "sig": 0.25}
WIDTH_OVERRIDE = {"VSBC": 2.0, "VAUX": 1.5}
# Pads on high-current nets that only carry signal-level current attach with thin tracks.
THIN_REFS = {"U2", "D1", "R3", "C3", "D3", "C2", "D2", "R8", "R10", "C6", "U3", "R1"}

# Schematic sheet groups: (title, [refs]) laid out left-to-right, top-to-bottom.
GROUPS = [
    ("POWER INPUT, REVERSE-POLARITY PROTECTION, MONITOR", ["J1", "F1", "Q1", "R1", "D1", "R2", "U2", "C1", "D2", "C2", "#FLG01", "#FLG02"]),
    ("FUSED OUTPUTS", ["F2", "J6", "F3", "J7"]),
    ("MOTOR RAIL SWITCH + E-STOP CHAIN (controlled stop, delayed cut)", ["Q2", "R3", "C3", "D3", "R4", "J14", "Q3", "R5", "R6", "R7", "C4", "J5", "C5", "C6", "R8", "R9", "C7", "D4", "R10"]),
    ("LOGIC SUPPLIES", ["U3", "C8", "U4", "C9", "C10", "D5", "R11"]),
    ("MCU MODULE (WeAct BlackPill STM32F411CEU6, socketed)", ["A1", "A2"]),
    ("ENCODERS", ["J2", "R12", "R13", "C11", "C12", "J3", "R14", "R15", "C13", "C14"]),
    ("MOTOR DRIVER INTERFACE", ["J4", "R16", "R17", "R18", "C15", "R19", "C16"]),
    ("I2C SENSOR BUS", ["J8", "J9", "J10", "R20", "R21"]),
    ("UART / SPI / BUMPER", ["J11", "R22", "R23", "J12", "J13", "J15", "R24", "C17"]),
    ("MECHANICAL", ["H1", "H2", "H3", "H4"]),
]


def nets() -> dict:
    """net name -> sorted list of (ref, pin)."""
    out: dict = {}
    for ref, _lib, _val, _fp, pins, *_ in COMPONENTS:
        for pin, net in pins.items():
            if net != "NC":
                out.setdefault(net, []).append((ref, pin))
    return {k: sorted(v) for k, v in sorted(out.items())}


def net_class(net: str) -> str:
    return "hi" if net in HI_NETS else "pwr" if net in PWR_NETS else "sig"


if __name__ == "__main__":
    n = nets()
    print(len(COMPONENTS), "components,", len(n), "nets")
    for k, v in n.items():
        if len(v) < 2:
            print("SINGLE-PIN NET:", k, v)
