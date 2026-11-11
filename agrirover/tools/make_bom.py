#!/usr/bin/env python3
"""Master bill of materials + cost roll-up for AgriRover GH-1.

Writes manufacturing/bom_master.csv and manufacturing/cost_summary.md.

Honesty rules for this file
  * `verified` says where a price / spec came from: "web 2026-11" = checked against a supplier or manufacturer page
    while this package was written; "memory" = recalled, NOT checked - re-verify before ordering.
  * `origin` is the country of manufacture only where the manufacturer states it; otherwise "not verified".
    A US distributor does not make a part US-made.
  * No stock levels are claimed. Availability must be checked at order time.
"""
import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "manufacturing"

# category, description, manufacturer, mpn, supplier (US unless noted), qty, unit_usd, mfr_hq, origin, verified, lead, alternatives, url
ROWS = [
    # ---------------------------------------------------------------- drivetrain
    ("Drivetrain", "70:1 metal gearmotor 37Dx70L, 12 V, 64 CPR encoder", "Pololu", "4754", "Pololu (Las Vegas, NV)", 2, 60.95, "USA", "not verified (gearmotors are imported)", "web 2026-11", "stock item", "Pololu 4753 (50:1), 4755 (100:1); goBILDA 5203 Yellow Jacket", "https://www.pololu.com/product/4754"),
    ("Drivetrain", "Stamped aluminium L-bracket pair for 37D motors", "Pololu", "1084", "Pololu", 1, 9.95, "USA", "not verified", "memory", "stock item", "machined bracket from 6 mm plate", "https://www.pololu.com/product/1084"),
    ("Drivetrain", "Scooter/skate wheel 100x24 mm, black PU", "Pololu (generic)", "3278", "Pololu", 2, 5.95, "USA", "not verified (listed as generic brand)", "web 2026-11 (item), price memory", "stock item", "any 100 mm PU scooter wheel with 608 seats", "https://www.pololu.com/product/3278"),
    ("Drivetrain", "Aluminium scooter-wheel adapter for 6 mm shaft", "Pololu", "2674", "Pololu", 2, 7.95, "USA", "not verified", "web 2026-11 (item), price memory", "stock item", "-", "https://www.pololu.com/product/2674"),
    ("Drivetrain", "Swivel caster, ~40 mm wheel, plate mount, overall height 65-71 mm", "any", "select at order", "McMaster-Carr / Amazon", 2, 8.00, "-", "not verified", "memory", "stock item", "ball transfer unit for hard floors", "https://www.mcmaster.com/casters/"),
    ("Drivetrain", "Dual motor driver: 2 x TB9051FTG single carrier (2.6 A cont., 4.5-28 V)", "Pololu", "2997", "Pololu", 2, 12.95, "USA", "not verified", "web 2026-11", "stock item", "Pololu DRV8874 carrier 4035; Cytron MDD3A", "https://www.pololu.com/product/2997"),
    # ---------------------------------------------------------------- power
    ("Power", "LiFePO4 pack 12.8 V 12 Ah with BMS/PCM, Powerpole output", "Bioenno Power", "BLF-1212A", "Bioenno Power (CA) / Powerwerx", 1, 124.99, "USA", "not verified (cells imported)", "web 2026-11", "stock item", "Bioenno BLF-1206A (6 Ah, ~3 h); Dakota Lithium 12 V 10 Ah", "https://www.bioennopower.com/products/12volt-12ah-lifepo4-battery"),
    ("Power", "LiFePO4 charger 14.6 V 2 A, 5.5/2.1 mm", "Bioenno Power", "BPC-1502DC", "Bioenno Power", 1, 15.00, "USA", "not verified", "web 2026-11 (bundle price difference)", "stock item", "any 14.6 V CC/CV LiFePO4 charger 2-5 A", "https://www.bioennopower.com"),
    ("Power", "5 V 5.5 A step-down regulator (SBC supply)", "Pololu", "4091 (D36V50F5)", "Pololu", 1, 34.95, "USA", "not verified", "web 2026-11", "stock item", "Pololu D24V50F5 (2851); Mean Well DDR-30G-5", "https://www.pololu.com/product/4091"),
    ("Power", "Emergency-stop push button, 22 mm, twist release, 2 NC", "any (IEC 60947-5-5)", "select at order", "AutomationDirect / McMaster-Carr", 1, 22.00, "-", "not verified", "memory", "stock item", "Schneider XB4/XB5, IDEC XA/XW, Eaton M22", "https://www.automationdirect.com"),
    ("Power", "Mini blade fuses 10 A / 5 A / 3 A + in-line holder near battery", "Littelfuse", "0297 series", "DigiKey / Mouser", 1, 6.00, "USA", "not verified", "memory", "stock item", "Bussmann ATM", "https://www.littelfuse.com"),
    ("Power", "Anderson Powerpole 15/30 A pairs, 18 AWG silicone wire, ferrules, heat-shrink", "Anderson / any", "PP15/30", "Powerwerx / DigiKey", 1, 25.00, "USA", "not verified", "memory", "stock item", "-", "https://powerwerx.com"),
    # ---------------------------------------------------------------- compute + electronics
    ("Compute", "Raspberry Pi 5, 8 GB (4 GB is sufficient for the measured-free estimate; see docs)", "Raspberry Pi Ltd", "SC1112", "PiShop.us / Micro Center / DigiKey", 1, 150.00, "UK", "UK (Sony UK Technology Centre, per Raspberry Pi) - confirm on the box", "web 2026-11: US street price ~130-200 USD and volatile (DRAM shortage)", "allocation risk", "Pi 5 4 GB (~110 USD); Jetson Orin Nano for heavier CV", "https://www.raspberrypi.com/products/raspberry-pi-5/"),
    ("Compute", "Pi 5 active cooler + 128 GB A2 microSD (or NVMe HAT + SSD)", "Raspberry Pi / SanDisk", "SC1148", "PiShop.us", 1, 25.00, "UK / USA", "not verified", "memory", "stock item", "-", "https://www.raspberrypi.com/products/active-cooler/"),
    ("Electronics", "STM32F411CEU6 'Black Pill' module", "WeAct Studio", "Adafruit 4877", "Adafruit / DigiKey", 1, 15.00, "China", "China", "item exists (web 2026-11); price memory - one third-party listing showed a much higher price", "stock item", "ST NUCLEO-F411RE (bench), same MCU", "https://www.adafruit.com/product/4877"),
    ("Electronics", "ARC-1 carrier PCB, 100x82 mm, 2-layer, 1 oz, bare (qty 5 minimum order, per-board cost)", "any PCB fab", "ARC-1 rev A", "OSH Park (US-made) / JLCPCB", 1, 12.00, "-", "depends on fab", "estimate", "5-12 days", "-", "electrical/pcb/fab/arc1_gerbers.zip"),
    ("Electronics", "ARC-1 components (44 BOM lines, see electrical/pcb/fab/arc1_bom.csv)", "various", "-", "DigiKey / Mouser", 1, 48.00, "-", "mixed", "estimate from typical unit prices", "stock items", "-", "electrical/pcb/fab/arc1_bom.csv"),
    ("Electronics", "6-DoF IMU breakout LSM6DSOX (Qwiic)", "Adafruit (ST sensor)", "4438", "Adafruit / DigiKey", 1, 11.95, "USA", "USA (Adafruit states NYC manufacture for its own boards) - confirm", "memory", "stock item", "SparkFun ISM330DHCX", "https://www.adafruit.com/product/4438"),
    # ---------------------------------------------------------------- sensing
    ("Sensing", "2D DToF lidar, 12 m, 10 Hz, UART/USB adapter", "Slamtec", "RPLIDAR C1", "DFRobot US warehouse / RobotShop / Amazon", 1, 80.00, "China", "China", "specs web 2026-11; price memory", "stock item, single source", "LDROBOT STL-19P/LD19; RPLIDAR A1M8 (indoor light only)", "https://www.slamtec.com/en/c1"),
    ("Sensing", "UVC camera 1080p, manual focus/exposure control over V4L2", "Logitech", "C920s (960-001257)", "Amazon / Best Buy / B&H", 2, 60.00, "Switzerland/USA", "not verified", "memory", "stock item", "Raspberry Pi Camera Module 3 (needs libcamera on Ubuntu - risk); Arducam UVC IMX series", "https://www.logitech.com"),
    ("Sensing", "CO2 sensor breakout SCD-41 (photoacoustic NDIR, +/-(40 ppm + 5 %))", "Adafruit (Sensirion sensor)", "5190", "Adafruit / DigiKey", 1, 49.95, "USA", "USA (Adafruit) - confirm; sensor Switzerland", "memory", "stock item", "SparkFun SEN-22396", "https://www.adafruit.com/product/5190"),
    ("Sensing", "T/RH sensor breakout SHT45 (+/-0.1 C, +/-1 %RH typ.)", "Adafruit (Sensirion sensor)", "5665", "Adafruit / DigiKey", 1, 12.50, "USA", "USA (Adafruit) - confirm", "memory", "stock item", "SHT41 (4885)", "https://www.adafruit.com/product/5665"),
    ("Sensing", "11-channel spectral sensor breakout AS7341", "Adafruit (ams OSRAM sensor)", "4698", "Adafruit / Mouser", 1, 29.37, "USA", "USA (Adafruit) - confirm", "web 2026-11 (Mouser)", "stock item", "SparkFun AS7341", "https://www.adafruit.com/product/4698"),
    ("Sensing", "Qwiic/STEMMA QT cables (2 x 500 mm, 3 x 100 mm), 18 % grey card for the camera reference", "Adafruit / any", "4401 etc.", "Adafruit", 1, 14.00, "USA", "not verified", "memory", "stock item", "-", "https://www.adafruit.com"),
    ("Sensing", "Bumper micro-switches, lever, NC, IP67 (2 off)", "Omron / Honeywell", "D2HW / ZD series", "DigiKey", 2, 4.50, "Japan / USA", "not verified", "memory", "stock item", "-", "https://www.digikey.com"),
    # ---------------------------------------------------------------- mechanical (fabricated)
    ("Mechanical", "Lower deck plate, 5052-H32 4.0 mm, laser cut (DXF)", "laser service", "PRT-101", "SendCutSend (Reno, NV) / OSH Cut", 1, 48.00, "USA", "USA (domestic job shop)", "estimate - quote before ordering", "3-7 days", "local sheet-metal shop", "mechanical/export/dxf/lower_plate.dxf"),
    ("Mechanical", "Upper deck plate, 5052-H32 3.0 mm, laser cut", "laser service", "PRT-102", "SendCutSend / OSH Cut", 1, 42.00, "USA", "USA (domestic job shop)", "estimate", "3-7 days", "-", "mechanical/export/dxf/upper_plate.dxf"),
    ("Mechanical", "Electronics tray 3.0 mm + caster shim 2.0 mm, laser cut", "laser service", "PRT-103/104", "SendCutSend / OSH Cut", 1, 24.00, "USA", "USA (domestic job shop)", "estimate", "3-7 days", "-", "mechanical/export/dxf/"),
    ("Mechanical", "Skirts: polycarbonate 2.0 mm, 2 side + 2 end, cut to DXF", "plastics service", "PRT-105/106", "SendCutSend / TAP Plastics", 1, 38.00, "USA", "USA (domestic job shop)", "estimate", "3-7 days", "3 mm ACM panel", "mechanical/export/dxf/"),
    ("Mechanical", "2020 T-slot extrusion: 4 x 100 mm + 1 x 1070 mm, ends tapped M5", "80/20 or Misumi", "20-2020 / HFS5-2020", "80/20 (Columbia City, IN) / Misumi USA / McMaster-Carr", 1, 32.00, "USA / Japan", "80/20: USA; Misumi: not verified", "estimate", "stock - 5 days", "OpenBuilds V-slot 2020", "https://8020.net"),
    ("Mechanical", "2020 corner gussets (4), T-nuts M5 (12), M3/M4/M5 stainless fastener assortment, standoffs, grommets", "various", "-", "McMaster-Carr", 1, 45.00, "USA", "not verified", "estimate", "stock item", "-", "https://www.mcmaster.com"),
    ("Mechanical", "3D-printed parts: battery cradle, camera head, sensor shield, lidar adaptor, skirt corners (4), bumper - ASA ~0.45 kg", "in-house FDM", "STL / 3MF set", "own printer / Xometry / Craftcloud", 1, 18.00, "-", "in-house", "estimate (filament only)", "2 days print time", "PETG for non-UV greenhouses", "mechanical/export/3mf/"),
    ("Mechanical", "Hook-and-loop battery strap 25 mm, cable glands PG7 (3), edge trim, labels", "any", "-", "McMaster-Carr", 1, 15.00, "-", "not verified", "estimate", "stock item", "-", "https://www.mcmaster.com"),
]
HEAD = ["item", "category", "description", "manufacturer", "mpn", "supplier", "qty", "unit_usd", "ext_usd", "manufacturer_hq", "country_of_origin",
        "price_spec_verified", "lead_time_note", "alternatives", "link_or_file"]


def main():
    OUT.mkdir(exist_ok=True)
    cats: dict = {}
    with open(OUT / "bom_master.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEAD)
        for i, r in enumerate(ROWS, 1):
            ext = round(r[5] * r[6], 2)
            cats[r[0]] = cats.get(r[0], 0) + ext
            w.writerow([i, r[0], r[1], r[2], r[3], r[4], r[5], f"{r[6]:.2f}", f"{ext:.2f}", r[7], r[8], r[9], r[10], r[11], r[12]])
    parts = sum(cats.values())
    verified = sum(r[5] * r[6] for r in ROWS if r[9].startswith("web 2026-11") and "price memory" not in r[9])
    labour_h, rate = 16, 45.0
    ship_tax = 0.12 * parts
    proto = parts + ship_tax
    md = ["# AgriRover GH-1 — cost summary\n",
          "Generated by `tools/make_bom.py` from the master BOM. All figures are **estimates in USD, November 2026**, single-unit "
          "prototype quantities, before sales tax unless noted. Prices marked *memory* or *estimate* in `bom_master.csv` were not "
          f"checked against a live listing; only {verified:.0f} USD of the {parts:.0f} USD parts total rests on prices verified on the web "
          "while this package was written. Re-quote everything before purchasing.\n",
          "## Prototype (quantity 1)\n", "| Category | Cost [USD] |", "|---|---:|"]
    for k, v in cats.items():
        md.append(f"| {k} | {v:.0f} |")
    md += [f"| **Parts subtotal** | **{parts:.0f}** |", f"| Shipping, small-order fees, sales tax allowance (12 %) | {ship_tax:.0f} |",
           f"| **Prototype material cost** | **{proto:.0f}** |",
           f"| Assembly + bring-up labour, {labour_h} h (own time; valued at {rate:.0f} USD/h = {labour_h*rate:.0f} USD) | not included above |",
           "",
           f"Battery + charger: {sum(r[5]*r[6] for r in ROWS if 'LiFePO4' in r[1]):.0f} USD. "
           f"Fabricated mechanical parts (laser-cut, extrusion, printed): {sum(r[5]*r[6] for r in ROWS if r[0]=='Mechanical'):.0f} USD. "
           f"ARC-1 board (bare PCB share + components): {sum(r[5]*r[6] for r in ROWS if 'ARC-1' in r[1]):.0f} USD.",
           "\n## Estimated production cost (batch of 25)\n",
           "Assumptions (not quotes): 15 % volume discount on catalogue parts, laser-cut and PCB costs at −40 % (set-up amortised), "
           "PCB assembled by a contract manufacturer (+18 USD/board), 4 h assembly and test per unit at 45 USD/h.\n",
           "| Item | Per unit [USD] |", "|---|---:|"]
    cat_parts = sum(v for k, v in cats.items() if k != "Mechanical") - sum(r[5] * r[6] for r in ROWS if "ARC-1" in r[1])
    mech = cats["Mechanical"]
    pcb = sum(r[5] * r[6] for r in ROWS if "ARC-1" in r[1])
    prod = cat_parts * 0.85 + mech * 0.6 + pcb * 0.6 + 18 + 4 * rate
    md += [f"| Catalogue parts (−15 %) | {cat_parts*0.85:.0f} |", f"| Fabricated mechanical (−40 %) | {mech*0.6:.0f} |",
           f"| ARC-1 PCB + parts (−40 %) + contract assembly | {pcb*0.6+18:.0f} |", f"| Assembly and test labour (4 h) | {4*rate:.0f} |",
           f"| **Estimated unit cost at 25 units** | **{prod:.0f}** |", "",
           "## Long-lead, single-source and obsolescence watch list\n",
           "| Part | Risk | Mitigation |", "|---|---|---|",
           "| Raspberry Pi 5 | Price rose repeatedly through 2026 (DRAM shortage); allocation possible | 4 GB variant is sufficient; software has no Pi-specific code outside the camera/GPIO setup |",
           "| RPLIDAR C1 | Single manufacturer (Slamtec, China); driver tied to vendor SDK | LD19/STL-19P is a drop-in class alternative (different ROS driver) |",
           "| WeAct Black Pill | Hobby-grade module, no long-term supply commitment, clones with different flash sizes exist | Socketed on ARC-1; NUCLEO-F411RE runs the same firmware; rev B can place the MCU directly |",
           "| Pololu 37D gearmotor | Single source; output-bushing radial load not specified | goBILDA 5203 series as second source (different bracket); outboard bearing provision |",
           "| Bioenno BLF-1212A | Discharge-current rating differs between resellers | Verify the label on the delivered pack; fuse F1 (10 A) is well below either figure |",
           "| TB9051FTG carriers | Toshiba automotive IC, active; carrier is Pololu-only | DRV8874 carrier fits the same 12-pin interface with a wiring change (documented in docs/04) |",
           "| AMS1117-3.3, 2N7002, passives | Multi-source jellybean parts | none needed |",
           "",
           "No part in the BOM is known to be obsolete or NRND as of the checks above; lifecycle status of the ARC-1 semiconductor "
           "lines must be confirmed on the distributor pages when the order is built.", ""]
    (OUT / "cost_summary.md").write_text("\n".join(md))
    print(f"parts {parts:.0f} USD, prototype {proto:.0f} USD, production est. {prod:.0f} USD/unit, verified share {verified:.0f} USD")


if __name__ == "__main__":
    main()
