#!/usr/bin/env python3
"""Bench console for the base controller: prints telemetry and accepts simple commands.

    python3 tools/base_console.py /dev/ttyAMA0
    commands:  e = enable   d = disable   c = clear fault   w <left> <right> = wheel speeds [rad/s]   q = quit
Wheel commands are re-sent at 20 Hz until changed (the firmware stops after 300 ms without one).
"""
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))
from agrirover.hw.base_driver import BaseDriver  # noqa: E402


def main():
    drv = BaseDriver(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0")
    drv.request_info()
    w, t_print, t_cmd = (0.0, 0.0), 0.0, 0.0
    print(__doc__)
    while True:
        st = drv.poll()
        now = time.monotonic()
        if now - t_cmd > 0.05:
            drv.set_wheels(*w)
            t_cmd = now
        if st and now - t_print > 0.5:
            t_print = now
            print(f"{st.state_name:9s} faults={','.join(st.fault_names) or '-':18s} vbat={st.vbat_mv/1000:5.2f} V ibat={st.ibat_ma/1000:5.2f} A "
                  f"vmot={st.vmot_mv/1000:5.2f} V  w=({st.w_left:+5.2f},{st.w_right:+5.2f}) rad/s  enc=({st.enc_left},{st.enc_right}) "
                  f"i=({st.i_left_ma},{st.i_right_ma}) mA  fw={drv.info}")
        if select.select([sys.stdin], [], [], 0.01)[0]:
            cmd = sys.stdin.readline().split()
            if not cmd:
                continue
            if cmd[0] == "q":
                drv.set_wheels(0, 0)
                drv.enable(False)
                return
            if cmd[0] == "e":
                drv.enable(True)
            elif cmd[0] == "d":
                drv.enable(False)
            elif cmd[0] == "c":
                drv.clear_fault()
            elif cmd[0] == "w" and len(cmd) == 3:
                w = (max(-6.0, min(6.0, float(cmd[1]))), max(-6.0, min(6.0, float(cmd[2]))))     # bench limit 0.3 m/s


if __name__ == "__main__":
    main()
