"""Environmental sensing: Sensirion SCD41 (CO2), SHT45 (T/RH), ams AS7341 (11-channel spectral light).

The sensors sit in the mast radiation shield and are read by the SBC over I2C (Qwiic) through the vendor-maintained
Adafruit CircuitPython drivers. `SimulatedEnvSensors` provides spatially varying fields for simulation.
"""
from __future__ import annotations

import math


def vpd_kpa(temp_c: float, rh_pct: float) -> float:
    """Air vapour-pressure deficit (Tetens)."""
    es = 0.6108 * math.exp(17.27 * temp_c / (temp_c + 237.3))
    return round(es * (1 - rh_pct / 100.0), 3)


class EnvSensors:
    """Real hardware. Import errors surface at construction so the caller can fall back or report a sensor fault."""

    def __init__(self, i2c=None):
        import adafruit_as7341
        import adafruit_scd4x
        import adafruit_sht4x
        import board
        self.i2c = i2c or board.I2C()
        self.scd = adafruit_scd4x.SCD4X(self.i2c)
        self.sht = adafruit_sht4x.SHT4x(self.i2c)
        self.spec = adafruit_as7341.AS7341(self.i2c)
        self.scd.start_periodic_measurement()            # new CO2 sample every 5 s
        self._co2 = None

    def read(self, x: float | None = None, y: float | None = None, t: float | None = None) -> dict:
        temp, rh = self.sht.measurements
        if self.scd.data_ready:
            self._co2 = float(self.scd.CO2)
        ch = self.spec.all_channels                       # 8 visible bands, raw counts
        return dict(temperature_c=round(temp, 2), humidity_pct=round(rh, 1), co2_ppm=self._co2,
                    light=float(sum(ch)), light_unit="AS7341 raw counts (sum F1-F8); PPFD needs calibration against a quantum sensor",
                    vpd_kpa=vpd_kpa(temp, rh), source="scd41+sht45+as7341")


class SimulatedEnvSensors:
    """Smooth synthetic climate field: warmer and drier towards the east wall, CO2 depleted mid-house, diurnal light."""

    def __init__(self, room=(12.0, 6.8), seed: int = 0, failed: bool = False):
        import numpy as np
        self.room, self.rng, self.failed = room, np.random.default_rng(seed), failed

    def read(self, x: float, y: float, t: float = 0.0) -> dict:
        if self.failed:
            raise IOError("simulated I2C failure")
        fx, fy = x / self.room[0], y / self.room[1]
        temp = 21.0 + 3.0 * fx + 0.6 * math.sin(2 * math.pi * t / 86400) + self.rng.normal(0, 0.05)
        rh = 72.0 - 10.0 * fx + 3.0 * fy + self.rng.normal(0, 0.4)
        co2 = 620 - 180 * math.exp(-((fx - 0.5) ** 2 + (fy - 0.5) ** 2) / 0.08) + self.rng.normal(0, 8)
        ppfd = 240 + 40 * math.sin(math.pi * fy) + self.rng.normal(0, 4)
        return dict(temperature_c=round(temp, 2), humidity_pct=round(rh, 1), co2_ppm=round(co2, 0), light=round(ppfd, 1),
                    light_unit="umol m-2 s-1 (simulated)", vpd_kpa=vpd_kpa(temp, rh), source="simulated")
