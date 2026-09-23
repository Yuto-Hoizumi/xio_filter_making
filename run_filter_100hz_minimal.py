"""Run the CanSat AHRS with fixed mock sensor values at about 100 Hz."""

import time

from cansat_ahrs import AhrsConfig, CanSatAhrs


SAMPLE_RATE_HZ = 100.0
PERIOD_S = 1.0 / SAMPLE_RATE_HZ

filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=SAMPLE_RATE_HZ))
program_start_s = time.perf_counter()

print("Running mock 9-axis filter at about 100 Hz. Press Ctrl+C to stop.")

try:
    while True:
        loop_start_s = time.perf_counter()

        filter_.update(
            timestamp_s=loop_start_s - program_start_s,
            gyroscope_dps=(0.0, 0.0, 0.0),
            accelerometer_g=(0.0, 0.0, 1.0),
            magnetometer=(1.0, 0.0, 0.0),
        )

        remaining_s = PERIOD_S - (time.perf_counter() - loop_start_s)
        if remaining_s > 0:
            time.sleep(remaining_s)
except KeyboardInterrupt:
    print("Stopped.")
