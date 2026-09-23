"""Generate a deterministic IMU log for smoke-testing the filter pipeline."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import random


def generate(
    path: Path,
    sample_rate_hz: float,
    duration_s: float,
    gyro_noise_std_dps: float = 0.03,
    accel_noise_std_g: float = 0.002,
    mag_noise_std: float = 0.005,
    seed: int = 42,
) -> int:
    random.seed(seed)
    count = int(sample_rate_hz * duration_s)
    period_s = 1.0 / sample_rate_hz

    with path.open("w", newline="", encoding="utf-8") as destination:
        fieldnames = [
            "timestamp_s",
            "gx_dps",
            "gy_dps",
            "gz_dps",
            "ax_g",
            "ay_g",
            "az_g",
            "mx",
            "my",
            "mz",
            "true_yaw_deg",
        ]
        writer = csv.DictWriter(destination, fieldnames=fieldnames)
        writer.writeheader()

        yaw_deg = 0.0
        for index in range(count):
            timestamp_s = index * period_s
            yaw_rate_dps = 90.0 if 4.0 <= timestamp_s < 5.0 else 0.0
            yaw_deg += yaw_rate_dps * period_s
            yaw_rad = math.radians(yaw_deg)

            writer.writerow(
                {
                    "timestamp_s": timestamp_s,
                    "gx_dps": random.gauss(0.0, gyro_noise_std_dps),
                    "gy_dps": random.gauss(0.0, gyro_noise_std_dps),
                    "gz_dps": yaw_rate_dps + random.gauss(0.0, gyro_noise_std_dps),
                    "ax_g": random.gauss(0.0, accel_noise_std_g),
                    "ay_g": random.gauss(0.0, accel_noise_std_g),
                    "az_g": 1.0 + random.gauss(0.0, accel_noise_std_g),
                    "mx": math.cos(yaw_rad) + random.gauss(0.0, mag_noise_std),
                    "my": -math.sin(yaw_rad) + random.gauss(0.0, mag_noise_std),
                    "mz": random.gauss(0.0, mag_noise_std),
                    "true_yaw_deg": yaw_deg,
                }
            )
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--gyro-noise-std", type=float, default=0.03)
    parser.add_argument("--accel-noise-std", type=float, default=0.002)
    parser.add_argument("--mag-noise-std", type=float, default=0.005)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    count = generate(
        args.output,
        args.sample_rate,
        args.duration,
        args.gyro_noise_std,
        args.accel_noise_std,
        args.mag_noise_std,
        args.seed,
    )
    print(f"generated {count} samples -> {args.output}")


if __name__ == "__main__":
    main()
