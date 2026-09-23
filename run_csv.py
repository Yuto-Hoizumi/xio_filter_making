"""Run the CanSat AHRS filter against a timestamped CSV log."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import math
from pathlib import Path
from typing import Optional

from cansat_ahrs import AhrsConfig, CanSatAhrs


REQUIRED_COLUMNS = ("timestamp_s", "gx_dps", "gy_dps", "gz_dps", "ax_g", "ay_g", "az_g")
MAGNETOMETER_COLUMNS = ("mx", "my", "mz")


def _optional_float(row: dict[str, str], key: str) -> Optional[float]:
    value = row.get(key, "").strip()
    return None if value == "" else float(value)


def _angle_error_deg(estimated_deg: float, true_deg: float) -> float:
    """Return the shortest signed angle from truth to estimate."""
    return (estimated_deg - true_deg + 180.0) % 360.0 - 180.0


def process_csv(
    input_path: Path,
    output_path: Path,
    config: AhrsConfig,
) -> tuple[int, list[float]]:
    filter_ = CanSatAhrs(config)
    count = 0
    yaw_errors_deg: list[float] = []

    with input_path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        missing = [column for column in REQUIRED_COLUMNS if column not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(missing)}")

        has_magnetometer = all(column in (reader.fieldnames or []) for column in MAGNETOMETER_COLUMNS)

        with output_path.open("w", newline="", encoding="utf-8") as destination:
            writer = None
            for row in reader:
                magnetometer = None
                if has_magnetometer:
                    values = [_optional_float(row, column) for column in MAGNETOMETER_COLUMNS]
                    if all(value is not None for value in values):
                        magnetometer = values

                output = filter_.update(
                    timestamp_s=float(row["timestamp_s"]),
                    gyroscope_dps=[float(row[column]) for column in ("gx_dps", "gy_dps", "gz_dps")],
                    accelerometer_g=[float(row[column]) for column in ("ax_g", "ay_g", "az_g")],
                    magnetometer=magnetometer,
                    external_heading_deg=_optional_float(row, "external_heading_deg"),
                )

                record = asdict(output)
                qw, qx, qy, qz = record.pop("quaternion_wxyz")
                eax, eay, eaz = record.pop("earth_acceleration_g")
                record.update(
                    qw=qw,
                    qx=qx,
                    qy=qy,
                    qz=qz,
                    earth_ax_g=eax,
                    earth_ay_g=eay,
                    earth_az_g=eaz,
                )

                true_yaw_deg = _optional_float(row, "true_yaw_deg")
                if true_yaw_deg is not None:
                    yaw_error_deg = _angle_error_deg(output.yaw_deg, true_yaw_deg)
                    record["true_yaw_deg"] = true_yaw_deg
                    record["yaw_error_deg"] = yaw_error_deg
                    yaw_errors_deg.append(yaw_error_deg)

                if writer is None:
                    writer = csv.DictWriter(destination, fieldnames=record.keys())
                    writer.writeheader()
                writer.writerow(record)
                count += 1

    return count, yaw_errors_deg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="timestamped raw IMU CSV")
    parser.add_argument("output", type=Path, help="filtered output CSV")
    parser.add_argument("--sample-rate", type=float, default=100.0, help="nominal sample rate in Hz")
    parser.add_argument("--gyro-range", type=float, default=2000.0, help="configured gyroscope range in degrees/s")
    args = parser.parse_args()

    count, yaw_errors_deg = process_csv(
        args.input,
        args.output,
        AhrsConfig(
            sample_rate_hz=args.sample_rate,
            gyroscope_range_dps=args.gyro_range,
        ),
    )
    print(f"processed {count} samples -> {args.output}")
    if yaw_errors_deg:
        mean_absolute_error_deg = sum(abs(value) for value in yaw_errors_deg) / len(yaw_errors_deg)
        root_mean_square_error_deg = math.sqrt(
            sum(value * value for value in yaw_errors_deg) / len(yaw_errors_deg)
        )
        maximum_absolute_error_deg = max(abs(value) for value in yaw_errors_deg)
        print(f"yaw_mean_absolute_error_deg: {mean_absolute_error_deg:.6f}")
        print(f"yaw_root_mean_square_error_deg: {root_mean_square_error_deg:.6f}")
        print(f"yaw_maximum_absolute_error_deg: {maximum_absolute_error_deg:.6f}")


if __name__ == "__main__":
    main()
