"""Run the mock CanSat AHRS workload at a real-time update rate."""

from __future__ import annotations

import argparse
import time

from cansat_ahrs import AhrsConfig, CanSatAhrs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    if args.sample_rate <= 0:
        parser.error("--sample-rate must be positive")
    if args.duration <= 0:
        parser.error("--duration must be positive")

    sample_count = max(1, round(args.sample_rate * args.duration))
    period_s = 1.0 / args.sample_rate
    filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=args.sample_rate))

    wall_start_s = time.perf_counter()
    cpu_start_s = time.process_time()
    next_release_s = wall_start_s + period_s

    compute_total_s = 0.0
    compute_max_s = 0.0
    start_lateness_max_s = 0.0
    deadline_miss_count = 0

    for _ in range(sample_count):
        remaining_s = next_release_s - time.perf_counter()
        if remaining_s > 0:
            time.sleep(remaining_s)

        update_start_s = time.perf_counter()
        start_lateness_s = max(0.0, update_start_s - next_release_s)

        filter_.update(
            timestamp_s=update_start_s - wall_start_s,
            gyroscope_dps=(0.0, 0.0, 0.0),
            accelerometer_g=(0.0, 0.0, 1.0),
            magnetometer=(1.0, 0.0, 0.0),
        )

        update_end_s = time.perf_counter()
        compute_s = update_end_s - update_start_s
        compute_total_s += compute_s
        compute_max_s = max(compute_max_s, compute_s)
        start_lateness_max_s = max(start_lateness_max_s, start_lateness_s)

        if update_end_s > next_release_s + period_s:
            deadline_miss_count += 1

        next_release_s += period_s

    wall_elapsed_s = time.perf_counter() - wall_start_s
    cpu_elapsed_s = time.process_time() - cpu_start_s

    print(f"target_sample_rate_hz: {args.sample_rate:.3f}")
    print(f"requested_duration_s: {args.duration:.3f}")
    print(f"samples: {sample_count}")
    print(f"wall_elapsed_s: {wall_elapsed_s:.6f}")
    print(f"achieved_updates_per_second: {sample_count / wall_elapsed_s:.3f}")
    print(f"mean_compute_microseconds: {compute_total_s * 1e6 / sample_count:.3f}")
    print(f"maximum_compute_microseconds: {compute_max_s * 1e6:.3f}")
    print(f"maximum_start_lateness_microseconds: {start_lateness_max_s * 1e6:.3f}")
    print(f"deadline_miss_count: {deadline_miss_count}")
    print(f"process_cpu_time_s: {cpu_elapsed_s:.6f}")
    print(f"one_core_cpu_percent: {100.0 * cpu_elapsed_s / wall_elapsed_s:.3f}")


if __name__ == "__main__":
    main()
