"""Measure filter-only update cost without sensor I/O or artificial sleeps."""

from __future__ import annotations

import argparse
import time

from cansat_ahrs import AhrsConfig, CanSatAhrs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=1_000_000)
    parser.add_argument("--sample-rate", type=float, default=100.0)
    args = parser.parse_args()

    filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=args.sample_rate))
    period_s = 1.0 / args.sample_rate

    start = time.perf_counter()
    for index in range(args.samples):
        filter_.update(
            index * period_s,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
        )
    elapsed_s = time.perf_counter() - start

    print(f"samples: {args.samples}")
    print(f"elapsed_s: {elapsed_s:.6f}")
    print(f"microseconds_per_update: {elapsed_s * 1e6 / args.samples:.3f}")
    print(f"maximum_filter_updates_per_second: {args.samples / elapsed_s:.1f}")


if __name__ == "__main__":
    main()
