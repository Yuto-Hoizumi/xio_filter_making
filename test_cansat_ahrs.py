from __future__ import annotations

import math
import unittest

from cansat_ahrs import AhrsConfig, CanSatAhrs


class CanSatAhrsTests(unittest.TestCase):
    def test_stationary_sensor_converges_near_level(self) -> None:
        filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=100.0))
        output = None
        for index in range(500):
            output = filter_.update(
                index / 100.0,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                (1.0, 0.0, 0.0),
            )

        assert output is not None
        self.assertAlmostEqual(output.roll_deg, 0.0, delta=0.1)
        self.assertAlmostEqual(output.pitch_deg, 0.0, delta=0.1)
        self.assertAlmostEqual(output.yaw_deg, 0.0, delta=0.1)
        self.assertFalse(output.startup)

    def test_six_axis_fallback_when_magnetometer_is_missing(self) -> None:
        filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=100.0))
        output = filter_.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        self.assertEqual(output.mode, "IMU_6DOF")

    def test_long_gap_restarts_filter(self) -> None:
        filter_ = CanSatAhrs(
            AhrsConfig(sample_rate_hz=100.0, maximum_sample_gap_s=0.25)
        )
        filter_.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        output = filter_.update(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        self.assertTrue(output.filter_restarted)
        self.assertTrue(output.startup)

    def test_invalid_accelerometer_holds_last_output(self) -> None:
        filter_ = CanSatAhrs(AhrsConfig(sample_rate_hz=100.0))
        before = filter_.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        after = filter_.update(0.01, (0.0, 0.0, 0.0), (math.nan, 0.0, 1.0))
        self.assertFalse(after.sample_valid)
        self.assertEqual(after.mode, "INVALID_HOLD")
        self.assertEqual(after.quaternion_wxyz, before.quaternion_wxyz)

    def test_impact_flag(self) -> None:
        filter_ = CanSatAhrs(AhrsConfig(impact_threshold_g=4.0))
        output = filter_.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 5.0))
        self.assertTrue(output.impact_detected)


if __name__ == "__main__":
    unittest.main()
