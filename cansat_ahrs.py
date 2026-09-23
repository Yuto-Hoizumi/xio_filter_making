"""CanSat attitude filter built on x-io Fusion.

Input units:
    gyroscope: degrees/second
    accelerometer: g
    magnetometer: any calibrated, consistent unit (typically microtesla)

The sensor axes must be remapped to the vehicle body axes before calling update.
The current prototype uses the NWU convention: X forward/north, Y left/west,
Z up.  This matches a right-handed ground-vehicle frame, but the final remap
must be checked on the real PCB.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional

import imufusion
import numpy as np


Vector3 = Iterable[float]


@dataclass(frozen=True)
class AhrsConfig:
    sample_rate_hz: float = 100.0
    gain: float = 0.5
    gyroscope_range_dps: float = 2000.0
    acceleration_rejection_deg: float = 10.0
    magnetic_rejection_deg: float = 10.0
    rejection_timeout_s: float = 5.0
    maximum_sample_gap_s: float = 0.25
    impact_threshold_g: float = 4.0


@dataclass(frozen=True)
class AhrsOutput:
    timestamp_s: float
    quaternion_wxyz: tuple[float, float, float, float]
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    earth_acceleration_g: tuple[float, float, float]
    mode: str
    sample_valid: bool
    filter_restarted: bool
    impact_detected: bool
    accelerometer_ignored: bool
    magnetometer_ignored: bool
    startup: bool
    angular_rate_recovery: bool
    acceleration_recovery: bool
    magnetic_recovery: bool


def _vector3(value: Vector3, name: str) -> np.ndarray:
    vector = np.asarray(tuple(value), dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must contain exactly three values")
    return vector


def _is_usable(vector: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(vector)) and np.linalg.norm(vector) > 1e-9)


class CanSatAhrs:
    """Stateful AHRS wrapper with timing validation and fallback modes."""

    def __init__(self, config: AhrsConfig = AhrsConfig()) -> None:
        if config.sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive")
        if config.maximum_sample_gap_s <= 0:
            raise ValueError("maximum_sample_gap_s must be positive")

        self.config = config
        self._default_period_s = 1.0 / config.sample_rate_hz
        self._previous_timestamp_s: Optional[float] = None

        self._bias = imufusion.Bias()
        self._bias.set_settings(
            imufusion.BiasSettings(sample_rate=config.sample_rate_hz)
        )

        self._ahrs = imufusion.Ahrs()
        self._ahrs.set_settings(
            imufusion.AhrsSettings(
                sample_rate=config.sample_rate_hz,
                convention=imufusion.CONVENTION_NWU,
                gain=config.gain,
                gyroscope_range=config.gyroscope_range_dps,
                acceleration_rejection=config.acceleration_rejection_deg,
                magnetic_rejection=config.magnetic_rejection_deg,
                rejection_timeout=config.rejection_timeout_s,
            )
        )

    def restart(self) -> None:
        """Restart attitude convergence after a sensor reset or long data gap."""
        self._ahrs.restart()
        self._previous_timestamp_s = None

    def update(
        self,
        timestamp_s: float,
        gyroscope_dps: Vector3,
        accelerometer_g: Vector3,
        magnetometer: Optional[Vector3] = None,
        external_heading_deg: Optional[float] = None,
    ) -> AhrsOutput:
        """Consume one sample and return orientation plus health flags.

        Priority is calibrated magnetometer, then an optional external heading,
        then six-axis operation.  ``external_heading_deg`` is intended for a
        separately gated GNSS course measurement; callers must only provide it
        when GNSS speed and fix quality make the course trustworthy.
        """
        timestamp_s = float(timestamp_s)
        gyro = _vector3(gyroscope_dps, "gyroscope_dps")
        accel = _vector3(accelerometer_g, "accelerometer_g")
        mag = None if magnetometer is None else _vector3(magnetometer, "magnetometer")

        finite_time = math.isfinite(timestamp_s)
        sample_valid = finite_time and _is_usable(accel) and bool(np.all(np.isfinite(gyro)))

        restarted = False
        period_s = self._default_period_s
        if finite_time and self._previous_timestamp_s is not None:
            measured_period_s = timestamp_s - self._previous_timestamp_s
            if measured_period_s <= 0 or measured_period_s > self.config.maximum_sample_gap_s:
                self._ahrs.restart()
                restarted = True
            else:
                period_s = measured_period_s

        if finite_time:
            self._previous_timestamp_s = timestamp_s

        if sample_valid:
            self._ahrs.set_sample_period(period_s)
            corrected_gyro = self._bias.update(gyro)

            if mag is not None and _is_usable(mag):
                self._ahrs.update(corrected_gyro, accel, mag)
                mode = "MARG_9DOF"
            elif external_heading_deg is not None and math.isfinite(external_heading_deg):
                self._ahrs.update_external_heading(
                    corrected_gyro,
                    accel,
                    float(external_heading_deg),
                )
                mode = "GNSS_HEADING"
            else:
                self._ahrs.update_no_magnetometer(corrected_gyro, accel)
                mode = "IMU_6DOF"
        else:
            mode = "INVALID_HOLD"

        quaternion = np.asarray(self._ahrs.get_quaternion(), dtype=float)
        euler = np.asarray(imufusion.quaternion_to_euler(quaternion), dtype=float)
        earth_acceleration = np.asarray(self._ahrs.get_earth_acceleration(), dtype=float)
        internal = self._ahrs.get_internal_states()
        flags = self._ahrs.get_flags()

        acceleration_norm = float(np.linalg.norm(accel)) if np.all(np.isfinite(accel)) else math.nan
        impact_detected = bool(
            math.isfinite(acceleration_norm)
            and acceleration_norm >= self.config.impact_threshold_g
        )

        return AhrsOutput(
            timestamp_s=timestamp_s,
            quaternion_wxyz=tuple(float(value) for value in quaternion),
            roll_deg=float(euler[0]),
            pitch_deg=float(euler[1]),
            yaw_deg=float(euler[2]),
            earth_acceleration_g=tuple(float(value) for value in earth_acceleration),
            mode=mode,
            sample_valid=sample_valid,
            filter_restarted=restarted,
            impact_detected=impact_detected,
            accelerometer_ignored=bool(internal.accelerometer_ignored),
            magnetometer_ignored=bool(internal.magnetometer_ignored),
            startup=bool(flags.startup),
            angular_rate_recovery=bool(flags.angular_rate_recovery),
            acceleration_recovery=bool(flags.acceleration_recovery),
            magnetic_recovery=bool(flags.magnetic_recovery),
        )
