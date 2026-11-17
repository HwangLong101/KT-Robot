"""Ideal no-slip differential-drive kinematics, SI units.

Equations: Dellaert & Hutchinson, Introduction to Robotics and Perception,
section 5.2, https://www.roboticsbook.org/S52_diffdrive_actions.html
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Robot:
    radius: float = 0.05        # wheel radius [m]
    track: float = 0.30         # wheel separation [m]
    wheel_limit: float = 12.0   # absolute wheel angular speed [rad/s]
    yaw_rate_limit: float = 2.0 # body angular speed [rad/s]

    def __post_init__(self):
        if any(not math.isfinite(x) or x <= 0 for x in
               (self.radius, self.track, self.wheel_limit, self.yaw_rate_limit)):
            raise ValueError('Robot dimensions and speed limits must be positive and finite')

    def inverse(self, v, omega):
        return ((v - self.track * omega / 2) / self.radius,
                (v + self.track * omega / 2) / self.radius)

    def forward(self, left, right):
        return (self.radius * (left + right) / 2,
                self.radius * (right - left) / self.track)

    def feasible_command(self, v, omega):
        # Scale both body commands together, preserving requested curvature.
        factor = max(1.0, abs(omega) / self.yaw_rate_limit)
        left, right = self.inverse(v / factor, omega / factor)
        factor_wheel = max(1.0, abs(left) / self.wheel_limit, abs(right) / self.wheel_limit)
        left, right = left / factor_wheel, right / factor_wheel
        v_actual, omega_actual = self.forward(left, right)
        return v_actual, omega_actual, left, right


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def integrate(pose, v, omega, dt):
    """Exact constant-twist integration over one zero-order-held command."""
    x, y, theta = pose
    if abs(omega) < 1e-10:
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
    else:
        x += v / omega * (math.sin(theta + omega * dt) - math.sin(theta))
        y -= v / omega * (math.cos(theta + omega * dt) - math.cos(theta))
    return x, y, wrap(theta + omega * dt)
