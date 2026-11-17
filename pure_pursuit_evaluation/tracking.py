"""Pure Pursuit adapted to the axle midpoint of a differential-drive robot.

Geometry follows R. C. Coulter, CMU-RI-TR-92-01 (1992).
Implementation here is newly written; upstream PythonRobotics stays in step 1.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
from model import Robot, integrate, wrap


@dataclass(frozen=True)
class Settings:
    dt: float = 0.05
    speed: float = 0.50
    lookahead: float = 0.55
    goal_tolerance: float = 0.05
    goal_gain: float = 1.0
    align_threshold: float = math.radians(75)
    heading_gain: float = 2.0
    max_time: float = 120.0
    speed_policy: str = 'baseline'
    lateral_limit: float = 0.12  # kinematic demand bound [m/s^2], not a friction model

    def __post_init__(self):
        if self.speed_policy not in ('baseline', 'curvature'):
            raise ValueError('speed_policy must be baseline or curvature')
        if any(not math.isfinite(x) or x <= 0 for k, x in asdict(self).items() if k != 'speed_policy'):
            raise ValueError('Controller parameters must be positive and finite')
        if self.align_threshold >= math.pi:
            raise ValueError('Alignment threshold must be below pi')
        if self.dt > self.max_time:
            raise ValueError('dt must not exceed max_time')


class Path:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) < 2
                or not np.isfinite(self.points).all()):
            raise ValueError('Path must contain at least two finite 2D points')
        self.delta = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if np.any(self.lengths < 1e-9):
            raise ValueError('Consecutive duplicate waypoints are not supported')
        self.s = np.r_[0, np.cumsum(self.lengths)]

    @property
    def total(self):
        return float(self.s[-1])

    def at(self, s):
        s = float(np.clip(s, 0, self.total))
        i = min(int(np.searchsorted(self.s, s, side='right') - 1), len(self.lengths)-1)
        return self.points[i] + (s-self.s[i]) / self.lengths[i] * self.delta[i]

    def project(self, xy, start=0.0, end=None):
        # Projection constrained to a forward arc-length interval.
        end = self.total if end is None else min(end, self.total)
        ids = np.flatnonzero((self.s[1:] >= start) & (self.s[:-1] <= end))
        d = self.delta[ids]
        u = np.sum((xy-self.points[ids])*d, axis=1) / self.lengths[ids]**2
        low = np.maximum(0, (start-self.s[ids]) / self.lengths[ids])
        high = np.minimum(1, (end-self.s[ids]) / self.lengths[ids])
        u = np.clip(u, low, high)
        q = self.points[ids] + u[:, None] * d
        distance = np.linalg.norm(q-xy, axis=1)
        j = int(np.argmin(distance))
        return float(self.s[ids[j]] + u[j]*self.lengths[ids[j]]), float(distance[j])

    def target(self, xy, progress, lookahead):
        # First forward intersection between the path and a look-ahead circle.
        first = min(int(np.searchsorted(self.s, progress, side='right')-1), len(self.lengths)-1)
        for i in range(max(first, 0), len(self.lengths)):
            d = self.delta[i]
            q = self.points[i] - xy
            a = float(d @ d)
            b = float(2 * q @ d)
            c = float(q @ q - lookahead**2)
            disc = b*b - 4*a*c
            if disc < 0:
                continue
            for u in sorted(((-b-math.sqrt(disc))/(2*a), (-b+math.sqrt(disc))/(2*a))):
                s = self.s[i] + u*self.lengths[i]
                if 0 <= u <= 1 and s > progress + 1e-8:
                    return self.points[i] + u*d
        # Off-path or near endpoint: use an arc-length target as fallback.
        return self.at(min(progress + lookahead, self.total))


COLUMNS = ['time_s', 'x_m', 'y_m', 'theta_rad', 'v_m_s', 'omega_rad_s',
           'left_rad_s', 'right_rad_s', 'path_error_m', 'goal_distance_m',
           'progress_m', 'target_x_m', 'target_y_m', 'mode_code']


def simulate(points, initial_pose, robot=None, settings=None):
    robot = Robot() if robot is None else robot
    settings = Settings() if settings is None else settings
    path = Path(points)
    pose = tuple(float(x) for x in initial_pose)
    if len(pose) != 3 or not np.isfinite(pose).all():
        raise ValueError('Initial pose must be three finite values')
    progress = 0.0
    rows = []
    reached = False
    # Log commands at time t; apply them over [t, t+dt). Final row is zero command.
    steps = int(math.floor(settings.max_time/settings.dt))
    for step in range(steps + 1):
        time = step * settings.dt
        xy = np.array(pose[:2])
        progress, _ = path.project(xy, progress, progress + max(2*settings.lookahead, 1.0))
        _, error = path.project(xy)
        goal_distance = float(np.linalg.norm(xy-path.points[-1]))
        remaining = path.total-progress
        # Require final-path progress too, preventing false success near an early pass at the goal.
        reached = goal_distance <= settings.goal_tolerance and remaining <= settings.lookahead
        target = path.points[-1] if remaining <= settings.lookahead else path.target(xy, progress, settings.lookahead)
        dx, dy = target-xy
        alpha = wrap(math.atan2(dy, dx) - pose[2])
        mode = 0  # 0=Pure Pursuit, 1=heading alignment, 2=goal reached, 3=timeout
        if reached or step == steps:
            v_cmd = omega_cmd = 0.0
            mode = 2 if reached else 3
        elif abs(alpha) > settings.align_threshold:
            # Auxiliary recovery for a target behind the robot (not Pure Pursuit itself).
            v_cmd = 0.0
            omega_cmd = settings.heading_gain * alpha
            mode = 1
        else:
            v_cmd = settings.speed
            if remaining <= settings.lookahead:
                v_cmd = min(v_cmd, settings.goal_gain * goal_distance)
            y_body = -math.sin(pose[2])*dx + math.cos(pose[2])*dy
            curvature = 2*y_body / max(dx*dx+dy*dy, 1e-12)
            if settings.speed_policy == 'curvature':
                # v^2*|kappa| <= lateral_limit, before common actuator scaling.
                # This is a simple educational speed law, not the full Nav2 RPP.
                v_cmd = min(v_cmd, math.sqrt(settings.lateral_limit / max(abs(curvature), 1e-12)))
            omega_cmd = v_cmd * curvature
        v, omega, left, right = robot.feasible_command(v_cmd, omega_cmd)
        rows.append([time, *pose, v, omega, left, right, error, goal_distance,
                     progress, *target, mode])
        if reached or step == steps:
            break
        pose = integrate(pose, v, omega, settings.dt)
    data = np.asarray(rows)
    ds = np.abs(data[:-1, 4])*settings.dt
    squared_error_segment = (data[:-1, 8]**2 + data[1:, 8]**2)/2
    distance_rmse = float(np.sqrt(np.sum(ds*squared_error_segment)/np.sum(ds))) if np.sum(ds) > 0 else None
    metrics = {
        'reached_goal': bool(reached),
        'termination': 'goal_tolerance' if reached else 'timeout',
        'time_s': float(data[-1, 0]),
        'sample_count': len(data),
        'final_goal_error_m': float(data[-1, 9]),
        'path_rmse_m': float(np.sqrt(np.mean(data[:, 8]**2))),
        'path_max_error_m': float(data[:, 8].max()),
        'distance_weighted_rmse_m': distance_rmse,
        'max_lateral_demand_m_s2': float(np.max(np.abs(data[:, 4]*data[:, 5]))),
        'max_lateral_demand_after_1m_m_s2': float(np.max(np.abs(data[data[:, 10] >= 1.0, 4]*data[data[:, 10] >= 1.0, 5]))) if np.any(data[:, 10] >= 1.0) else None,
        'max_abs_wheel_speed_rad_s': float(np.max(np.abs(data[:, 6:8]))),
        'max_abs_yaw_rate_rad_s': float(np.max(np.abs(data[:, 5]))),
        'distance_travelled_m': float(np.sum(np.abs(data[:-1, 4]))*settings.dt),
        'alignment_steps': int(np.sum(data[:, 13] == 1)),
        'path_progress_fraction': float(data[-1, 10] / path.total),
        'settings': asdict(settings), 'robot': asdict(robot),
        'error_definition': 'Minimum Euclidean distance from axle midpoint to reference polyline, including initial offset',
        'assumptions': ['Ideal no-slip motion', 'Perfect pose', 'Instantaneous wheel-speed response',
                        'No obstacles', 'No final-heading constraint']}
    return data, metrics
