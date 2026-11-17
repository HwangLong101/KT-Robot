"""Analytic motion checks plus end-to-end goal/limit checks."""
import math
import unittest
import numpy as np
from model import Robot, integrate
from tracking import Settings, simulate
from scenarios import NAMES, scenario


class KinematicsChecks(unittest.TestCase):
    def test_straight_motion(self):
        robot = Robot()
        v, omega = robot.forward(4., 4.)
        np.testing.assert_allclose(integrate((1, 2, 0), v, omega, 2), (1.4, 2, 0), atol=1e-12)

    def test_counter_rotation(self):
        v, omega = Robot().forward(-3., 3.)
        np.testing.assert_allclose(integrate((1, 2, 0), v, omega, 1), (1, 2, 1), atol=1e-12)

    def test_quarter_circle(self):
        # v=0.5 m/s, omega=1 rad/s for pi/2 s -> radius 0.5 m.
        np.testing.assert_allclose(integrate((0, 0, 0), .5, 1., math.pi/2), (.5, .5, math.pi/2), atol=1e-12)

    def test_wheel_limits_and_curvature(self):
        robot = Robot()
        v, w, left, right = robot.feasible_command(1., 4.)
        self.assertLessEqual(max(abs(left), abs(right)), robot.wheel_limit+1e-12)
        self.assertLessEqual(abs(w), robot.yaw_rate_limit+1e-12)
        self.assertAlmostEqual(w/v, 4.)
        np.testing.assert_allclose(robot.forward(left, right), (v, w))


class SimulationChecks(unittest.TestCase):
    def assert_physics_and_goal(self, data, metrics):
        robot = Robot()
        self.assertTrue(metrics['reached_goal'])
        self.assertLessEqual(metrics['final_goal_error_m'], Settings().goal_tolerance)
        self.assertTrue(np.isfinite(data).all())
        self.assertLessEqual(np.max(np.abs(data[:, 6:8])), robot.wheel_limit+1e-10)
        self.assertLessEqual(np.max(np.abs(data[:, 5])), robot.yaw_rate_limit+1e-10)
        np.testing.assert_allclose(data[:, 4], robot.radius*(data[:, 6]+data[:, 7])/2, atol=1e-12)
        np.testing.assert_allclose(data[:, 5], robot.radius*(data[:, 7]-data[:, 6])/robot.track, atol=1e-12)
        np.testing.assert_allclose(data[-1, 4:8], 0)

    def test_three_paths(self):
        for name in NAMES:
            with self.subTest(name=name):
                self.assert_physics_and_goal(*simulate(*scenario(name)))

    def test_recovery_when_heading_away(self):
        path, pose = scenario('straight')
        data, metrics = simulate(path, (pose[0], pose[1], math.pi))
        self.assert_physics_and_goal(data, metrics)
        self.assertGreater(metrics['alignment_steps'], 0)

    def test_closed_path_not_immediate_false_success(self):
        points = [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]
        data, metrics = simulate(points, (0, 0, 0))
        self.assert_physics_and_goal(data, metrics)
        self.assertGreater(metrics['time_s'], 10.)
        self.assertGreater(metrics['path_progress_fraction'], .95)

    def test_timeout_is_not_success(self):
        data, metrics = simulate(*scenario('straight'), settings=Settings(max_time=.1))
        self.assertFalse(metrics['reached_goal'])
        self.assertEqual(metrics['termination'], 'timeout')
        np.testing.assert_allclose(data[-1, 4:8], 0)

    def test_invalid_parameters(self):
        for value in (0., -1., float('nan')):
            with self.assertRaises(ValueError):
                Settings(lookahead=value)

    def test_step2_baseline_regression(self):
        # Values independently recorded from the delivered step-2 implementation.
        expected = {'straight': (17.65, .1467936974794322),
                    's_curve': (24.20, .1589330481752767),
                    'corners': (28.70, .12055942377470927)}
        for name, (time, rmse) in expected.items():
            _, metrics = simulate(*scenario(name))
            self.assertAlmostEqual(metrics['time_s'], time, places=9)
            self.assertAlmostEqual(metrics['path_rmse_m'], rmse, places=11)

    def test_curvature_policy_cap_and_goal(self):
        for name in NAMES:
            with self.subTest(name=name):
                data, metrics = simulate(*scenario(name), settings=Settings(speed_policy='curvature'))
                self.assert_physics_and_goal(data, metrics)
                # Recover body velocities from applied wheel commands, independently of metrics.
                v = .05*(data[:,6]+data[:,7])/2
                omega = .05*(data[:,7]-data[:,6])/.30
                self.assertLessEqual(np.max(np.abs(v*omega)), .12+1e-10)

    def test_invalid_policy(self):
        with self.assertRaises(ValueError):
            Settings(speed_policy='unknown')


if __name__ == '__main__':
    unittest.main(verbosity=2)
