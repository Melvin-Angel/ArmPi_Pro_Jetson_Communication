import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros1_ws', 'src', 'armpi_jetson_bridge', 'scripts'))
from wheel_math import WheelOdometry, count_delta, decode_counts


class WheelMathTest(unittest.TestCase):
    def make(self):
        # One count corresponds to one metre, for easy kinematic checks.
        odom = WheelOdometry(1 / (2 * math.pi), 1, 1, [1, 1, 1, 1], 100)
        odom.update([0, 0, 0, 0], 1.0)
        return odom

    def test_cardinal_motion_and_rotation(self):
        for counts, expected in [([1, 1, 1, 1], (1, 0, 0)),
                                 ([-1, 1, 1, -1], (0, 1, 0)),
                                 ([-1, 1, -1, 1], (0, 0, 1))]:
            result = self.make().update(counts, 2.0)
            for actual, value in zip(result[:3], expected):
                self.assertAlmostEqual(actual, value)

    def test_decode_and_wrap(self):
        self.assertEqual(decode_counts(struct.pack('<4i', -1, 2, 3, 4)), (-1, 2, 3, 4))
        self.assertEqual(count_delta(-2147483648, 2147483647), 1)
        with self.assertRaises(ValueError):
            decode_counts(b'bad')

    def test_encoder_joint_angles_and_rejection(self):
        odom = WheelOdometry(0.0475, 100, 0.196, [-1, 1, 1, -1], 1000)
        odom.update([10, 20, 30, 40], 1.0)
        self.assertEqual(odom.angles, [0.0] * 4)
        odom.update([-15, 45, 55, 15], 1.5)
        for angle, speed in zip(odom.angles, odom.angular_velocities):
            self.assertAlmostEqual(angle, math.pi / 2)
            self.assertAlmostEqual(speed, math.pi)
        previous = list(odom.angles)
        with self.assertRaises(ValueError):
            odom.update([10000] * 4, 1.6)
        self.assertEqual(odom.angles, previous)

    def test_reject_discontinuity(self):
        with self.assertRaises(ValueError):
            self.make().update([1000] * 4, 1.1)


if __name__ == '__main__':
    unittest.main()
