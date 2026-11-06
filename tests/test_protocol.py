import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol"))
from armpi_protocol import JsonLineSocket, ProtocolError, encode, make_message, validate_message


class ProtocolTest(unittest.TestCase):
    def test_round_trip_with_fragmentation(self):
        left, right = socket.socketpair()
        try:
            message = make_message("base_velocity", {"vx_mps": 0.1, "vy_mps": 0, "wz_radps": -0.2}, "one")
            raw = encode(message)
            left.sendall(raw[:9])
            left.sendall(raw[9:])
            self.assertEqual(JsonLineSocket(right).recv_messages(), [message])
        finally:
            left.close()
            right.close()

    def test_rejects_bad_arm_pose(self):
        message = make_message("arm_pose", {"position_m": [0, 1], "pitch_deg": 0, "duration_s": 1})
        with self.assertRaises(ProtocolError):
            validate_message(message)

    def test_wheel_odometry_round_trip_and_validation(self):
        payload = {"sec": 42, "nanosec": 123, "frame_id": "wheel_odom",
                   "child_frame_id": "base_link", "position": [1, 2, 0],
                   "orientation": [0, 0, 0, 1], "linear": [0.1, 0, 0],
                   "angular": [0, 0, 0.2], "pose_covariance": [0.0] * 36,
                   "twist_covariance": [0.0] * 36}
        left, right = socket.socketpair()
        try:
            message = make_message("wheel_odometry", payload)
            left.sendall(encode(message))
            self.assertEqual(JsonLineSocket(right).recv_messages(), [message])
        finally:
            left.close()
            right.close()
        payload["pose_covariance"] = [0.0]
        with self.assertRaises(ProtocolError):
            validate_message(make_message("wheel_odometry", payload))

    def test_gripper_requires_operation(self):
        with self.assertRaises(ProtocolError):
            validate_message(make_message("gripper", {}))

    def test_joint_state_transport_and_invalid_values(self):
        payload = {"sec": 42, "nanosec": 12, "frame_id": "base_link",
                   "name": ["joint1", "joint2"], "position": [0.2, -0.3],
                   "velocity": [0.0, 0.0], "effort": []}
        left, right = socket.socketpair()
        try:
            message = make_message("joint_states", payload)
            left.sendall(encode(message))
            self.assertEqual(JsonLineSocket(right).recv_messages(), [message])
        finally:
            left.close()
            right.close()
        payload["position"] = [float('nan'), 0.0]
        with self.assertRaises(ProtocolError):
            validate_message(make_message("joint_states", payload))
        payload["position"] = [0.0]
        with self.assertRaises(ProtocolError):
            validate_message(make_message("joint_states", payload))


if __name__ == "__main__":
    unittest.main()
