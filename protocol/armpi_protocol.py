"""Versioned, newline-delimited JSON protocol shared by the ROS 1 and ROS 2 nodes."""
import json
import math
import socket
import time
import uuid

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 16 * 1024
COMMAND_TYPES = {"base_velocity", "arm_pose", "gripper", "stop", "get_state", "heartbeat"}


def chassis_direction_degrees(vx, vy):
    """Map ROS +x forward/+y left onto vendor 90° forward/180° left."""
    return (math.degrees(math.atan2(vy, vx)) + 90.0) % 360.0 if vx or vy else 0.0


class ProtocolError(ValueError):
    pass


def make_message(message_type, payload=None, request_id=None):
    return {
        "version": PROTOCOL_VERSION,
        "id": request_id or str(uuid.uuid4()),
        "type": message_type,
        # Ubuntu 18.04's Python 3.6 lacks time.time_ns().  The Pi supplied
        # with the ArmPi Pro uses that version, while this protocol does not
        # need sub-nanosecond clock precision.
        "timestamp_ns": int(time.time() * 1000000000),
        "payload": payload or {},
    }


def _number(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ProtocolError("%s must be a number" % name)
    return float(value)


def validate_message(message):
    if not isinstance(message, dict):
        raise ProtocolError("frame must be a JSON object")
    required = {"version", "id", "type", "timestamp_ns", "payload"}
    missing = required - set(message)
    if missing:
        raise ProtocolError("missing fields: %s" % ", ".join(sorted(missing)))
    if message["version"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if not isinstance(message["id"], str) or not message["id"]:
        raise ProtocolError("id must be a non-empty string")
    if not isinstance(message["type"], str):
        raise ProtocolError("type must be a string")
    if not isinstance(message["payload"], dict):
        raise ProtocolError("payload must be an object")

    payload = message["payload"]
    kind = message["type"]
    if kind == "base_velocity":
        for key in ("vx_mps", "vy_mps", "wz_radps"):
            _number(payload.get(key), key)
    elif kind == "arm_pose":
        position = payload.get("position_m")
        if not isinstance(position, list) or len(position) != 3:
            raise ProtocolError("position_m must be a three-element array")
        for index, value in enumerate(position):
            _number(value, "position_m[%d]" % index)
        _number(payload.get("pitch_deg"), "pitch_deg")
        _number(payload.get("duration_s"), "duration_s")
    elif kind == "joint_states":
        if type(payload.get("sec")) is not int or not 0 <= payload["sec"] <= 2147483647:
            raise ProtocolError("invalid joint timestamp")
        if type(payload.get("nanosec")) is not int or not 0 <= payload["nanosec"] < 1000000000:
            raise ProtocolError("invalid joint timestamp")
        names = payload.get("name")
        if (not isinstance(payload.get("frame_id"), str) or not isinstance(names, list)
                or not names or any(not isinstance(name, str) or not name for name in names)
                or len(names) != len(set(names))):
            raise ProtocolError("invalid joint names or frame")
        for key in ("position", "velocity", "effort"):
            values = payload.get(key)
            if not isinstance(values, list) or len(values) not in (0, len(names)):
                raise ProtocolError("invalid joint " + key)
            if any(not math.isfinite(_number(value, key)) for value in values):
                raise ProtocolError("non-finite joint " + key)
    elif kind == "wheel_odometry":
        for key in ("sec", "nanosec"):
            if type(payload.get(key)) is not int or payload[key] < 0:
                raise ProtocolError("invalid odometry timestamp")
        if payload["nanosec"] >= 1000000000 or payload["sec"] > 2147483647:
            raise ProtocolError("invalid odometry timestamp")
        for key in ("frame_id", "child_frame_id"):
            if not isinstance(payload.get(key), str) or not payload[key]:
                raise ProtocolError("invalid odometry frame")
        for key, length in (("position", 3), ("orientation", 4), ("linear", 3),
                            ("angular", 3), ("pose_covariance", 36), ("twist_covariance", 36)):
            values = payload.get(key)
            if not isinstance(values, list) or len(values) != length:
                raise ProtocolError("invalid odometry " + key)
            for value in values:
                if not math.isfinite(_number(value, key)):
                    raise ProtocolError("non-finite odometry " + key)
    elif kind == "gripper":
        if payload.get("command") not in ("open", "close") and "pulse" not in payload:
            raise ProtocolError("gripper requires command open/close or pulse")
        if "pulse" in payload:
            _number(payload["pulse"], "pulse")
        if "duration_s" in payload:
            _number(payload["duration_s"], "duration_s")
    return message


def encode(message):
    validate_message(message)
    raw = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    if len(raw) > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds maximum size")
    return raw


class JsonLineSocket(object):
    """A buffered NDJSON socket. `recv_messages` returns all complete frames."""
    def __init__(self, sock):
        self.sock = sock
        self._buffer = b""

    def send(self, message):
        self.sock.sendall(encode(message))

    def recv_messages(self):
        data = self.sock.recv(4096)
        if not data:
            raise ConnectionError("peer disconnected")
        self._buffer += data
        if len(self._buffer) > MAX_FRAME_BYTES:
            raise ProtocolError("unterminated frame exceeds maximum size")
        messages = []
        while b"\n" in self._buffer:
            raw, self._buffer = self._buffer.split(b"\n", 1)
            if not raw:
                continue
            if len(raw) > MAX_FRAME_BYTES:
                raise ProtocolError("frame exceeds maximum size")
            try:
                message = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ProtocolError("invalid JSON: %s" % exc)
            messages.append(validate_message(message))
        return messages
