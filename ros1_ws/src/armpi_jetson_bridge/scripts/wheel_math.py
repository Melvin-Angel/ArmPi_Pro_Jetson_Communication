"""Wheel count decoding and planar mecanum odometry; no ROS dependency."""
import math
import struct


def decode_counts(data):
    if len(data) != 16:
        raise ValueError('expected 16 encoder bytes')
    return struct.unpack('<4i', bytes(bytearray(data)))


def count_delta(current, previous):
    return (current - previous + 2**31) % 2**32 - 2**31


class WheelOdometry:
    def __init__(self, radius, ticks_per_revolution, wheelbase_sum, signs, max_ticks_per_second):
        if any(not math.isfinite(v) or v <= 0 for v in (radius, ticks_per_revolution, wheelbase_sum, max_ticks_per_second)):
            raise ValueError('calibration values must be positive')
        if len(signs) != 4 or any(s not in (-1, 1) for s in signs):
            raise ValueError('four wheel signs must be -1 or 1')
        self.scale = 2 * math.pi * radius / ticks_per_revolution
        self.lever = wheelbase_sum
        self.signs = signs
        self.limit = max_ticks_per_second
        self.previous = None
        self.stamp = None
        self.x = self.y = self.yaw = 0.0

    def update(self, counts, stamp):
        if len(counts) != 4 or not math.isfinite(stamp):
            raise ValueError('expected four counts and a finite timestamp')
        if self.previous is None:
            self.previous, self.stamp = counts, stamp
            return None
        dt = stamp - self.stamp
        delta = [count_delta(a, b) for a, b in zip(counts, self.previous)]
        self.previous, self.stamp = counts, stamp
        if dt <= 0 or dt > 1.0 or any(abs(d) > self.limit * dt for d in delta):
            raise ValueError('encoder discontinuity or invalid sample interval; re-baselined')
        # Inputs are canonical front-left, front-right, rear-left, rear-right.
        fl, fr, rl, rr = [d * s * self.scale for d, s in zip(delta, self.signs)]
        dx = (fl + fr + rl + rr) / 4.0
        dy = (-fl + fr + rl - rr) / 4.0
        dyaw = (-fl + fr - rl + rr) / (4.0 * self.lever)
        mid = self.yaw + dyaw / 2.0
        self.x += math.cos(mid) * dx - math.sin(mid) * dy
        self.y += math.sin(mid) * dx + math.cos(mid) * dy
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))
        return self.x, self.y, self.yaw, dx / dt, dy / dt, dyaw / dt
