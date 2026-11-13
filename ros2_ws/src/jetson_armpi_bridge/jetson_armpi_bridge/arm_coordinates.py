"""Configurable logical arm_base to vendor IK conversion; no ROS dependency."""
import math


def pure_pitch(q):
    values = [float(v) for v in q]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('orientation must be finite')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-9:
        raise ValueError('orientation must have nonzero norm')
    x, y, z, w = [v / norm for v in values]
    if math.hypot(x, z) > math.sin(0.02 / 2):
        raise ValueError('arm command must be pitch-only')
    return (2 * math.atan2(y, w) + math.pi) % (2 * math.pi) - math.pi


class ArmCoordinates:
    def __init__(self, axes=(-2, 1, 3), offset=(0., 0., 0.), pitch_scale=-1., pitch_offset_deg=0.):
        self.axes = tuple(axes)
        self.offset = tuple(float(v) for v in offset)
        self.pitch_scale = float(pitch_scale)
        self.pitch_offset_deg = float(pitch_offset_deg)
        if len(self.axes) != 3 or any(type(v) is not int for v in self.axes) or sorted(abs(v) for v in self.axes) != [1, 2, 3]:
            raise ValueError('axes must be a signed permutation of 1,2,3')
        if len(self.offset) != 3 or not all(math.isfinite(v) for v in (*self.offset, self.pitch_scale, self.pitch_offset_deg)) or self.pitch_scale == 0:
            raise ValueError('offset/pitch parameters must be finite; pitch scale nonzero')

    def to_vendor(self, xyz, pitch):
        if len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, pitch)):
            raise ValueError('pose must be finite')
        position = [(1 if axis > 0 else -1) * xyz[abs(axis)-1] + off
                    for axis, off in zip(self.axes, self.offset)]
        return position, self.pitch_scale * math.degrees(pitch) + self.pitch_offset_deg

    def from_vendor(self, xyz, pitch_deg):
        if len(xyz) != 3 or not all(math.isfinite(v) for v in (*xyz, pitch_deg)):
            raise ValueError('pose must be finite')
        position = [0., 0., 0.]
        for axis, value, off in zip(self.axes, xyz, self.offset):
            position[abs(axis)-1] = (1 if axis > 0 else -1) * (value-off)
        return position, math.radians((pitch_deg-self.pitch_offset_deg) / self.pitch_scale)
