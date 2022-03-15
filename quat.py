from dataclasses import dataclass
import math

# a minimal implementation of quaternions, based on the airsim implementation,
# which is just enough to implement rotations.
# this is less error prone than implementing rotations with euler angles


@dataclass(unsafe_hash=True, frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    # makes using these fields more memory and runtime efficient
    __slots__ = ['x', 'y', 'z', 'w']

    @staticmethod
    def from_euler_angles(roll, pitch, yaw) -> "Quaternion":
        t0 = math.cos(yaw * 0.5)
        t1 = math.sin(yaw * 0.5)
        t2 = math.cos(roll * 0.5)
        t3 = math.sin(roll * 0.5)
        t4 = math.cos(pitch * 0.5)
        t5 = math.sin(pitch * 0.5)

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return Quaternion(x, y, z, w)

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        t, x, y, z = self.w, self.x, self.y, self.z
        a, b, c, d = other.w, other.x, other.y, other.z
        return Quaternion(w=a * t - b * x - c * y - d * z,
                          x=b * t + a * x + d * y - c * z,
                          y=c * t + a * y + b * z - d * x,
                          z=d * t + z * a + c * x - b * y)

    def conjugate(self):
        return Quaternion(-self.x, -self.y, -self.z, self.w)
