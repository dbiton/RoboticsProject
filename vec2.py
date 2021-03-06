from dataclasses import dataclass
import math
from typing import Optional


@dataclass(unsafe_hash=True, frozen=True)
class Vec2:
    x: float
    y: float

    # makes using these fields more memory and runtime efficient
    __slots__ = ['x', 'y']

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float) -> "Vec2":
        return Vec2(self.x * other, self.y * other)

    def __rmul__(self, other: float) -> "Vec2":
        return Vec2(self.x * other, self.y * other)

    def __truediv__(self, other: float) -> "Vec2":
        return Vec2(self.x / other, self.y / other)

    def __neg__(self) -> "Vec2":
        return Vec2(-self.x, -self.y)

    def round(self) -> "Vec2":
        """
        return the same vector, with its coordinates rounded to the nearest integer
        """
        return Vec2(round(self.x), round(self.y))

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.sqrt(self.dot(self))

    def distance(self, other: "Vec2") -> float:
        return (other - self).length()

    def signed_area(self, other: "Vec2") -> float:
        """
        find the signed area of the parallelogram formed by the vectors.
        in 3D this would be the length of the cross product.
        """
        return self.x * other.y - self.y * other.x

    def angle(self, other: "Vec2") -> float:
        """
        find the angle from this vector to the other, in radians
        """
        return math.atan2(self.signed_area(other), self.dot(other))

    def rotate(self, angle: float) -> "Vec2":
        """
        rotate the vector by the angle, given in radians
        """
        cos = math.cos(angle)
        sin = math.sin(angle)
        return Vec2(self.x * cos - self.y * sin,
                    self.x * sin + self.y * cos)

    def project(self, other: "Vec2") -> "Vec2":
        """
        returns the projection of the other vector onto this one
        """
        # multiplication by the othogonal projection matrix (v*v^T)/(v^T*v)
        x = self.x * self.x * other.x + self.x * self.y * other.y
        y = self.x * self.y * other.x + self.y * self.y * other.y
        squared_length = self.dot(self)
        if squared_length < 0.0001:
            # this vector is so small the projection is onto a point,
            # which is just the point itself
            return self
        return Vec2(x, y) / self.dot(self)

    def normalize(self) -> "Vec2":
        length = self.length()
        if length < 0.0001:
            # avoid dividing by zero
            return Vec2(0, 0)
        return self / length


def getFoVCoverage(center: Vec2, radius: float) -> Optional[float]:
    """
    returns the angle of view ocluded by the half circle,
    with a given center and radius, from the center to its edge, relative to the origin
    """
    origin = Vec2(0, 0)
    dist = origin.distance(center)
    if dist <= radius:
        # the point is inside the circle,
        # so the tangent is undefined
        return None
    # the tangent, radius and line from origin to center form a right triangle
    return math.atan2(radius, math.sqrt(dist**2 - radius**2))


def checkoverlapCircle(a: Vec2, b: Vec2, o: Vec2, radius: float) -> bool:
    """
    checks if the segment (a,b) overlaps with the circle around o
    """
    if o.distance(a) < radius or o.distance(b) < radius:
        return True

    # find the projection of o onto the line passing through a and b
    line = b - a
    p = line.project(o - a) + a
    if o.distance(p) > radius:
        # if the projection is outside the circle,
        # all other points would be furthur away from the center,
        # and therefore not inside the circle
        return False
    # since a and b are outside of the circle,
    # the segment overlaps iff the segment (a,b) intersects with the ray from o to p
    #
    # if the points are on oposite sides, the angle between them would be pi
    # and 0 if they are on the same side
    return abs((b - p).angle(a - p)) > math.pi / 2
