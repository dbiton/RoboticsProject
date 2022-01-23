import math


class Position:
    def __init__(self):
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_m = 0.0

    def __str__(self):
        return f'({self.x_m}, {self.y_m}, {self.z_m})'

    def vectorTo(self, o):
        return Vector(self.x_m - o.x_m, self.y_m - o.y_m, self.z_m - o.z_m)

    def distanceTo(self, o):
        return self.vectorTo(o).length()


class Vector:
    def __init__(self, x, y, z):
        self.x_m = x
        self.y_m = y
        self.z_m = z

    def eulerAngles(self):
        d = self.normalized()
        pitch = math.asin(-d.y_m)
        yaw = math.atan2(d.x_m, d.z_m)
        return pitch, yaw

    def length(self):
        return math.sqrt(self.x_m ** 2 + self.y_m ** 2 + self.z_m ** 2)

    def normalized(self):
        l = self.length()
        return Vector(self.x_m / l, self.y_m / l, self.z_m / l)


class Orientation:
    def __init__(self):
        self.x_rad = 0.0
        self.y_rad = 0.0
        self.z_rad = 0.0

    def __str__(self):
        return f'({self.x_rad}, {self.y_rad}, {self.z_rad})'


class Pose:
    def __init__(self):
        self.pos = Position()
        self.orientation = Orientation()

    def __str__(self):
        return f'pos: {str(self.pos)}\norientation: {str(self.orientation)}'


class PointCloud:
    def __init__(self):
        self.points = []

    def __str__(self):
        return str(self.points)
