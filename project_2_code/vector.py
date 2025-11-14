import math

class Vector:

    def __init__(self, x=float(0), y=float(0), z=float(0)):
        self.x = x
        self.y = y

    def __add__(self, v):
        return Vector(
            self.x + v.x,
            self.y + v.y
        )

    def __neg__(self):
        return Vector(
            self.x * -1,
            self.y * -1
        )

    def __sub__(self, v):
        return self + -v

    def __mul__(self, s: float):
        return Vector(
            self.x * s,
            self.y * s,
        )

    def __rmul__(self, s: float):
        return self * s

    def __str__(self):
        return "[" + str(self.x) + ", " + str(self.y) + "]"

    @property
    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def to_list(self) -> list:
        return [self.x, self.y]

    def dot(self, v) -> float:
        return self.x*v.x + self.y*v.y

    def unit(self):
        norm = self.norm
        return Vector(
            self.x / norm,
            self.y / norm,
        )

    def angle(self, v) -> float:
        return math.acos(self.dot(v)/(self.norm * v.norm))

    def project(self, v):
        return self.dot(v.unit()) * v.unit()

    def flip_x(self):
        return Vector(
            x=-self.x,
            y=self.y
        )