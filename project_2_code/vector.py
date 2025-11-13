class Vector:

    def __init__(self, x=float(0), y=float(0), z=float(0)):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, v: Vector) -> Vector:
        return Vector(
            self.x + v.x,
            self.y + v.y,
            self.z + v.z
        )

    def __neg__(self):
        return Vector(
            self.x * -1,
            self.y * -1,
            self.z * -1
        )

    def __sub__(self, v: Vector) -> Vector:
        return self + -v

    def __mul__(self, s: float) -> Vector:
        return Vector(
            self.x * s,
            self.y * s,
            self.z * s
        )

    def __rmul__(self, other) -> Vector:
        return self * other

    def __str__(self):
        return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + "]"

    @property
    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def to_list(self) -> list:
        return [self.x, self.y, self.z]

    def dot(self, v: Vector) -> float:
        return self.x*v.x + self.y*v.y + self.z*v.z

    def cross(self, v: Vector) -> Vector:
        return Vector(
            self.y*v.z - self.z*v.y,
            self.z*v.x - self.x*v.z,
            self.x*v.y - self.y*v.x
        )

    def unit(self):
        norm = self.norm
        return Vector(
            self.x / norm,
            self.y / norm,
            self.z / norm
        )

    def angle(self, v: Vector) -> float:
        return math.acos(self.dot(v)/(self.norm * v.norm))

    def project(self, v: Vector) -> Vector:
        return self.dot(v.unit()) * v.unit()

    def flip_x(self):
        return Vector(
            x=-self.x,
            y=self.y
        )