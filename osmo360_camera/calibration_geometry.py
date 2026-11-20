"""Pure KB5 + Brown tangential geometry; no image, USB or frame interpretation.

The caller must establish the native model, pixel scale and lens binding. This
class does not interpret model8, xi/s, extrinsics or field of view. It accepts
exactly five forward radial coefficients, never truncates a larger model, and
returns only the invertible branch connected to the axis.

Equations: AdrianEddy/telemetry-parser, src/dji/rig.rs (lens/model_coefficients).
"""

from dataclasses import dataclass, field
import math
from numbers import Real


class CalibrationGeometryError(ValueError):
    """Malformed calibration, singular projection or unavailable inverse."""


def _number(value):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise CalibrationGeometryError('Finite real geometry values required')
    return float(value)


def _vector(values, size):
    try:
        result = tuple(_number(v) for v in values)
    except TypeError as error:
        raise CalibrationGeometryError('Geometry input must be a coordinate sequence') from error
    if len(result) != size:
        raise CalibrationGeometryError(f'Expected {size} coordinates')
    return result


def _poly(coefficients, x):
    value = 0.0
    for coefficient in reversed(coefficients):
        value = value*x + coefficient
    return _number(value)


def _roots(coefficients, low, high):
    """Isolate real roots via derivative intervals, including repeated roots."""
    c = list(coefficients)
    while len(c) > 1 and c[-1] == 0:
        c.pop()
    if len(c) == 1:
        return []
    critical = _roots([i*c[i] for i in range(1, len(c))], low, high)
    points, roots = [low, *critical, high], []
    for x in points:
        scale = sum(abs(v)*x**i for i, v in enumerate(c))
        if abs(_poly(c, x)) <= 1e-14*max(1.0, scale):
            roots.append(x)
    for a, b in zip(points, points[1:]):
        fa, fb = _poly(c, a), _poly(c, b)
        if (fa < 0) == (fb < 0) or fa == 0 or fb == 0:
            continue
        for _ in range(70):
            mid = (a+b)/2
            if (_poly(c, mid) < 0) == (fa < 0):
                a = mid
            else:
                b = mid
        roots.append((a+b)/2)
    return sorted(set(roots))


@dataclass(frozen=True)
class KB5Projection:
    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, ...]
    p1: float = 0.0
    p2: float = 0.0
    _theta_end: float = field(init=False, repr=False)

    def __post_init__(self):
        for name in ('fx', 'fy', 'cx', 'cy', 'p1', 'p2'):
            object.__setattr__(self, name, _number(getattr(self, name)))
        if self.fx <= 0 or self.fy <= 0:
            raise CalibrationGeometryError('Positive focal lengths required')
        if not isinstance(self.k, tuple) or len(self.k) != 5:
            raise CalibrationGeometryError('Exactly five forward KB coefficients required as a tuple')
        object.__setattr__(self, 'k', _vector(self.k, 5))
        roots = _roots([1.0, *((2*i+3)*v for i, v in enumerate(self.k))], 0.0, math.pi**2)
        object.__setattr__(self, '_theta_end', math.sqrt(roots[0]) if roots else math.pi)
        self._radius(self._theta_end)

    def _radius(self, theta):
        return _number(theta*_poly((1.0, *self.k), theta*theta))

    def _brown(self, x, y):
        r2 = x*x+y*y
        a, b, c = 1+2*self.p1*y+6*self.p2*x, 2*self.p1*x+2*self.p2*y, 1+6*self.p1*y+2*self.p2*x
        determinant = a*c-b*b
        # This symmetric affine Jacobian's positive-definite domain is convex:
        # it selects the one-to-one branch connected to the optical axis.
        if not all(math.isfinite(v) for v in (x, y, a, b, c, determinant)) or a <= 0 or determinant <= 0:
            raise CalibrationGeometryError('Tangential model is outside its invertible branch')
        return (_number(x+2*self.p1*x*y+self.p2*(r2+2*x*x)),
                _number(y+self.p1*(r2+2*y*y)+2*self.p2*x*y), a, b, c, determinant)

    def project(self, ray):
        """Lens-frame direction (+z optical, +x right, +y down) to pixel."""
        x, y, z = _vector(ray, 3)
        scale = max(abs(x), abs(y), abs(z))
        if scale == 0:
            raise CalibrationGeometryError('Zero direction has no projection')
        x, y, z = x/scale, y/scale, z/scale
        rho = math.hypot(x, y)
        if rho == 0:
            if z > 0:
                return self.cx, self.cy
            raise CalibrationGeometryError('Backward optical axis has ambiguous azimuth')
        theta = math.atan2(rho, z)
        if theta >= self._theta_end:
            raise CalibrationGeometryError('Radial model is outside its invertible branch')
        radius = self._radius(theta)
        xd, yd, *_ = self._brown(radius*x/rho, radius*y/rho)
        return _number(self.cx+self.fx*xd), _number(self.cy+self.fy*yd)

    def unproject(self, pixel):
        """Pixel to unit lens-frame ray; undo tangential then radial distortion."""
        u, v = _vector(pixel, 2)
        target_x, target_y = _number((u-self.cx)/self.fx), _number((v-self.cy)/self.fy)
        x = y = 0.0
        tolerance = 1e-13*max(1.0, math.hypot(target_x, target_y))
        for _ in range(60):
            xd, yd, a, b, c, determinant = self._brown(x, y)
            ex, ey = xd-target_x, yd-target_y
            residual = math.hypot(ex, ey)
            if residual <= tolerance:
                break
            dx, dy = (c*ex-b*ey)/determinant, (a*ey-b*ex)/determinant
            for _ in range(40):
                nx, ny = x-dx, y-dy
                try:
                    px, py, *_ = self._brown(nx, ny)
                    if math.hypot(px-target_x, py-target_y) < residual:
                        x, y = nx, ny
                        break
                except CalibrationGeometryError:
                    pass
                dx, dy = dx/2, dy/2
            else:
                raise CalibrationGeometryError('Tangential inverse did not converge on its branch')
        else:
            raise CalibrationGeometryError('Tangential inverse iteration limit')
        radius = math.hypot(x, y)
        if radius == 0:
            return 0.0, 0.0, 1.0
        if radius >= self._radius(self._theta_end):
            raise CalibrationGeometryError('Pixel is beyond the radial inverse branch')
        low, high = 0.0, self._theta_end
        for _ in range(80):
            mid = (low+high)/2
            if self._radius(mid) < radius:
                low = mid
            else:
                high = mid
        theta = (low+high)/2
        sine = math.sin(theta)
        return sine*x/radius, sine*y/radius, math.cos(theta)
