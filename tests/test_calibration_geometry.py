"""Native-coefficient golden values and independent inverse/branch regressions."""

from dataclasses import replace
import math
import unittest

from osmo360_camera.calibration_geometry import KB5Projection, CalibrationGeometryError


# First lens from the captured 2026-11-20 TLV13: all five radial terms matter.
NATIVE = dict(fx=1049.9498291015625, fy=1049.541748046875,
              cx=1917.8426513671875, cy=1921.01611328125,
              k=(0.06804720312356949, -0.013568899594247341,
                 0.011446000076830387, -0.007481350097805262, 0.0011231499956920743),
              p1=-0.0004904349916614592, p2=0.001168790040537715)


def direction(angle, azimuth=0):
    theta, phi = math.radians(angle), math.radians(azimuth)
    return math.sin(theta)*math.cos(phi), math.sin(theta)*math.sin(phi), math.cos(theta)


class CalibrationGeometryTests(unittest.TestCase):
    def assert_vector(self, a, b, places=10):
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=places)

    def test_optical_axis_and_direction_scale(self):
        model = KB5Projection(**NATIVE)
        self.assertEqual(model.project((0, 0, 5)), (model.cx, model.cy))
        self.assertEqual(model.unproject((model.cx, model.cy)), (0., 0., 1.))
        ray = direction(95, 32)
        self.assert_vector(model.project(ray), model.project(tuple(v*1e300 for v in ray)))

    def test_roundtrip_front_and_rear_hemispheres_with_tangential_distortion(self):
        model = KB5Projection(**NATIVE)
        for angle in (0, 30, 80, 95):
            for azimuth in (0, 30, 90, 180, 270):
                with self.subTest(angle=angle, azimuth=azimuth):
                    ray = direction(angle, azimuth)
                    self.assert_vector(model.unproject(model.project(ray)), ray)

    def test_nonzero_tangential_pixel_corner_roundtrips(self):
        model = KB5Projection(**NATIVE)
        # Mathematical image coordinates, not a claim that the lens sees corners.
        for pixel in ((0, 0), (3840, 0), (0, 3840), (3840, 3840)):
            with self.subTest(pixel=pixel):
                ray = model.unproject(pixel)
                self.assertAlmostEqual(math.hypot(*ray), 1., places=12)
                self.assert_vector(model.project(ray), pixel, places=7)

    def test_native_coefficients_match_expanded_polynomial_goldens(self):
        model = KB5Projection(**NATIVE)
        # Computed separately with the expanded theta^3..theta^11 polynomial
        # and direct Brown equations, not through the implementation helpers.
        goldens = [
            (30, 30, 2403.176261833497, 2200.767350477851),
            (80, 45, 3051.6889115906624, 3050.3920879014645),
            (95, 32, 3507.5469650937052, 2909.9443981356394),
            (95, 225, 605.1146435934818, 603.3030703750101),
        ]
        for angle, azimuth, u, v in goldens:
            with self.subTest(angle=angle, azimuth=azimuth):
                self.assert_vector(model.project(direction(angle, azimuth)), (u, v), places=8)

    def test_fifth_radial_term_cannot_be_discarded_or_relabelled(self):
        native = replace(KB5Projection(**NATIVE), p1=0., p2=0.)
        truncated = replace(native, k=(*native.k[:4], 0.))
        difference = native.project(direction(80))[0] - truncated.project(direction(80))[0]
        self.assertAlmostEqual(difference, 46.37107962551029, places=9)
        with self.assertRaises(CalibrationGeometryError):
            truncated.project(direction(95))
        for count in (0, 4, 8, 9):
            with self.subTest(count=count), self.assertRaises(CalibrationGeometryError):
                replace(native, k=tuple(0. for _ in range(count)))

    def test_radial_first_fold_is_not_inverted_through_another_branch(self):
        model = KB5Projection(100, 100, 0, 0, (-1/3, 0., 0., 0., 0.))
        self.assert_vector(model.unproject(model.project(direction(30))), direction(30))
        with self.assertRaises(CalibrationGeometryError):
            model.project(direction(70))
        with self.assertRaises(CalibrationGeometryError):
            model.unproject((100, 0))

    def test_repeated_radial_derivative_root_is_rejected_too(self):
        # derivative=(1-theta^2)^2 only touches zero; endpoint-sign checks miss it.
        model = KB5Projection(100, 100, 0, 0, (-2/3, 1/5, 0., 0., 0.))
        with self.assertRaises(CalibrationGeometryError):
            model.project(direction(70))

    def test_tangential_fold_or_unreachable_pixel_does_not_return_a_ray(self):
        model = KB5Projection(1, 1, 0, 0, (0.,)*5, p2=1)
        with self.assertRaises(CalibrationGeometryError):
            model.project(direction(math.degrees(.2), 180))
        with self.assertRaises(CalibrationGeometryError):
            model.unproject((-.1, 0))

    def test_malformed_calibration_and_coordinates_fail_explicitly(self):
        model = KB5Projection(**NATIVE)
        for changes in (dict(fx=0), dict(fy=-1), dict(cx=math.nan), dict(p1=math.inf),
                        dict(p2=True), dict(k=[0.]*5), dict(k=(math.nan, 0., 0., 0., 0.))):
            with self.subTest(changes=changes), self.assertRaises(CalibrationGeometryError):
                replace(model, **changes)
        for ray in ((0, 0, 0), (0, 0, -1), (math.nan, 0, 1), (1, 2), (True, 0, 1)):
            with self.subTest(ray=ray), self.assertRaises(CalibrationGeometryError):
                model.project(ray)
        for pixel in ((math.inf, 0), (1,), None):
            with self.subTest(pixel=pixel), self.assertRaises(CalibrationGeometryError):
                model.unproject(pixel)


if __name__ == '__main__':
    unittest.main()
