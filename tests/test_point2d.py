#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  7 17:04:52 2019

@author: lothar
"""
import unittest

from hypothesis import given, assume
from hypothesis.strategies import integers, floats
finite_floats = lambda n : n*(floats(allow_nan=False, allow_infinity=False),)
finite_ints = lambda n : n*(integers(),)

from math import isnan, isinf

from context import aiboids
from aiboids.point2d import Point2d

class TestPoint2dInits(unittest.TestCase):

    @given(*finite_floats(2))
    def test_xy_coords(self, x, y):
        vtest = Point2d(x, y)
        self.assertEqual(vtest.x, x)
        self.assertEqual(vtest.y, y)

    @given(*finite_floats(2))
    def test_tuple_vs_coords(self, x, y):
        vtest = Point2d(x, y)
        self.assertEqual(vtest.ntuple, (vtest.x, vtest.y))

    def test_for_zero_vector(self):
        from aiboids.point2d import ZERO_VECTOR
        self.assertEqual(ZERO_VECTOR, Point2d(0, 0))
        self.assertIsNot(ZERO_VECTOR, Point2d(0, 0))

    @given(*finite_floats(2))
    def test_zero_inplace(self, x, y):
        vtest = Point2d(x, y)
        vtest.zero()
        self.assertEqual(vtest, Point2d(0, 0))

    @given(*finite_ints(2))
    def test_promote_integer_coords(self, x, y):
        vtest = Point2d(x, y)
        self.assertEqual(vtest.x, float(x))
        self.assertEqual(vtest.y, float(y))


class TestPoint2dVectorNorms(unittest.TestCase):

    def test_zero_vector_has_no_unit(self):
        from aiboids.point2d import ZERO_VECTOR
        with self.assertRaises(ZeroDivisionError):
            ZERO_VECTOR.unit()

    @given(*finite_floats(2))
    def test_vector_norms(self, x, y):
        if isnan(x*x + y*y) or isinf(x*x + y*y):
            self.assertRaises(OverflowError)
        else:
            vtest = Point2d(x, y)
            self.assertAlmostEqual(vtest.norm()**2, vtest.sqnorm())

    @given(*finite_floats(2))
    def test_length_of_unit_vectors(self, x, y):
        assume(x*x + y*y != 0)
        if isnan(x*x + y*y) or isinf(x*x + y*y):
            self.assertRaises(OverflowError)
        else:
            vtest = Point2d(x, y)
            u = vtest.unit()
            vtest.normalize()
            self.assertAlmostEqual(u.norm(), 1.0)
            self.assertAlmostEqual(vtest.norm(), 1.0)

    @given(*finite_floats(2))
    def test_vector_equals_magnitude_times_direction(self, x, y):
        vtest = Point2d(x,y)
        if vtest == Point2d(0, 0):
            self.assertRaises(ZeroDivisionError)
        elif isnan(x*x + y*y) or isinf(x*x + y*y):
            self.assertRaises(OverflowError)
        else:
            vscaled = vtest.norm()*vtest.unit()
            self.assertAlmostEqual(vtest.x, vscaled.x)
            self.assertAlmostEqual(vtest.y, vscaled.y)

class TestPoint2dVectorAlgebra(unittest.TestCase):

    @given(*finite_floats(4))
    def test_coordinate_addition(self, a, b, c, d):
        vtest = Point2d(a, b) + Point2d(c, d)
        self.assertEqual(vtest, Point2d(a + c, b + d))

    @given(*finite_floats(2))
    def test_negation_subtraction_inverses(self, x, y):
        from aiboids.point2d import ZERO_VECTOR
        vtest = Point2d(x,y)
        self.assertEqual(vtest + (-vtest), ZERO_VECTOR)
        self.assertEqual(vtest - vtest, ZERO_VECTOR)

    @given(*finite_floats(2))
    def test_doubling_vs_addition(self, x, y):
        vtest = Point2d(x, y)
        self.assertEqual(2*vtest, vtest + vtest)

    @given(*finite_floats(4))
    def test_cauchy_schwarz_inequality(self, a, b, c, d):
        if isnan(a*a + b*b) or isinf(a*a + b*b):
            self.assertRaises(OverflowError)
        elif isnan(c*c + d*d) or isinf(c*c + d*d):
            self.assertRaises(OverflowError)
        else:
            vtest = Point2d(a, b)
            wtest = Point2d(c, d)
            if isnan(vtest*wtest) or isnan(vtest.norm()*wtest.norm()):
                self.assertRaises(OverflowError)
            else:
                self.assertLessEqual(abs(vtest*wtest), vtest.norm()*wtest.norm())

class TestPoint2dRotations(unittest.TestCase):

    @given(*finite_floats(3))
    def negative_rotation_is_inverse(self, x, y, theta):
        vtest = Point2d(x, y)
        vrot = vtest.rotated_by(theta)
        self.assertEqual(vtest, vrot.rotated_by(-theta))

    @given(*finite_floats(2))
    def test_orthogonal_dot_product(self, x, y):
        if isnan(x*y) or isinf(x*y):
            self.assertRaises(OverflowError)
        else:
            self.assertEqual(Point2d(x,y)*Point2d(y,-x), 0.0)
            self.assertEqual(Point2d(x,y)*Point2d(-y,x), 0.0)

    @given(*finite_floats(2))
    def test_rotations_are_orthogonal(self, x, y):
        from math import pi
        vtest = Point2d(x, y)
        vperp = vtest.rotated_by(pi/2)
        vperp2 = vtest.rotated_by(-pi/2)
        self.assertAlmostEqual(vtest*vperp, 0.0)
        self.assertAlmostEqual(vtest*vperp2, 0.0)

    @given(*finite_floats(2))
    def test_rotations_are_orthogonal_in_degrees(self, x, y):
        vtest = Point2d(x, y)
        vperp = vtest.rotated_by(90, True)
        vperp2 = vtest.rotated_by(-90, True)
        self.assertAlmostEqual(vtest*vperp, 0.0)
        self.assertAlmostEqual(vtest*vperp2, 0.0)



if __name__ == '__main__':
    unittest.main()
