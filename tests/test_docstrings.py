#!/usr/bin/env python3
"""
Test all submodule docstrings via doctest.
"""
import sys
import doctest
import unittest
from unittest.mock import Mock
from context import aiboids

class TestDocstrings(unittest.TestCase):

    def test_docstrings_base_entity(self):
        from aiboids import base_entity
        doctest.testmod(base_entity)

    def test_docstrings_pgrender(self):
        sys.modules['pygame'] = Mock()
        sys.modules['pygame.locals'] = Mock()
        from aiboids import pgrender
        doctest.testmod(pgrender)

    def test_docstrings_point2d(self):
        from aiboids import point2d
        doctest.testmod(point2d)

    def test_docstrings_statemachine(self):
        from aiboids import statemachine
        doctest.testmod(statemachine)

    def test_docstrings_steering(self):
        from aiboids import steering
        doctest.testmod(steering)

    def test_docstrings_vehicle2d(self):
        from aiboids import vehicle2d
        doctest.testmod(vehicle2d)

if __name__ == '__main__':
    unittest.main()
