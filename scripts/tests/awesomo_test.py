#!/usr/bin/env python2
import sys
import os
import unittest
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from atl import CarrotController


class atlTests(unittest.TestCase):
    def setUp(self):
        waypoints = [
            np.array([0, 0]),
            np.array([1, 0]),
            np.array([1, 1]),
            np.array([0, 1]),
            np.array([0, 0])
        ]
        look_ahead_dist = 0.1
        self.carrot_controller = CarrotController(waypoints, look_ahead_dist)

    def test_closest_point(self):
        wp_start = np.array([0, 0])
        wp_end = np.array([5, 0])
        point = np.array([3, 0])

        closest, retval = self.carrot_controller.closest_point(
            wp_start,
            wp_end,
            point
        )
        np.testing.assert_array_equal(closest, np.array([3, 0]))
        self.assertEqual(retval, 0)

    def test_carrot_point(self):
        wp_start = np.array([0, 0])
        wp_end = np.array([5, 0])
        pos = np.array([3, 0])
        r = 1.0

        carrot, retval = self.carrot_controller.carrot_point(
            pos,
            r,
            wp_start,
            wp_end
        )
        np.testing.assert_array_equal(carrot, np.array([4, 0]))

    def test_update(self):
        position = [0, 0.5]
        carrot_pt = self.carrot_controller.update(position)
        np.testing.assert_array_equal(carrot_pt, np.array([0.1, 0]))
        self.assertEqual(len(self.carrot_controller.waypoints), 5)

        position = [1.0, 0]
        carrot_pt = self.carrot_controller.update(position)
        np.testing.assert_array_equal(carrot_pt, np.array([1.1, 0]))
        self.assertEqual(len(self.carrot_controller.waypoints), 4)


if __name__ == "__main__":
    unittest.main()
