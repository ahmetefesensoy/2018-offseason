import unittest

import numpy as np
from frc_perception.hsv_detector import HsvCubeDetector, depth_scale_for_encoding


class HsvDetectorTest(unittest.TestCase):
    def test_yellow_cube_is_detected_but_blue_region_and_tiny_noise_are_not(self):
        image = np.zeros((180, 240, 3), dtype=np.uint8)
        image[30:80, 40:90] = (0, 220, 220)
        image[100:150, 140:190] = (255, 0, 0)
        image[5:7, 5:7] = (0, 255, 255)
        result = HsvCubeDetector().infer(image)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][0], (40, 30, 90, 80))
        self.assertEqual(result[0][1], 0)
        self.assertEqual(HsvCubeDetector().infer(image), result)

    def test_empty_image_has_no_detections(self):
        self.assertEqual(HsvCubeDetector().infer(np.zeros((40, 40, 3), np.uint8)), ())

    def test_depth_encodings_cannot_scale_metres_as_millimetres(self):
        self.assertEqual(depth_scale_for_encoding("16UC1"), .001)
        self.assertEqual(depth_scale_for_encoding("32FC1"), 1.0)
        with self.assertRaises(ValueError):
            depth_scale_for_encoding("8UC1")

    def test_non_image_input_is_rejected(self):
        with self.assertRaises(ValueError):
            HsvCubeDetector().infer(np.zeros((40, 40), np.uint8))
