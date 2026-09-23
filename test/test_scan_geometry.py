import math
import json
from pathlib import Path
import unittest
import yaml
from armatron.scan_geometry import mask_ranges, validate_boxes


class ScanGeometryTest(unittest.TestCase):
    def test_configured_post_centres_are_masked_and_gap_is_visible(self):
        root = Path(__file__).parents[1]
        p = yaml.safe_load((root/'config/odometry/scan_filter.yaml').read_text())['scan_filter']['ros__parameters']
        boxes = validate_boxes(json.loads(p['profile_boxes']))
        self.assertGreaterEqual(len(boxes), 4)  # Additional configured self-masks are allowed.
        for x,y,*_ in boxes:
            distance, angle = math.hypot(x,y), math.atan2(y,x)
            self.assertTrue(math.isnan(mask_ranges([distance], angle, 0., .1, 10., boxes, p['mask_padding'])[0]))
        self.assertEqual(mask_ranges([.2], math.pi, 0., .1, 10., boxes, p['mask_padding']), [.2])

    def test_only_endpoints_inside_box_are_masked(self):
        box = validate_boxes([[.2, 0., .02, .02, 0.]])
        self.assertTrue(math.isnan(mask_ranges([.2], 0., 0., .1, 10., box)[0]))
        for distance in (.12, .18, .22, 1.):
            self.assertEqual(mask_ranges([distance], 0., 0., .1, 10., box), [distance])

    def test_mask_rotation_transform_and_padding(self):
        box = [[0., .2, .02, .02, math.pi/4]]
        self.assertTrue(math.isnan(mask_ranges([.2], 0., 0., .1, 10., box,
                                             scan_to_mask=(0.,0.,math.pi/2))[0]))
        self.assertTrue(math.isnan(mask_ranges([.212], 0., 0., .1, 10.,
                                             [[.2,0.,.02,.02,0.]], padding=.005)[0]))

    def test_unconfigured_masks_preserve_near_returns(self):
        values = [.12, .2, .34, 1., float('inf'), float('nan')]
        actual = mask_ranges(values, 0., .1, .1, 10., [])
        self.assertEqual(actual[:-1], values[:-1])
        self.assertTrue(math.isnan(actual[-1]))
        with self.assertRaises(ValueError):
            validate_boxes([[0.,0.,-1.,.02,0.]])

    def test_denoising_precedes_inflation_in_both_costmaps(self):
        root = Path(__file__).parents[1]
        config = yaml.safe_load((root / 'config/nav2/navigation.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            p = config[name][name]['ros__parameters']
            self.assertLess(p['plugins'].index('denoise_layer'), p['plugins'].index('inflation_layer'))
            self.assertEqual(p['denoise_layer']['minimal_group_size'], 2)
            self.assertEqual(p['denoise_layer']['group_connectivity_type'], 8)
        self.assertEqual(config['amcl']['ros__parameters']['laser_min_range'], -1.)
