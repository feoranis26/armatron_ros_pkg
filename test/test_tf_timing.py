import unittest
from armatron.tf_timing import Timing


class TimingTest(unittest.TestCase):
    def test_fresh_forward_dated_tf_and_receipt_gap(self):
        t = Timing()
        t.add(10.5, 10., 100.)
        t.add(10.55, 10.05, 100.05)
        report = t.report(10.1, 100.1)
        self.assertEqual(report['receipt_age_p95_s'], -.5)
        self.assertEqual(report['max_receipt_gap_s'], .05)
        self.assertEqual(report['repeated_stamps'], 0)

    def test_repeated_stamps_are_distinct_from_delivery_silence(self):
        t = Timing()
        t.add(10., 10., 100.)
        t.add(10., 11., 101.)
        t.add(9., 12., 102.)
        report = t.report(14., 104.)
        self.assertEqual(report['repeated_stamps'], 1)
        self.assertEqual(report['backward_stamps'], 1)
        self.assertEqual(report['seconds_since_last_receipt'], 2.)
        self.assertEqual(report['last_stamp_age_now_s'], 5.)
        self.assertEqual(report['receipt_age_max_s'], 3.)

    def test_missing_stream_is_reported(self):
        self.assertEqual(Timing().report(10., 10.), {'samples': 0})
