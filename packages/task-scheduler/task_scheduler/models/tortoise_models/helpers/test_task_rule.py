import datetime
import unittest

from .task_rule import calculate_next_time, is_time_between


class TestCase(unittest.TestCase):
    """Basic test case which all other test cases inherit from"""

    def setUp(self):
        """Sets up the testing environment"""
        pass

    def tearDown(self):
        """Tears down the testing environment"""
        pass

    def test_calculate_next_time_with_date(self):
        """Tests the calculate_next_time method"""
        now = datetime.now()
        delta = datetime.timedelta(days=1)
        self.assertEqual(calculate_next_time(now, delta).date, (now + delta).date)

    def test_calculate_next_time_with_time(self):
        """Tests the calculate_next_time method with a time"""
        now = datetime.now()
        delta = datetime.timedelta(days=1)
        self.assertEqual(
            calculate_next_time(now, delta, now.time()).time(), (now + delta).time()
        )

    def test_calculate_next_time_with_time_and_date(self):
        """Tests the calculate_next_time method with a time and a date"""
        now = datetime.now()
        delta = datetime.timedelta(minutes=1)
        self.assertEqual(
            calculate_next_time(now, delta, now.time(), now.date()).date(),
            (now + delta).date(),
        )

    def test_is_time_between(self):
        """Tests the is_time_between method"""
        now = datetime.now()
        delta = datetime.timedelta(minutes=1)
        self.assertTrue(is_time_between(now, now + delta, now.time()))
        self.assertFalse(is_time_between(now, now + delta, now.time() + delta))
        self.assertFalse(is_time_between(now, now + delta, now.time() - delta))
        self.assertFalse(
            is_time_between(now, now + delta, now.time() + delta, now.date())
        )
