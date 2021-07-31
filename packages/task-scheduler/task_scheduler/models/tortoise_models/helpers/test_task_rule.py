import datetime
import unittest

from .task_rule import calculate_next_time


class TestCase(unittest.TestCase):
    """Basic test case which all other test cases inherit from"""

    def setUp(self):
        """Sets up the testing environment"""
        pass

    def tearDown(self):
        """Tears down the testing environment"""
        pass

    def test_calculate_next_time(self):
        """Tests the calculate_next_time method"""
        now = datetime.now()
        # calculate_next_time(now,
