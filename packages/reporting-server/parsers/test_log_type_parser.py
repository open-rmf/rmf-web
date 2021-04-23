import unittest

from models.raw_log import LogLevel

from .log_type_parser import get_log_type


class TestCaseLogTypeParserCase(unittest.TestCase):
    def setUp(self):
        pass

    def test_warning_messages(self):
        self.assertEqual(get_log_type("this is a Warn"), LogLevel.WARN)
        self.assertEqual(get_log_type("WARN:"), LogLevel.WARN)
        self.assertEqual(get_log_type("WARNING:"), LogLevel.WARN)
        self.assertEqual(get_log_type("warn"), LogLevel.WARN)

    def test_error_messages(self):
        self.assertEqual(get_log_type("this is an Error"), LogLevel.ERROR)
        self.assertEqual(get_log_type("ERROR:"), LogLevel.ERROR)
        self.assertEqual(get_log_type("error"), LogLevel.ERROR)

    def test_debug_messages(self):
        self.assertEqual(get_log_type("this is a Debug msg"), LogLevel.DEBUG)
        self.assertEqual(get_log_type("DEBUG:"), LogLevel.DEBUG)
        self.assertEqual(get_log_type("debug"), LogLevel.DEBUG)

    def test_info_messages(self):
        self.assertEqual(get_log_type("this is an Info msg"), LogLevel.INFO)
        self.assertEqual(get_log_type("INFO:"), LogLevel.INFO)
        self.assertEqual(get_log_type("info"), LogLevel.INFO)
