import unittest

import rclpy
import rclpy.node


class RosFixture(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        cls.rcl_ctx = rclpy.Context()
        rclpy.init(context=cls.rcl_ctx)
        cls.node = rclpy.node.Node("test_node", context=cls.rcl_ctx)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown(context=cls.rcl_ctx)
