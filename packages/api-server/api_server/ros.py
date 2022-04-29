# pylint: disable=global-statement
import os
import threading

import rclpy
import rclpy.node

from .app_config import app_config

_spin_thread: threading.Thread
_ros_node: rclpy.node.Node


def ros_node():
    return _ros_node


def setup():
    global _spin_thread
    global _ros_node

    # TODO: remove support for `RMF_SERVER_USE_SIM_TIME`?
    use_sim_time_env = os.environ.get("RMF_SERVER_USE_SIM_TIME", None)
    if use_sim_time_env:
        use_sim_time = not use_sim_time_env.lower() in ["0", "false"]
    else:
        use_sim_time = False
    if use_sim_time:
        rclpy.init(
            args=["--ros-args"] + app_config.ros_args + ["-p", "use_sim_time:=true"]
        )
    else:
        rclpy.init(args=["--ros-args"] + app_config.ros_args)

    _ros_node = rclpy.node.Node("rmf_api_server")
    _spin_thread = threading.Thread(target=lambda: rclpy.spin(_ros_node))
    _spin_thread.start()


def shutdown():
    rclpy.shutdown()
    _spin_thread.join()


def ros_spin():
    global _spin_thread
    _spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node))
    _spin_thread.start()


def ros_shutdown():
    rclpy.shutdown()
    global _spin_thread
    _spin_thread.join()
    del _spin_thread
