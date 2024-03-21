import os
import threading

import rclpy
import rclpy.node

from .app_config import app_config

_spin_thread: threading.Thread
_ros_node: rclpy.node.Node


def ros_node():
    return _ros_node


def startup():
    """
    Initializes the rcl context and creates the ros node.
    Must be called before all calls to `ros_node()`.
    """
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
    _ros_node = rclpy.node.Node("rmf_api_server")  # type: ignore rclpy has invalid typing


def shutdown():
    """
    Shuts down the rcl context and wait for the spin thread to finish.
    """
    rclpy.shutdown()
    _spin_thread.join()


def spin_background():
    """
    Start spinning the ros node in a background thread.
    Must be called after setting up all subscriptions to avoid missing messages.
    """
    global _spin_thread
    _spin_thread = threading.Thread(target=lambda: rclpy.spin(_ros_node))
    _spin_thread.start()
