# pylint: disable=global-statement
import os
import threading

import rclpy
import rclpy.node

_spin_thread: threading.Thread = None  # type: ignore

use_sim_time_env = os.environ.get("RMF_SERVER_USE_SIM_TIME", None)
if use_sim_time_env:
    use_sim_time = not use_sim_time_env.lower() in ["0", "false"]
else:
    use_sim_time = False
if use_sim_time:
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
else:
    rclpy.init()

ros_node = rclpy.node.Node("rmf_api_server")


def ros_spin():
    global _spin_thread
    _spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node))
    _spin_thread.start()


def ros_shutdown():
    rclpy.shutdown()
    global _spin_thread
    _spin_thread.join()
    del _spin_thread
