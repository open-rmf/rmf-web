import contextlib
import os
import threading

import rclpy
import rclpy.node

from api_server.fast_io.singleton_dep import singleton_dep

from .app_config import app_config


@singleton_dep
@contextlib.contextmanager
def get_ros_node():
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

    node = rclpy.node.Node("rmf_api_server")
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.start()

    yield node

    rclpy.shutdown()
    spin_thread.join()
