import os
import threading
from typing import override

import rclpy
import rclpy.node

from api_server.fast_io.singleton_dep import SingletonDep

from .app_config import app_config


class RosNode(SingletonDep):
    def __init__(self):
        self.node: rclpy.node.Node
        self._spin_thread: threading.Thread

    async def __aenter__(self):
        """
        Initializes the rcl context and creates the ros node.
        """
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

        self.node = rclpy.node.Node("rmf_api_server")
        self._spin_thread = threading.Thread(target=lambda: rclpy.spin(self.node))
        self._spin_thread.start()

    async def __aexit__(self, *exc):
        """
        Shuts down the rcl context and wait for the spin thread to finish.
        """
        if self._spin_thread is None:
            raise AssertionError("_spin_thread is None, is the context entered?")
        rclpy.shutdown()
        self._spin_thread.join()
