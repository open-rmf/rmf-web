from uuid import uuid4

import rclpy
from rmf_door_msgs.msg import DoorRequest
from rmf_lift_msgs.msg import LiftRequest


class RosNode(rclpy.node.Node):
    def __init__(self):
        super().__init__(f"rest_server_{uuid4().hex}")
        self.door_req = self.create_publisher(DoorRequest, "adapter_door_requests", 10)
        self.lift_req = self.create_publisher(LiftRequest, "adapter_lift_requests", 10)
