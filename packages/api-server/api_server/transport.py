import asyncio

import rclpy
from rclpy.subscription import Subscription
from rmf_door_msgs.msg import DoorState
from rosidl_runtime_py.convert import message_to_ordereddict

from .rmf_io import RmfIO, RmfGateway


class RmfTransport():
    def __init__(self, ros2_node: rclpy.node.Node, rmf_gateway: RmfGateway):
        self.ros2_node = ros2_node
        self.rmf_gateway = rmf_gateway
        self.door_states_sub: Optional[Subscription] = None

    def subscribe_all(self):
        self.door_states_sub = self.ros2_node.create_subscription(
            DoorState, 'door_states', lambda msg: self.rmf_gateway.door_states.on_next(message_to_ordereddict(msg)), 10)

    def unsubscribe_all(self):
        if self.door_states_sub:
            self.door_states_sub.destroy()
            self.door_states_sub = None
