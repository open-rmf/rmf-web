import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from rmf_door_msgs.msg import DoorState

from .rmf_io import RmfIO


class RmfSource():
    def __init__(self, ros2_node: rclpy.Node, rmf_io: ):
        self.rmf_io = rmf_io
        self.door_states_sub = ros2_node.create_subscription(
            DoorState, 'door_states',
            lambda msg: self.rmf_io.on_door_state(message_to_ordereddict(msg)), 10)
