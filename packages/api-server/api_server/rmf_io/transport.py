from typing import Optional

import rclpy
from building_map_msgs.msg import BuildingMap
from rclpy.node import Node as RosNode
from rclpy.subscription import Subscription
from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState

from .rmf_io import RmfGateway


class RmfTransport:
    def __init__(self, ros2_node: RosNode, rmf_gateway: RmfGateway):
        self.ros2_node = ros2_node
        self.rmf_gateway = rmf_gateway
        self.door_states_sub: Optional[Subscription] = None
        self.building_map_sub: Optional[Subscription] = None

    def subscribe_all(self):
        self.door_states_sub = self.ros2_node.create_subscription(
            DoorState,
            "door_states",
            self.rmf_gateway.door_states.on_next,
            10,
        )

        self.lift_states_sub = self.ros2_node.create_subscription(
            LiftState,
            "lift_states",
            self.rmf_gateway.lift_states.on_next,
            10,
        )

        self.dispenser_states_sub = self.ros2_node.create_subscription(
            DispenserState,
            "dispenser_states",
            self.rmf_gateway.dispenser_states.on_next,
            10,
        )

        self.ingestor_states_sub = self.ros2_node.create_subscription(
            IngestorState,
            "ingestor_states",
            self.rmf_gateway.ingestor_states.on_next,
            10,
        )

        self.fleet_states_sub = self.ros2_node.create_subscription(
            FleetState,
            "fleet_states",
            self.rmf_gateway.fleet_states.on_next,
            10,
        )

        self.building_map_sub = self.ros2_node.create_subscription(
            BuildingMap,
            "map",
            self.rmf_gateway.building_map.on_next,
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def unsubscribe_all(self):
        if self.door_states_sub:
            self.door_states_sub.destroy()
            self.door_states_sub = None

        if self.lift_states_sub:
            self.lift_states_sub.destroy()
            self.lift_state_subs = None

        if self.dispenser_states_sub:
            self.dispenser_states_sub.destroy()
            self.dispenser_states_sub = None

        if self.ingestor_states_sub:
            self.ingestor_states_sub.destroy()
            self.ingestor_states_sub = None

        if self.fleet_states_sub:
            self.fleet_states_sub.destroy()
            self.fleet_states_sub = None

        if self.building_map_sub:
            self.building_map_sub.destroy()
            self.building_map_sub = None
