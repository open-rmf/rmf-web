# pragma: no cover

import asyncio
import base64
import hashlib
import logging
from typing import Any, List, Optional

import rclpy
import rclpy.client
import rclpy.qos
from builtin_interfaces.msg import Time as RosTime
from fastapi import HTTPException
from rclpy.subscription import Subscription
from rmf_building_map_msgs.msg import AffineImage as RmfAffineImage
from rmf_building_map_msgs.msg import BuildingMap as RmfBuildingMap
from rmf_building_map_msgs.msg import Level as RmfLevel
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest
from rmf_door_msgs.msg import DoorState as RmfDoorState
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest
from rmf_lift_msgs.msg import LiftState as RmfLiftState
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask
from rosidl_runtime_py.convert import message_to_ordereddict

from .logger import logger as base_logger
from .models import BuildingMap, DispenserState, DoorState, IngestorState, LiftState
from .repositories import CachedFilesRepository, cached_files_repo
from .rmf_io import rmf_events
from .ros import ros_node


def process_building_map(
    rmf_building_map: RmfBuildingMap,
    cached_files: CachedFilesRepository,
) -> BuildingMap:
    """
    1. Converts a `BuildingMap` message to an ordered dict.
    2. Saves the images into `{cache_directory}/{map_name}/`.
    3. Change the `AffineImage` `data` field to the url of the image.
    """
    processed_map = message_to_ordereddict(rmf_building_map)

    for i, level in enumerate(rmf_building_map.levels):
        level: RmfLevel
        for j, image in enumerate(level.images):
            image: RmfAffineImage
            # look at non-crypto hashes if we need more performance
            sha1_hash = hashlib.sha1()
            sha1_hash.update(image.data)
            fingerprint = base64.b32encode(sha1_hash.digest()).lower().decode()
            relpath = f"{rmf_building_map.name}/{level.name}-{image.name}.{fingerprint}.{image.encoding}"  # pylint: disable=line-too-long
            urlpath = cached_files.add_file(image.data, relpath)
            processed_map["levels"][i]["images"][j]["data"] = urlpath
    return BuildingMap(**processed_map)


class RmfGateway:
    def __init__(
        self,
        cached_files: CachedFilesRepository,
        *,
        logger: Optional[logging.Logger] = None,
    ):
        self._door_req = ros_node().create_publisher(
            RmfDoorRequest, "adapter_door_requests", 10
        )
        self._lift_req = ros_node().create_publisher(
            RmfLiftRequest, "adapter_lift_requests", 10
        )
        self._submit_task_srv = ros_node().create_client(RmfSubmitTask, "submit_task")
        self._cancel_task_srv = ros_node().create_client(RmfCancelTask, "cancel_task")

        self.cached_files = cached_files
        self.logger = logger or base_logger.getChild(self.__class__.__name__)
        self._subscriptions: List[Subscription] = []

        self._subscribe_all()

    async def call_service(self, client: rclpy.client.Client, req, timeout=1) -> Any:
        """
        Utility to wrap a ros service call in an awaitable,
        raises HTTPException if service call fails.
        """
        fut = client.call_async(req)
        try:
            result = await asyncio.wait_for(fut, timeout=timeout)
            return result
        except asyncio.TimeoutError as e:
            raise HTTPException(503, "ros service call timed out") from e

    def _subscribe_all(self):
        door_states_sub = ros_node().create_subscription(
            RmfDoorState,
            "door_states",
            lambda msg: rmf_events.door_states.on_next(DoorState.from_orm(msg)),
            10,
        )
        self._subscriptions.append(door_states_sub)

        def convert_lift_state(lift_state: RmfLiftState):
            dic = message_to_ordereddict(lift_state)
            return LiftState(**dic)

        lift_states_sub = ros_node().create_subscription(
            RmfLiftState,
            "lift_states",
            lambda msg: rmf_events.lift_states.on_next(convert_lift_state(msg)),
            10,
        )
        self._subscriptions.append(lift_states_sub)

        dispenser_states_sub = ros_node().create_subscription(
            RmfDispenserState,
            "dispenser_states",
            lambda msg: rmf_events.dispenser_states.on_next(
                DispenserState.from_orm(msg)
            ),
            10,
        )
        self._subscriptions.append(dispenser_states_sub)

        ingestor_states_sub = ros_node().create_subscription(
            RmfIngestorState,
            "ingestor_states",
            lambda msg: rmf_events.ingestor_states.on_next(IngestorState.from_orm(msg)),
            10,
        )
        self._subscriptions.append(ingestor_states_sub)

        map_sub = ros_node().create_subscription(
            RmfBuildingMap,
            "map",
            lambda msg: rmf_events.building_map.on_next(
                process_building_map(msg, self.cached_files)
            ),
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_ALL,
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscriptions.append(map_sub)

    @staticmethod
    def now() -> Optional[RosTime]:
        """
        Returns the current sim time, or `None` if not using sim time
        """
        return ros_node().get_clock().now().to_msg()

    def request_door(self, door_name: str, mode: int) -> None:
        msg = RmfDoorRequest(
            door_name=door_name,
            request_time=ros_node().get_clock().now().to_msg(),
            requester_id=ros_node().get_name(),  # FIXME: use username
            requested_mode=RmfDoorMode(
                value=mode,
            ),
        )
        self._door_req.publish(msg)

    def request_lift(
        self, lift_name: str, destination: str, request_type: int, door_mode: int
    ):
        msg = RmfLiftRequest(
            lift_name=lift_name,
            request_time=ros_node().get_clock().now().to_msg(),
            session_id=ros_node().get_name(),
            request_type=request_type,
            destination_floor=destination,
            door_state=door_mode,
        )
        self._lift_req.publish(msg)


_rmf_gateway: RmfGateway


def rmf_gateway() -> RmfGateway:
    return _rmf_gateway


def startup():
    """
    Starts subscribing to all ROS topics.
    Must be called after the ros node is created and before spinning the it.
    """
    global _rmf_gateway
    _rmf_gateway = RmfGateway(cached_files_repo)
    return _rmf_gateway
