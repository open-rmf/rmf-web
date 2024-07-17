# pragma: no cover

import asyncio
import base64
import hashlib
import logging
from datetime import datetime
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

# pylint: disable-next=no-name-in-module
from rmf_fleet_msgs.msg import BeaconState as RmfBeaconState
from rmf_fleet_msgs.msg import DeliveryAlert as RmfDeliveryAlert
from rmf_fleet_msgs.msg import DeliveryAlertAction as RmfDeliveryAlertAction
from rmf_fleet_msgs.msg import DeliveryAlertCategory as RmfDeliveryAlertCategory
from rmf_fleet_msgs.msg import DeliveryAlertTier as RmfDeliveryAlertTier
from rmf_fleet_msgs.msg import MutexGroupManualRelease as RmfMutexGroupManualRelease
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest
from rmf_lift_msgs.msg import LiftState as RmfLiftState
from rmf_task_msgs.msg import Alert as RmfAlert
from rmf_task_msgs.msg import AlertResponse as RmfAlertResponse
from rmf_task_msgs.srv import CancelTask as RmfCancelTask
from rmf_task_msgs.srv import SubmitTask as RmfSubmitTask
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool as BoolMsg

from .models import (
    AlertParameter,
    AlertRequest,
    AlertResponse,
    BeaconState,
    BuildingMap,
    DeliveryAlert,
    DispenserState,
    DoorState,
    FireAlarmTriggerState,
    IngestorState,
    LiftState,
)
from .models.delivery_alerts import action_from_msg, category_from_msg, tier_from_msg
from .repositories import CachedFilesRepository, cached_files_repo
from .rmf_io import alert_events, rmf_events
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
    def __init__(self, cached_files: CachedFilesRepository):
        self._door_req = ros_node().create_publisher(
            RmfDoorRequest, "adapter_door_requests", 10
        )

        transient_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._adapter_lift_req = ros_node().create_publisher(
            RmfLiftRequest, "adapter_lift_requests", transient_qos
        )
        self._submit_task_srv = ros_node().create_client(RmfSubmitTask, "submit_task")
        self._cancel_task_srv = ros_node().create_client(RmfCancelTask, "cancel_task")

        self._delivery_alert_response = ros_node().create_publisher(
            RmfDeliveryAlert,
            "delivery_alert_response",
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._alert_response = ros_node().create_publisher(
            RmfAlertResponse,
            "alert_response",
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._mutex_group_release = ros_node().create_publisher(
            RmfMutexGroupManualRelease,
            "mutex_group_manual_release",
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self._fire_alarm_trigger = ros_node().create_publisher(
            BoolMsg,
            "fire_alarm_trigger",
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.cached_files = cached_files
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
            100,
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

        def convert_beacon_state(beacon_state: RmfBeaconState):
            return BeaconState(
                id=beacon_state.id,
                online=beacon_state.online,
                category=beacon_state.category,
                activated=beacon_state.activated,
                level=beacon_state.level,
            )

        beacon_sub = ros_node().create_subscription(
            RmfBeaconState,
            "beacon_state",
            lambda msg: rmf_events.beacons.on_next(convert_beacon_state(msg)),
            100,
        )
        self._subscriptions.append(beacon_sub)

        def convert_delivery_alert(delivery_alert: RmfDeliveryAlert):
            category = category_from_msg(delivery_alert.category.value)
            tier = tier_from_msg(delivery_alert.tier.value)
            action = action_from_msg(delivery_alert.action.value)
            return DeliveryAlert(
                id=delivery_alert.id,  # pyright: ignore[reportGeneralTypeIssues]
                category=category,  # pyright: ignore[reportGeneralTypeIssues]
                tier=tier,  # pyright: ignore[reportGeneralTypeIssues]
                task_id=delivery_alert.task_id,  # pyright: ignore[reportGeneralTypeIssues]
                action=action,  # pyright: ignore[reportGeneralTypeIssues]
                message=delivery_alert.message,  # pyright: ignore[reportGeneralTypeIssues]
            )

        def handle_delivery_alert(delivery_alert: DeliveryAlert):
            logging.info("Received delivery alert:")
            logging.info(delivery_alert)
            rmf_events.delivery_alerts.on_next(delivery_alert)

        delivery_alert_request_sub = ros_node().create_subscription(
            RmfDeliveryAlert,
            "delivery_alert_request",
            lambda msg: handle_delivery_alert(convert_delivery_alert(msg)),
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscriptions.append(delivery_alert_request_sub)

        def convert_alert(alert: RmfAlert):
            tier = AlertRequest.Tier.Info
            if alert.tier == RmfAlert.TIER_WARNING:
                tier = AlertRequest.Tier.Warning
            elif alert.tier == RmfAlert.TIER_ERROR:
                tier = AlertRequest.Tier.Error

            parameters = []
            for p in alert.alert_parameters:
                parameters.append(AlertParameter(name=p.name, value=p.value))

            return AlertRequest(
                id=alert.id,
                unix_millis_alert_time=round(datetime.now().timestamp() * 1000),
                title=alert.title,
                subtitle=alert.subtitle,
                message=alert.message,
                display=alert.display,
                tier=tier,
                responses_available=alert.responses_available,
                alert_parameters=parameters,
                task_id=alert.task_id if len(alert.task_id) > 0 else None,
            )

        def handle_alert(alert: AlertRequest):
            logging.info(f"Received alert: {alert}")
            alert_events.alert_requests.on_next(alert)

        alert_sub = ros_node().create_subscription(
            RmfAlert,
            "alert",
            lambda msg: handle_alert(convert_alert(msg)),
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscriptions.append(alert_sub)

        def convert_alert_response(alert_response: RmfAlertResponse):
            return AlertResponse(
                id=alert_response.id,
                unix_millis_response_time=round(datetime.now().timestamp() * 1000),
                response=alert_response.response,
            )

        def handle_alert_response(alert_response: AlertResponse):
            logging.info(f"Received alert response: {alert_response}")
            alert_events.alert_responses.on_next(alert_response)

        alert_response_sub = ros_node().create_subscription(
            RmfAlertResponse,
            "alert_response",
            lambda msg: handle_alert_response(convert_alert_response(msg)),
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscriptions.append(alert_response_sub)

        def handle_fire_alarm_trigger(fire_alarm_trigger_msg: BoolMsg):
            if fire_alarm_trigger_msg.data:
                logging.info("Fire alarm triggered")
            else:
                logging.info("Fire alarm trigger reset")
            fire_alarm_trigger_state = FireAlarmTriggerState(
                unix_millis_time=round(datetime.now().timestamp() * 1000),
                trigger=fire_alarm_trigger_msg.data,
            )
            rmf_events.fire_alarm_trigger.on_next(fire_alarm_trigger_state)

        fire_alarm_trigger_sub = ros_node().create_subscription(
            BoolMsg,
            "fire_alarm_trigger",
            handle_fire_alarm_trigger,
            rclpy.qos.QoSProfile(
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscriptions.append(fire_alarm_trigger_sub)

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
        self,
        lift_name: str,
        destination: str,
        request_type: int,
        door_mode: int,
        additional_session_ids: List[str],
    ):
        msg = RmfLiftRequest(
            lift_name=lift_name,
            request_time=ros_node().get_clock().now().to_msg(),
            session_id=ros_node().get_name(),
            request_type=request_type,
            destination_floor=destination,
            door_state=door_mode,
        )
        self._adapter_lift_req.publish(msg)

        for session_id in additional_session_ids:
            msg.session_id = session_id
            self._adapter_lift_req.publish(msg)

    def respond_to_delivery_alert(
        self,
        alert_id: str,
        category: int,
        tier: int,
        task_id: str,
        action: int,
        message: str,
    ):
        msg = RmfDeliveryAlert()
        msg.id = alert_id
        msg.category = RmfDeliveryAlertCategory(value=category)
        msg.tier = RmfDeliveryAlertTier(value=tier)
        msg.task_id = task_id
        msg.action = RmfDeliveryAlertAction(value=action)
        msg.message = message
        self._delivery_alert_response.publish(msg)

    def respond_to_alert(self, alert_id: str, response: str):
        msg = RmfAlertResponse()
        msg.id = alert_id
        msg.response = response
        self._alert_response.publish(msg)

    def manual_release_mutex_groups(
        self,
        mutex_groups: List[str],
        fleet: str,
        robot: str,
    ):
        msg = RmfMutexGroupManualRelease()
        msg.release_mutex_groups = mutex_groups
        msg.fleet = fleet
        msg.robot = robot
        self._mutex_group_release.publish(msg)

    def reset_fire_alarm_trigger(self):
        reset_msg = BoolMsg()
        reset_msg.data = False
        self._fire_alarm_trigger.publish(reset_msg)


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
