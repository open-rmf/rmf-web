import asyncio
import logging
from datetime import datetime
from typing import cast

import rclpy
import rclpy.node
import rclpy.qos
from rclpy.subscription import Subscription
from rmf_task_msgs.msg import Alert as RmfAlert
from rmf_task_msgs.msg import AlertResponse as RmfAlertResponse
from tortoise.exceptions import IntegrityError

from api_server.exceptions import AlreadyExistsError, InvalidInputError, NotFoundError
from api_server.fast_io.singleton_dep import singleton_dep
from api_server.repositories.alerts import AlertRepository
from api_server.rmf_io.events import AlertEvents, get_alert_events
from api_server.ros import get_ros_node

from .models import AlertParameter, AlertRequest


class AlertsGateway:
    def __init__(
        self,
        ros_node: rclpy.node.Node,
        alert_events: AlertEvents,
        alert_repo: AlertRepository,
        loop: asyncio.AbstractEventLoop,
        *,
        logger: logging.Logger | None = None,
    ):
        self._ros_node = ros_node
        self._alert_events = alert_events
        self._alert_repo = alert_repo
        self._loop = loop
        self._logger = logger or logging.getLogger()

        self.transient_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._alert_response = self._ros_node.create_publisher(
            RmfAlertResponse,
            "alert_response",
            self.transient_qos,
        )

        self._subscriptions: list[Subscription] = []
        self._subscribe_all()

    def _subscribe_all(self):
        def convert_alert(msg):
            alert = cast(RmfAlert, msg)
            tier = AlertRequest.Tier.Info
            if alert.tier == RmfAlert.TIER_WARNING:
                tier = AlertRequest.Tier.Warning
            elif alert.tier == RmfAlert.TIER_ERROR:
                tier = AlertRequest.Tier.Error

            parameters = []
            for p in alert.alert_parameters:
                parameters.append(AlertParameter(name=p.name, value=p.value))

            responses_available = cast(list[str], alert.responses_available)
            return AlertRequest(
                id=alert.id,
                unix_millis_alert_time=round(datetime.now().timestamp() * 1000),
                title=alert.title,
                subtitle=alert.subtitle,
                message=alert.message,
                display=alert.display,
                tier=tier,
                responses_available=responses_available,
                alert_parameters=parameters,
                task_id=alert.task_id if len(alert.task_id) > 0 else None,
            )

        def handle_alert(alert: AlertRequest):
            async def create_alert(alert: AlertRequest):
                try:
                    created_alert = await self._alert_repo.create_new_alert(alert)
                except IntegrityError as e:
                    logging.error("%s, %s", str(e), alert)
                    return
                except AlreadyExistsError as e:
                    logging.error("%s, %s", str(e), alert)
                    return
                if not created_alert:
                    logging.error("Failed to create alert: %s", alert)
                    return

                self._alert_events.alert_requests.on_next(created_alert)
                logging.debug("%s", alert)

            logging.info(f"Received alert: {alert}")
            self._loop.create_task(create_alert(alert))

        alert_sub = self._ros_node.create_subscription(
            RmfAlert,
            "alert",
            lambda msg: handle_alert(convert_alert(msg)),
            self.transient_qos,
        )
        self._subscriptions.append(alert_sub)

        # FIXME(ac): Due to also subscribing to alert responses, this callback
        # gets triggered as well even if the response is called through REST,
        # which publishes a ROS 2 message and gets picked up by this subscriber.
        # This causes alert_repo.create_response to be called twice in total,
        # resulting in a conflict of responses for the same alert ID. This does
        # not cause any issues, just that an error log is produced.
        def handle_alert_response(msg):
            msg = cast(RmfAlertResponse, msg)

            async def create_response(alert_id: str, response: str):
                try:
                    created_response = await self._alert_repo.create_response(
                        msg.id, msg.response
                    )
                except IntegrityError as e:
                    logging.error(
                        "%s, id: %s, response: %s", str(e), alert_id, response
                    )
                    return
                except AlreadyExistsError as e:
                    logging.error(
                        "%s, id: %s, response: %s", str(e), alert_id, response
                    )
                    return
                except NotFoundError as e:
                    logging.error(
                        "%s, id: %s, response: %s", str(e), alert_id, response
                    )
                    return
                except InvalidInputError as e:
                    logging.error(
                        "%s, id: %s, response: %s", str(e), alert_id, response
                    )
                    return
                if not created_response:
                    logging.error(
                        f"Failed to create alert response [{msg.response}] for alert id [{msg.id}]"
                    )
                    return

                self._alert_events.alert_responses.on_next(created_response)
                logging.debug("%s", created_response)

            logging.info(f"Received response [{msg.response}] for alert id [{msg.id}]")
            self._loop.create_task(create_response(msg.id, msg.response))

        alert_response_sub = self._ros_node.create_subscription(
            RmfAlertResponse,
            "alert_response",
            handle_alert_response,
            self.transient_qos,
        )
        self._subscriptions.append(alert_response_sub)

    async def __aexit__(self, *exc):
        for sub in self._subscriptions:
            sub.destroy()

    def respond_to_alert(self, alert_id: str, response: str):
        msg = RmfAlertResponse()
        msg.id = alert_id
        msg.response = response
        self._alert_response.publish(msg)


@singleton_dep
def get_alerts_gateway():
    return AlertsGateway(
        get_ros_node(),
        get_alert_events(),
        AlertRepository(),
        asyncio.get_event_loop(),
    )
