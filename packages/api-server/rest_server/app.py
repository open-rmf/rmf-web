import logging
import os
import sys
import threading

import rclpy
import socketio
from fastapi import APIRouter, FastAPI
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_door_msgs.msg import DoorRequest as RmfDoorRequest
from rmf_lift_msgs.msg import LiftRequest as RmfLiftRequest

from .app_config import app_config
from .building_map import building_map_router
from .models import DoorMode, LiftRequest
from .node import RosNode

# There is a bug with uvicorn and socketio that causes
# `RuntimeError: Event loop stopped before Future completed.`
# when getting SIGINT, the reason is because both uvicorn and socketio installs signal
# handlers, socketio's handler overwrites uvicorn's and in its handler to calls
# `asyncio.get_event_loop().stop()`.
# This forces the event loop to stop before uvicorn completes its work, hence the error appears.
# The side effect is that the shutdown handler is never called because the loop has
# been stopped.
#
# This is a hack to stop socketio from installing signal handlers.
socketio.asyncio_client.engineio.asyncio_client.async_signal_handler_set = True

logger = logging.getLogger("rest_app")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
if "RMF_REST_SERVER_DEBUG" in os.environ:
    logger.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)

app = FastAPI(
    openapi_url=f"{app_config.root_path}/openapi.json",
    docs_url=f"{app_config.root_path}/docs",
    swagger_ui_oauth2_redirect_url=f"{app_config.root_path}/docs/oauth2-redirect",
)
router = APIRouter(prefix=app_config.root_path)
sio = socketio.AsyncClient(logger=logger)
node: RosNode


def rclpy_main():
    global node  # pylint: disable=global-statement

    rclpy.init()
    node = RosNode()

    logger.info("start spinning rclpy node")
    rclpy.spin(node)
    logger.info("finished spinning rclpy node")


@app.on_event("startup")
async def on_startup():
    threading.Thread(target=rclpy_main).start()

    try:
        await sio.connect(app_config.api_server_url)
    except socketio.exceptions.ConnectionError:
        logger.error(
            "unable to connect to socketio server, some functions will not be available"
        )
        await sio.disconnect()

    if sio.connected:
        app.include_router(
            building_map_router(sio, logger.getChild("building_map")),
            prefix="/building_map",
        )

    app.include_router(router)

    logger.info("started app")


@app.on_event("shutdown")
async def on_shutdown():
    await sio.disconnect()
    rclpy.shutdown()
    logger.info("shutdown app")


@router.post("/doors/{door_name}/request")
async def post_door_request(door_name: str, mode: DoorMode):
    msg = RmfDoorRequest(
        door_name=door_name,
        request_time=node.get_clock().now().to_msg(),
        requester_id=node.get_name(),
        requested_mode=RmfDoorMode(
            value=mode.mode,
        ),
    )
    node.door_req.publish(msg)


@router.post("/lifts/{lift_name}/request")
async def post_lift_request(lift_name: str, request: LiftRequest):
    msg = RmfLiftRequest(
        lift_name=lift_name,
        request_time=node.get_clock().now().to_msg(),
        session_id=node.get_name(),
        request_type=request.mode,
        destination_floor=request.destination,
        door_state=request.door_mode,
    )
    node.door_req.publish(msg)
