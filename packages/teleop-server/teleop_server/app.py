import ast
import copy
import json

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from teleop_server.models import FloatModel, StringModel
from teleop_server.msgs import TeleoperationMessages
from teleop_server.room import WebSocketRoom

app = FastAPI()
websocket_rooms = {}
WEBSOCKET_PREFIX = "/ws"


async def _websocket_sub_only(room: str, websocket: WebSocket):
    if room not in websocket_rooms.keys():
        websocket_rooms[room] = WebSocketRoom()

    await websocket_rooms[room].connect(websocket)
    while True:
        try:
            await websocket.receive_text()
            pass
        except WebSocketDisconnect:
            await websocket_rooms[room].disconnect(websocket)
            # TODO(BH): Print out identifying information of websocket
            print("Disconnected.")
            break
        except Exception as e:
            print("error:", e)


def _get_websocket_room_path(url_path: str):
    return f"{WEBSOCKET_PREFIX}{url_path}"


@app.post("/teleop/{robot_id}/video/join_room", tags=["Teleoperation - Video"])
async def join_video_room(robot_id: str, room_id: StringModel, request: Request):
    room = _get_websocket_room_path(request.url.path)
    print(room)
    try:
        if room in websocket_rooms.keys():
            resp = copy.deepcopy(
                TeleoperationMessages.TELEOPERATION_JOIN_VIDEO_ROOM_DEFINITION
            )
            resp["robot_id"] = robot_id
            resp["room_id"] = room_id.data
            await websocket_rooms[room].broadcast(json.dumps(resp))
            return True
        else:
            print("No websocket listeners were found.")
            return False
    except Exception as e:
        print(f"error: {e}")
        return False


@app.websocket("/ws/teleop/{robot_id}/video/join_room")
async def join_video_room_ws(robot_id: str, websocket: WebSocket):
    await _websocket_sub_only(websocket.url.path, websocket)


@app.post("/teleop/{robot_id}/video/leave_room", tags=["Teleoperation - Video"])
async def leave_video_room(robot_id: str, room_id: StringModel, request: Request):
    room = _get_websocket_room_path(request.url.path)
    try:
        if room in websocket_rooms.keys():
            resp = copy.deepcopy(
                TeleoperationMessages.TELEOPERATION_LEAVE_VIDEO_ROOM_DEFINITION
            )
            resp["robot_id"] = robot_id
            resp["room_id"] = room_id.data
            await websocket_rooms[room].broadcast(json.dumps(resp))
            return True
        else:
            print("No websocket listeners were found.")
            return False
    except Exception as e:
        print(f"error: {e}")
        return False


@app.websocket("/ws/teleop/{robot_id}/video/leave_room")
async def leave_video_room_ws(robot_id: str, websocket: WebSocket):
    await _websocket_sub_only(websocket.url.path, websocket)


@app.post("/teleop/{robot_id}/drive", tags=["Teleoperation - Discrete"])
async def drive_robot_in_meters_forward(
    robot_id: str, x_m: FloatModel, x_vel_m_s: FloatModel, request: Request
):
    room = _get_websocket_room_path(request.url.path)
    try:
        if room in websocket_rooms.keys():
            resp = copy.deepcopy(
                TeleoperationMessages.TELEOPERATION_DRIVE_ROBOT_IN_METERS_FORWARD_DEFINITION
            )
            resp["robot_id"] = robot_id
            resp["x_m"] = x_m.data
            resp["x_vel_m_s"] = x_vel_m_s.data
            await websocket_rooms[room].broadcast(json.dumps(resp))
            return True
        else:
            print("No websocket listeners were found.")
            return False
    except Exception as e:
        print(f"error: {e}")
        return False


@app.websocket("/ws/teleop/{robot_id}/drive")
async def drive(robot_id: str, websocket: WebSocket):
    await _websocket_sub_only(websocket.url.path, websocket)


@app.post("/teleop/{robot_id}/rotate", tags=["Teleoperation - Discrete"])
async def rotate_robot_in_radians_clockwise(
    robot_id: str, theta_rad: FloatModel, theta_vel_rad_s: FloatModel, request: Request
):

    room = _get_websocket_room_path(request.url.path)
    try:
        if room in websocket_rooms.keys():
            resp = copy.deepcopy(
                TeleoperationMessages.TELEOPERATION_ROTATE_ROBOT_IN_RADIANS_CLOCKWISE_DEFINITION
            )
            resp["robot_id"] = robot_id
            resp["theta_rad"] = theta_rad.data
            resp["theta_vel_rad_s"] = theta_vel_rad_s.data
            await websocket_rooms[room].broadcast(json.dumps(resp))
            return True
        else:
            print("No websocket listeners were found.")
            return False
    except Exception as e:
        print(f"error: {e}")
        return False


@app.websocket("/ws/teleop/{robot_id}/rotate")
async def rotate(robot_id: str, websocket: WebSocket):
    await _websocket_sub_only(websocket.url.path, websocket)


@app.websocket("/ws/teleop/{robot_id}/cmd_vel")
async def cmd_vel(robot_id: str, websocket: WebSocket):
    room = websocket.url.path
    if room not in websocket_rooms.keys():
        websocket_rooms[room] = WebSocketRoom()

    await websocket_rooms[room].connect(websocket)
    while True:
        try:
            ws_string = await websocket.receive_json()
            data = ast.literal_eval(ws_string)
            resp = copy.deepcopy(TeleoperationMessages.TELEOPERATION_CMD_VEL_DEFINITION)
            for key in resp.keys():
                resp[key] = data[key]
                # TODO(BH): Sanitization using Pydantic
            await websocket_rooms[room].broadcast(json.dumps(resp))
        except WebSocketDisconnect:
            await websocket_rooms[room].disconnect(websocket)
            # TODO(BH): Print out identifying information of websocket
            print("Disconnected.")
            break
        except KeyError as e:
            print(f"Missing JSON field: {e}")
        except Exception as e:
            print("error:", e)
