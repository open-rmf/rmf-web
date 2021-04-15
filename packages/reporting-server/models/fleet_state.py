from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class RobotStateEnum(str, Enum):
    MODE_IDLE = "closed"
    MODE_CHARGING = "charging"
    MODE_MOVING = "moving"
    MODE_PAUSED = "paused"
    MODE_WAITING = "waiting"
    MODE_GOING_HOME = "going_home"
    MODE_DOCKING = "docking"
    MODE_EMERGENCY = "emergency"
    MODE_ADAPTER_ERROR = "adapter_error"


class RobotStateService:
    def get_state_name(self, state: int):
        if state == 0:
            return RobotStateEnum.MODE_IDLE
        elif state == 1:
            return RobotStateEnum.MODE_CHARGING
        elif state == 2:
            return RobotStateEnum.MODE_MOVING
        elif state == 3:
            return RobotStateEnum.MODE_PAUSED
        elif state == 4:
            return RobotStateEnum.MODE_WAITING
        elif state == 5:
            return RobotStateEnum.MODE_GOING_HOME
        elif state == 6:
            return RobotStateEnum.MODE_DOCKING
        elif state == 7:
            return RobotStateEnum.MODE_EMERGENCY
        elif state == 7:
            return RobotStateEnum.MODE_ADAPTER_ERROR


# {"name": "tinyRobot", "robots": [{"name": "tinyRobot1", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 11.55367374420166, "y": -11.317498207092285, "yaw": -1.5998055934906006, "level_name": "L1", "index": 0}, "path": []}, {"name": "tinyRobot2", "model": "", "task_id": "", "seq": 3194, "mode": {"mode": 1, "mode_request_id": 0}, "battery_percent": 100.0, "location": {"t": {"sec": 1600, "nanosec": 189000000}, "x": 15.15751838684082, "y": -11.22861385345459, "yaw": -1.5839799642562866, "level_name": "L1", "index": 0}, "path": []}]}\n', 'stream': 'stdout'}


class FleetState(models.Model):
    id = fields.IntField(pk=True)
    robots = fields.JSONField()
    payload = fields.JSONField()
    name = fields.CharField(max_length=200)
    created = fields.DatetimeField(auto_now_add=True)

    service = RobotStateService()
