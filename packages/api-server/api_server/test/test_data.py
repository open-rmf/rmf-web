from rmf_building_map_msgs.msg import Door as RmfDoor
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_fleet_msgs.msg import RobotMode as RmfRobotMode
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftState as RmfLiftState
from rmf_task_msgs.msg import TaskSummary as RmfTaskSummary

from api_server.models import (
    AffineImage,
    BuildingMap,
    DispenserState,
    Door,
    DoorMode,
    DoorState,
    FleetState,
    IngestorState,
    Level,
    Lift,
    LiftState,
    RobotMode,
    RobotState,
    TaskSummary,
)


def make_door(name: str = "test_door") -> Door:
    return Door(
        name=name,
        door_type=RmfDoor.DOOR_TYPE_SINGLE_SLIDING,
        motion_direction=1,
    )


def make_lift(name: str = "test_lift") -> Lift:
    return Lift(
        name=name,
        levels=["L1", "L2"],
        doors=[
            make_door(),
        ],
    )


def make_building_map():
    return BuildingMap(
        name="test_name",
        levels=[
            Level(
                name="L1",
                elevation=0.0,
                doors=[make_door()],
                images=[
                    AffineImage(
                        name="test_image",
                        encoding="png",
                        data="http://localhost/test_image.png",
                    )
                ],
            ),
        ],
        lifts=[make_lift()],
    )


def make_door_state(name: str, mode: int = RmfDoorMode.MODE_CLOSED) -> DoorState:
    return DoorState(
        door_name=name,
        current_mode=DoorMode(value=mode),
    )


def make_lift_state(name: str = "test_lift") -> LiftState:
    return LiftState(
        lift_name=name or "test_lift",
        available_floors=["L1", "L2"],
        current_floor="L1",
        destination_floor="L1",
        door_state=RmfLiftState.DOOR_CLOSED,
        motion_state=RmfLiftState.MOTION_STOPPED,
        available_modes=[RmfLiftState.MODE_AGV],
        current_mode=RmfLiftState.MODE_AGV,
        session_id="test_session",
    )


def make_dispenser_state(guid: str = "test_dispenser") -> DispenserState:
    return DispenserState(
        guid=guid,
        mode=RmfDispenserState.IDLE,
    )


def make_ingestor_state(guid: str = "test_ingestor") -> IngestorState:
    return IngestorState(
        guid=guid,
        mode=RmfIngestorState.IDLE,
    )


def make_robot_state(name: str = "test_robot") -> RobotState:
    return RobotState(
        name=name,
        model="test_model",
        task_id="",
        seq=0,
        mode=RobotMode(mode=RmfRobotMode.MODE_IDLE),
        battery_percent=0.0,
        path=[],
    )


def make_fleet_state(name: str = "test_fleet") -> FleetState:
    return FleetState(
        name=name,
        robots=[make_robot_state()],
    )


def make_task_summary(task_id: str = "test_task") -> TaskSummary:
    return TaskSummary(
        task_id=task_id,
        state=RmfTaskSummary.STATE_ACTIVE,
    )
