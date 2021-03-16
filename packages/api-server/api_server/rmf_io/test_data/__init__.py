import os.path

from building_map_msgs.msg import AffineImage, BuildingMap, Door, Graph, Level, Lift
from builtin_interfaces.msg import Time
from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorMode, DoorState
from rmf_fleet_msgs.msg import FleetState, Location, RobotMode, RobotState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState


def make_door(name: str = "test_door") -> Door:
    return Door(
        name=name,
        v1_x=0.0,
        v1_y=0.0,
        v2_x=0.0,
        v2_y=0.0,
        door_type=Door.DOOR_TYPE_SINGLE_SLIDING,
        motion_range=0.0,
        motion_direction=1,
    )


def make_lift(name: str = "test_lift") -> Lift:
    return Lift(
        name=name,
        levels=["L1", "L2"],
        doors=[
            make_door(),
        ],
        wall_graph=Graph(
            name="test_graph",
            vertices=[],
            edges=[],
            params=[],
        ),
        ref_x=0.0,
        ref_y=0.0,
        ref_yaw=0.0,
        width=0.0,
        depth=0.0,
    )


def make_building_map():
    with open(f"{os.path.dirname(__file__)}/office.png", "br") as f:
        image_data = f.read()

    return BuildingMap(
        name="test_name",
        levels=[
            Level(
                name="L1",
                elevation=0.0,
                images=[
                    AffineImage(
                        name="test_image",
                        x_offset=0.0,
                        y_offset=0.0,
                        yaw=0.0,
                        scale=1.0,
                        encoding="png",
                        data=image_data,
                    )
                ],
                places=[],
                doors=[],
                nav_graphs=[],
                wall_graph=Graph(
                    name="test_graph",
                    vertices=[],
                    edges=[],
                    params=[],
                ),
            ),
        ],
        lifts=[],
    )


def make_door_state(name: str, mode: int = DoorMode.MODE_CLOSED) -> DoorState:
    return DoorState(
        door_name=name,
        current_mode=DoorMode(value=mode),
        door_time=Time(sec=0, nanosec=0),
    )


def make_lift_state(name: str = "test_lift") -> LiftState:
    return LiftState(
        lift_name=name or "test_lift",
        lift_time=Time(sec=0, nanosec=0),
        available_floors=["L1", "L2"],
        current_floor="L1",
        destination_floor="L1",
        door_state=LiftState.DOOR_CLOSED,
        motion_state=LiftState.MOTION_STOPPED,
        available_modes=[LiftState.MODE_AGV],
        current_mode=LiftState.MODE_AGV,
        session_id="test_session",
    )


def make_dispenser_state(guid: str = "test_dispenser") -> DispenserState:
    return DispenserState(
        guid=guid,
        time=Time(sec=0, nanosec=0),
        mode=DispenserState.IDLE,
        request_guid_queue=[],
        seconds_remaining=0.0,
    )


def make_ingestor_state(guid: str = "test_ingestor") -> IngestorState:
    return IngestorState(
        guid=guid,
        time=Time(sec=0, nanosec=0),
        mode=IngestorState.IDLE,
        request_guid_queue=[],
        seconds_remaining=0.0,
    )


def make_robot_state(name: str = "test_robot") -> RobotState:
    return RobotState(
        name=name,
        model="test_model",
        task_id="",
        seq=0,
        mode=RobotMode(mode=RobotMode.MODE_IDLE, mode_request_id=0),
        battery_percent=0.0,
        location=Location(
            t=Time(sec=0, nanosec=0),
            x=0.0,
            y=0.0,
            yaw=0.0,
            level_name="test_level",
            index=0,
        ),
        path=[],
    )


def make_fleet_state(name: str = "test_fleet") -> FleetState:
    return FleetState(
        name=name,
        robots=[make_robot_state()],
    )
