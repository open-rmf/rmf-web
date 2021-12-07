import json

from rmf_building_map_msgs.msg import Door as RmfDoor
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_fleet_msgs.msg import RobotMode as RmfRobotMode
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftState as RmfLiftState

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
    TaskState,
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


def make_task_state(task_id: str = "test_task") -> TaskState:
    # from https://raw.githubusercontent.com/open-rmf/rmf_api_msgs/960b286d9849fc716a3043b8e1f5fb341bdf5778/rmf_api_msgs/samples/task_state/multi_dropoff_delivery.json
    sample_task = json.loads(
        """
{
  "booking": {
    "id": "delivery_2021:11:08:23:50",
    "unix_millis_earliest_start_time": 1636388400000,
    "priority": "none",
    "automatic": false
  },
  "category": "Multi-Delivery",
  "detail": [
    {
      "category": "Pick Up",
      "params": {
        "location": "Kitchen",
        "items": [
          {
            "type": "soda",
            "quantity": 1
          },
          {
            "type": "water",
            "quantity": 1
          }
        ]
      }
    },
    {
      "category": "Drop Off",
      "params": {
        "location": "room_203",
        "items": [
          {
            "type": "soda",
            "quantity": 1
          }
        ]
      }
    },
    {
      "category": "Drop Off",
      "params": {
        "location": "room_521",
        "items": [
          {
            "type": "water",
            "quantity": 1
          }
        ]
      }
    }
  ],
  "unix_millis_start_time": 1636388410000,
  "estimate_millis": 2000000,
  "phases": {
    "1": {
      "id": 1,
      "category": "Pick Up",
      "detail": {
        "location": "Kitchen",
        "items": [
          {
            "type": "soda",
            "quantity": 1
          },
          {
            "type": "water",
            "quantity": 1
          }
        ]
      },
      "estimate_millis": 600000,
      "final_event_id": 0,
      "events": {
        "0": {
          "id": 0,
          "status": "completed",
          "name": "Pick Up Sequence",
          "detail": "",
          "deps": [1, 2]
        },
        "1": {
          "id": 1,
          "status": "completed",
          "name": "Go to [place:kitchen]",
          "detail": "",
          "deps": [3, 4, 8]
        },
        "2": {
          "id": 2,
          "status": "completed",
          "name": "Receive items",
          "detail": [
            {
              "type": "soda",
              "quantity": 1
            },
            {
              "type": "water",
              "quantity": 1
            }
          ],
          "deps": []
        },
        "3": {
          "id": 3,
          "status": "completed",
          "name": "Move to [place:kitchen_door_exterior]",
          "detail": "",
          "deps": []
        },
        "4": {
          "id": 4,
          "status": "completed",
          "name": "Pass through [door:kitchen_door]",
          "detail": "",
          "deps": [5, 6, 7]
        },
        "5": {
          "id": 5,
          "status": "completed",
          "name": "Wait for [door:kitchen_door] to open",
          "detail": "",
          "deps": []
        },
        "6": {
          "id": 6,
          "status": "completed",
          "name": "Move to [place:kitchen_door_interior]",
          "detail": "",
          "deps": []
        },
        "7": {
          "id": 7,
          "status": "completed",
          "name": "Wait for [door:kitchen_door] to close",
          "detail": "",
          "deps": []
        },
        "8": {
          "id": 8,
          "status": "completed",
          "name": "Move to [place:kitchen]",
          "detail": "",
          "deps": []
        }
      }
    },
    "2": {
      "id": 2,
      "category": "Drop Off",
      "detail": {
        "location": "room_203",
        "items": [
          {
            "type": "soda",
            "quantity": 1
          }
        ]
      },
      "estimate_millis": 720000,
      "final_event_id": 0,
      "events": {
        "0": {
          "id": 0,
          "status": "underway",
          "name": "Drop Off Sequence",
          "detail": "",
          "deps": [1, 2]
        },
        "1": {
          "id": 1,
          "status": "underway",
          "name": "Go to [place:room_203]",
          "detail": "",
          "deps": [3, 4, 8, 9, 14]
        },
        "2": {
          "id": 2,
          "status": "standby",
          "name": "Unload items",
          "detail": [
            {
              "type": "soda",
              "quantity": 1
            }
          ],
          "deps": []
        },
        "3": {
          "id": 3,
          "status": "completed",
          "name": "Move to [place:kitchen_door_interior]",
          "detail": "",
          "deps": []
        },
        "4": {
          "id": 4,
          "status": "underway",
          "name": "Pass through [door:kitchen_door]",
          "detail": "",
          "deps": [5, 6, 7]
        },
        "5": {
          "id": 5,
          "status": "underway",
          "name": "Wait for [door:kitchen_door] to open",
          "detail": "",
          "deps": []
        },
        "6": {
          "id": 6,
          "status": "standby",
          "name": "Move to [place:kitchen_door_exterior]",
          "detail": "",
          "deps": []
        },
        "7": {
          "id": 7,
          "status": "standby",
          "name": "Wait for [door:kitchen_door] to close",
          "detail": "",
          "deps": []
        },
        "8": {
          "id": 8,
          "status": "standby",
          "name": "Move to [place:lift_lobby_05_floor_B1]",
          "detail": "",
          "deps": []
        },
        "9": {
          "id": 9,
          "status": "standby",
          "name": "Take [lift:lift_05_03] to [place:lift_lobby_05_floor_L2]",
          "detail": "",
          "deps": [10, 11, 12, 13]
        },
        "10": {
          "id": 10,
          "status": "underway",
          "name": "Wait for lift",
          "detail": "Currently assigned [lift:lift_05_03]",
          "deps": []
        },
        "11": {
          "id": 11,
          "status": "standby",
          "name": "Move to [place:lift_05_03_floor_B1]",
          "detail": "",
          "deps": []
        },
        "12": {
          "id": 12,
          "status": "standby",
          "name": "Lift [lift:lift_05_03] to [place:lift_05_03_floor_2]",
          "detail": "",
          "deps": []
        },
        "13": {
          "id": 13,
          "status": "standby",
          "name": "Wait for [lift:lift_05_03] to open",
          "detail": "",
          "deps": []
        },
        "14": {
          "id": 14,
          "status": "standby",
          "name": "Move to [place:room_203]",
          "detail": "",
          "deps": []
        }
      }
    },
    "3": {
      "id": 3,
      "category": "Drop Off",
      "detail": {
        "location": "room_521",
        "items": [
          {
            "type": "water",
            "quantity": 1
          }
        ]
      },
      "estimate_millis": 680000
    }
  },
  "completed": [ 1 ],
  "active": 2,
  "pending": [ 3 ]
}
        """
    )
    sample_task["booking"]["id"] = task_id
    return TaskState(**sample_task)
