import json
from uuid import uuid4

from rmf_building_map_msgs.msg import Door as RmfDoor
from rmf_dispenser_msgs.msg import DispenserState as RmfDispenserState
from rmf_door_msgs.msg import DoorMode as RmfDoorMode
from rmf_ingestor_msgs.msg import IngestorState as RmfIngestorState
from rmf_lift_msgs.msg import LiftState as RmfLiftState

from api_server.models import (
    AffineImage,
    BuildingMap,
    DispenserState,
    Door,
    DoorMode,
    DoorState,
    FleetLog,
    FleetState,
    IngestorState,
    Level,
    Lift,
    LiftState,
    TaskEventLog,
    TaskState,
)
from api_server.models import tortoise_models as ttm
from api_server.models.ros_pydantic.builtin_interfaces import Time
from api_server.models.ros_pydantic.rmf_building_map_msgs import Graph


def make_door(name: str = "test_door") -> Door:
    return Door(
        name=name,
        v1_x=0,
        v1_y=0,
        v2_x=0,
        v2_y=0,
        door_type=RmfDoor.DOOR_TYPE_SINGLE_SLIDING,
        motion_range=0,
        motion_direction=1,
    )


def make_lift(name: str = "test_lift") -> Lift:
    return Lift(
        name=name,
        levels=["L1", "L2"],
        doors=[
            make_door(),
        ],
        wall_graph=Graph(name="", vertices=[], edges=[], params=[]),
        ref_x=0,
        ref_y=0,
        ref_yaw=0,
        width=0,
        depth=0,
    )


def make_building_map():
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
                        scale=0.0,
                        encoding="png",
                        data="http://localhost/test_image.png",
                    )
                ],
                places=[],
                doors=[make_door()],
                nav_graphs=[],
                wall_graph=Graph(name="", vertices=[], edges=[], params=[]),
            ),
        ],
        lifts=[make_lift()],
    )


def make_door_state(name: str, mode: int = RmfDoorMode.MODE_CLOSED) -> DoorState:
    return DoorState(
        door_time=Time(sec=0, nanosec=0),
        door_name=name,
        current_mode=DoorMode(value=mode),
    )


def make_lift_state(name: str = "test_lift") -> LiftState:
    return LiftState(
        lift_time=Time(sec=0, nanosec=0),
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
        time=Time(sec=0, nanosec=0),
        guid=guid,
        mode=RmfDispenserState.IDLE,
        request_guid_queue=[],
        seconds_remaining=0.0,
    )


def make_ingestor_state(guid: str = "test_ingestor") -> IngestorState:
    return IngestorState(
        time=Time(sec=0, nanosec=0),
        guid=guid,
        mode=RmfIngestorState.IDLE,
        request_guid_queue=[],
        seconds_remaining=0.0,
    )


def make_fleet_state() -> FleetState:
    return FleetState(name=str(uuid4()), robots={})


def make_fleet_log() -> FleetLog:
    return FleetLog(name=str(uuid4()), log=[], robots={})


def make_task_state(
    task_id: str = "test_task", labels: list[str] | None = None
) -> TaskState:
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
    sample_task["booking"]["labels"] = labels or []
    return TaskState(**sample_task)


def make_task_favorite(
    favorite_task_id: str = "default_id",
) -> ttm.TaskFavoritePydantic:
    sample_favorite_task = json.loads(
        """
   {
      "id":"56248943-16e7-459c-a9cc-ad242ff8646d",
      "name":"Delivery task",
      "unix_millis_earliest_start_time":1636388410000,
      "priority":{
         "type":"binary",
         "value":0
      },
      "category":"delivery",
      "description":{
         "pickup":{
            "place":"",
            "handler":"",
            "payload":""
         },
         "dropoff":{
            "place":"",
            "handler":"",
            "payload":""
         }
      },
      "user":"stub"
   }
    """
    )
    sample_favorite_task["id"] = favorite_task_id
    return ttm.TaskFavoritePydantic(**sample_favorite_task)


def make_task_log(task_id: str) -> TaskEventLog:
    sample = TaskEventLog(
        **json.loads(
            """
{
  "task_id": "delivery_2021:11:08:23:50",
  "log": [
    {
      "seq": 0,
      "tier": "info",
      "unix_millis_time": 1636388410000,
      "text": "Beginning task"
    }
  ],
  "phases": {
    "1": {
      "log": [
        {
          "seq": 0,
          "tier": "info",
          "unix_millis_time": 1636388410000,
          "text": "Beginning phase"
        }
      ],
      "events": {
        "1": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388409995,
            "text": "Generating plan to get from [place:parking_03] to [place:kitchen]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388409996,
            "text": "Finished generating plan to get from [place:parking_03] to [place:kitchen]"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388414500,
            "text": "Finished: Move to [place:kitchen_door_exterior]"
          },
          {
            "seq": 3,
            "tier": "info",
            "unix_millis_time": 1636388415000,
            "text": "Finished: Wait for [door:kitchen_door] to open"
          },
          {
            "seq": 4,
            "tier": "info",
            "unix_millis_time": 1636388418001,
            "text": "Finished: Move to [place:kitchen_door_interior]"
          },
          {
            "seq": 5,
            "tier": "info",
            "unix_millis_time": 1636388419111,
            "text": "Finished: Wait for [door:kitchen_door] to close"
          },
          {
            "seq": 6,
            "tier": "info",
            "unix_millis_time": 1636388421121,
            "text": "Finished: Move to [place:kitchen]"
          }
        ],
        "2": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388421421,
            "text": "Requested [item:soda], [item:water]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388421521,
            "text": "Request acknowledged"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388430000,
            "text": "Received [item:soda]"
          },
          {
            "seq": 3,
            "tier": "info",
            "unix_millis_time": 1636388440000,
            "text": "Received [item:water]"
          }
        ],
        "3": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388410000,
            "text": "Moving towards [place:kitchen_door_exterior] from [place:parking_03]"
          },
          {
            "seq": 1,
            "tier": "warning",
            "unix_millis_time": 1636388411000,
            "text": "Delayed by obstacle blocking [robot:deliverbot_01]"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388414500,
            "text": "Arrived at [place:kitchen_door_exterior]"
          }
        ],
        "5": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388414500,
            "text": "Requested [door:kitchen_door] to open"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388414600,
            "text": "[door:kitchen_door] acknowledged request"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388415000,
            "text": "[door:kitchen_door] has opened"
          }
        ],
        "6": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388415010,
            "text": "Moving towards [place:kitchen_door_interior] from [place:kitchen_door_exterior]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388418000,
            "text": "Arrived at [place:kitchen_door_interior]"
          }
        ],
        "7": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388418010,
            "text": "Requested [door:kitchen_door] to close"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388418110,
            "text": "[door:kitchen_door] acknowledged request"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388419110,
            "text": "[door:kitchen] has closed"
          }
        ],
        "8": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388419120,
            "text": "Moving towards [place:kitchen] from [place:kitchen_door_interior]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388421120,
            "text": "Arrived at [place:kitchen]"
          }
        ]
      }
    },
    "2": {
      "log": [
        {
          "seq": 0,
          "tier": "info",
          "unix_millis_time": 1636388444500,
          "text": "Beginning phase"
        }
      ],
      "events": {
        "1": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388440000,
            "text": "Generating plan to get from [place:kitchen] to [place:room_203]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388440010,
            "text": "Finished generating plan to get from [place:kitchen_03] to [place:room_203]"
          },
          {
            "seq": 2,
            "tier": "info",
            "unix_millis_time": 1636388450000,
            "text": "Finished: Move to [place:kitchen_door_interior]"
          }
        ],
        "3": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388440010,
            "text": "Moving towards [place:kitchen_door_interior] from [place:kitchen]"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388450000,
            "text": "Arrived at [place:kitchen_door_interior]"
          }
        ],
        "5": [
          {
            "seq": 0,
            "tier": "info",
            "unix_millis_time": 1636388450010,
            "text": "Requested [door:kitchen_door] to open"
          },
          {
            "seq": 1,
            "tier": "info",
            "unix_millis_time": 1636388450110,
            "text": "[door:kitchen_door] acknowledged request"
          }
        ]
      }
    }
  }
}
  """
        )
    )
    sample.task_id = task_id
    return sample
