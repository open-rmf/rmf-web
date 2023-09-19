export default {
  openapi: '3.0.2',
  info: { title: 'RMF API Server', version: '0.1.0' },
  paths: {
    '/socket.io': {
      get: {
        summary: 'Socket.io endpoint',
        description:
          '\n# NOTE: This endpoint is here for documentation purposes only, this is _not_ a REST endpoint.\n\n## About\nThis exposes a minimal pubsub system built on top of socket.io.\nIt works similar to a normal socket.io endpoint, except that are 2 special\nrooms which control subscriptions.\n\n## Rooms\n### subscribe\nClients must send a message to this room to start receiving messages on other rooms.\nThe message must be of the form:\n\n```\n{\n    "room": "<room_name>"\n}\n```\n\n### unsubscribe\nClients can send a message to this room to stop receiving messages on other rooms.\nThe message must be of the form:\n\n```\n{\n    "room": "<room_name>"\n}\n```\n            \n### /alerts\n\n\n```\n{\n  "title": "Alert",\n  "description": "General alert that can be triggered by events.",\n  "type": "object",\n  "properties": {\n    "id": {\n      "title": "Id",\n      "maxLength": 255,\n      "type": "string"\n    },\n    "original_id": {\n      "title": "Original Id",\n      "maxLength": 255,\n      "type": "string"\n    },\n    "category": {\n      "title": "Category",\n      "description": "Default: default<br/>Task: task<br/>Fleet: fleet<br/>Robot: robot",\n      "maxLength": 7,\n      "type": "string"\n    },\n    "unix_millis_created_time": {\n      "title": "Unix Millis Created Time",\n      "minimum": -9223372036854775808,\n      "maximum": 9223372036854775807,\n      "type": "integer"\n    },\n    "acknowledged_by": {\n      "title": "Acknowledged By",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "unix_millis_acknowledged_time": {\n      "title": "Unix Millis Acknowledged Time",\n      "minimum": -9223372036854775808,\n      "maximum": 9223372036854775807,\n      "nullable": true,\n      "type": "integer"\n    }\n  },\n  "required": [\n    "id",\n    "original_id",\n    "category",\n    "unix_millis_created_time"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /beacons\n\n\n```\n{\n  "title": "BeaconState",\n  "type": "object",\n  "properties": {\n    "id": {\n      "title": "Id",\n      "maxLength": 255,\n      "type": "string"\n    },\n    "online": {\n      "title": "Online",\n      "type": "boolean"\n    },\n    "category": {\n      "title": "Category",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "activated": {\n      "title": "Activated",\n      "type": "boolean"\n    },\n    "level": {\n      "title": "Level",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    }\n  },\n  "required": [\n    "id",\n    "online",\n    "activated"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /building_map\n\n\n```\n{\n  "title": "BuildingMap",\n  "type": "object",\n  "properties": {\n    "name": {\n      "title": "Name",\n      "default": "",\n      "type": "string"\n    },\n    "levels": {\n      "title": "Levels",\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/Level"\n      }\n    },\n    "lifts": {\n      "title": "Lifts",\n      "default": [],\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/Lift"\n      }\n    }\n  },\n  "required": [\n    "name",\n    "levels",\n    "lifts"\n  ],\n  "definitions": {\n    "AffineImage": {\n      "title": "AffineImage",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "x_offset": {\n          "title": "X Offset",\n          "default": 0,\n          "type": "number"\n        },\n        "y_offset": {\n          "title": "Y Offset",\n          "default": 0,\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "default": 0,\n          "type": "number"\n        },\n        "scale": {\n          "title": "Scale",\n          "default": 0,\n          "type": "number"\n        },\n        "encoding": {\n          "title": "Encoding",\n          "default": "",\n          "type": "string"\n        },\n        "data": {\n          "title": "Data",\n          "type": "string"\n        }\n      },\n      "required": [\n        "name",\n        "x_offset",\n        "y_offset",\n        "yaw",\n        "scale",\n        "encoding",\n        "data"\n      ]\n    },\n    "Place": {\n      "title": "Place",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "x": {\n          "title": "X",\n          "default": 0,\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "default": 0,\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "default": 0,\n          "type": "number"\n        },\n        "position_tolerance": {\n          "title": "Position Tolerance",\n          "default": 0,\n          "type": "number"\n        },\n        "yaw_tolerance": {\n          "title": "Yaw Tolerance",\n          "default": 0,\n          "type": "number"\n        }\n      },\n      "required": [\n        "name",\n        "x",\n        "y",\n        "yaw",\n        "position_tolerance",\n        "yaw_tolerance"\n      ]\n    },\n    "Door": {\n      "title": "Door",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "v1_x": {\n          "title": "V1 X",\n          "default": 0,\n          "type": "number"\n        },\n        "v1_y": {\n          "title": "V1 Y",\n          "default": 0,\n          "type": "number"\n        },\n        "v2_x": {\n          "title": "V2 X",\n          "default": 0,\n          "type": "number"\n        },\n        "v2_y": {\n          "title": "V2 Y",\n          "default": 0,\n          "type": "number"\n        },\n        "door_type": {\n          "title": "Door Type",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 255,\n          "type": "integer"\n        },\n        "motion_range": {\n          "title": "Motion Range",\n          "default": 0,\n          "type": "number"\n        },\n        "motion_direction": {\n          "title": "Motion Direction",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "name",\n        "v1_x",\n        "v1_y",\n        "v2_x",\n        "v2_y",\n        "door_type",\n        "motion_range",\n        "motion_direction"\n      ]\n    },\n    "Param": {\n      "title": "Param",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "type": {\n          "title": "Type",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        },\n        "value_int": {\n          "title": "Value Int",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        },\n        "value_float": {\n          "title": "Value Float",\n          "default": 0,\n          "type": "number"\n        },\n        "value_string": {\n          "title": "Value String",\n          "default": "",\n          "type": "string"\n        },\n        "value_bool": {\n          "title": "Value Bool",\n          "default": false,\n          "type": "boolean"\n        }\n      },\n      "required": [\n        "name",\n        "type",\n        "value_int",\n        "value_float",\n        "value_string",\n        "value_bool"\n      ]\n    },\n    "GraphNode": {\n      "title": "GraphNode",\n      "type": "object",\n      "properties": {\n        "x": {\n          "title": "X",\n          "default": 0,\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "default": 0,\n          "type": "number"\n        },\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "params": {\n          "title": "Params",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Param"\n          }\n        }\n      },\n      "required": [\n        "x",\n        "y",\n        "name",\n        "params"\n      ]\n    },\n    "GraphEdge": {\n      "title": "GraphEdge",\n      "type": "object",\n      "properties": {\n        "v1_idx": {\n          "title": "V1 Idx",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        },\n        "v2_idx": {\n          "title": "V2 Idx",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        },\n        "params": {\n          "title": "Params",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Param"\n          }\n        },\n        "edge_type": {\n          "title": "Edge Type",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 255,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "v1_idx",\n        "v2_idx",\n        "params",\n        "edge_type"\n      ]\n    },\n    "Graph": {\n      "title": "Graph",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "vertices": {\n          "title": "Vertices",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/GraphNode"\n          }\n        },\n        "edges": {\n          "title": "Edges",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/GraphEdge"\n          }\n        },\n        "params": {\n          "title": "Params",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Param"\n          }\n        }\n      },\n      "required": [\n        "name",\n        "vertices",\n        "edges",\n        "params"\n      ]\n    },\n    "Level": {\n      "title": "Level",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "elevation": {\n          "title": "Elevation",\n          "default": 0,\n          "type": "number"\n        },\n        "images": {\n          "title": "Images",\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/AffineImage"\n          }\n        },\n        "places": {\n          "title": "Places",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Place"\n          }\n        },\n        "doors": {\n          "title": "Doors",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Door"\n          }\n        },\n        "nav_graphs": {\n          "title": "Nav Graphs",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Graph"\n          }\n        },\n        "wall_graph": {\n          "title": "Wall Graph",\n          "default": {\n            "name": "",\n            "vertices": [],\n            "edges": [],\n            "params": []\n          },\n          "allOf": [\n            {\n              "$ref": "#/definitions/Graph"\n            }\n          ]\n        }\n      },\n      "required": [\n        "name",\n        "elevation",\n        "images",\n        "places",\n        "doors",\n        "nav_graphs",\n        "wall_graph"\n      ]\n    },\n    "Lift": {\n      "title": "Lift",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "default": "",\n          "type": "string"\n        },\n        "levels": {\n          "title": "Levels",\n          "default": [],\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        },\n        "doors": {\n          "title": "Doors",\n          "default": [],\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Door"\n          }\n        },\n        "wall_graph": {\n          "title": "Wall Graph",\n          "default": {\n            "name": "",\n            "vertices": [],\n            "edges": [],\n            "params": []\n          },\n          "allOf": [\n            {\n              "$ref": "#/definitions/Graph"\n            }\n          ]\n        },\n        "ref_x": {\n          "title": "Ref X",\n          "default": 0,\n          "type": "number"\n        },\n        "ref_y": {\n          "title": "Ref Y",\n          "default": 0,\n          "type": "number"\n        },\n        "ref_yaw": {\n          "title": "Ref Yaw",\n          "default": 0,\n          "type": "number"\n        },\n        "width": {\n          "title": "Width",\n          "default": 0,\n          "type": "number"\n        },\n        "depth": {\n          "title": "Depth",\n          "default": 0,\n          "type": "number"\n        }\n      },\n      "required": [\n        "name",\n        "levels",\n        "doors",\n        "wall_graph",\n        "ref_x",\n        "ref_y",\n        "ref_yaw",\n        "width",\n        "depth"\n      ]\n    }\n  }\n}\n```\n\n\n### /doors/{door_name}/state\n\n\n```\n{\n  "title": "DoorState",\n  "type": "object",\n  "properties": {\n    "door_time": {\n      "title": "Door Time",\n      "default": {\n        "sec": 0,\n        "nanosec": 0\n      },\n      "allOf": [\n        {\n          "$ref": "#/definitions/Time"\n        }\n      ]\n    },\n    "door_name": {\n      "title": "Door Name",\n      "default": "",\n      "type": "string"\n    },\n    "current_mode": {\n      "title": "Current Mode",\n      "default": {\n        "value": 0\n      },\n      "allOf": [\n        {\n          "$ref": "#/definitions/DoorMode"\n        }\n      ]\n    }\n  },\n  "required": [\n    "door_time",\n    "door_name",\n    "current_mode"\n  ],\n  "definitions": {\n    "Time": {\n      "title": "Time",\n      "type": "object",\n      "properties": {\n        "sec": {\n          "title": "Sec",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        },\n        "nanosec": {\n          "title": "Nanosec",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ]\n    },\n    "DoorMode": {\n      "title": "DoorMode",\n      "type": "object",\n      "properties": {\n        "value": {\n          "title": "Value",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "value"\n      ]\n    }\n  }\n}\n```\n\n\n### /doors/{door_name}/health\n\n\n```\n{\n  "title": "DoorHealth",\n  "type": "object",\n  "properties": {\n    "health_status": {\n      "title": "Health Status",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "health_message": {\n      "title": "Health Message",\n      "nullable": true,\n      "type": "string"\n    },\n    "id_": {\n      "title": "Id ",\n      "maxLength": 255,\n      "type": "string"\n    }\n  },\n  "required": [\n    "health_status",\n    "id_"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /lifts/{lift_name}/state\n\n\n```\n{\n  "title": "LiftState",\n  "type": "object",\n  "properties": {\n    "lift_time": {\n      "title": "Lift Time",\n      "default": {\n        "sec": 0,\n        "nanosec": 0\n      },\n      "allOf": [\n        {\n          "$ref": "#/definitions/Time"\n        }\n      ]\n    },\n    "lift_name": {\n      "title": "Lift Name",\n      "default": "",\n      "type": "string"\n    },\n    "available_floors": {\n      "title": "Available Floors",\n      "default": [],\n      "type": "array",\n      "items": {\n        "type": "string"\n      }\n    },\n    "current_floor": {\n      "title": "Current Floor",\n      "default": "",\n      "type": "string"\n    },\n    "destination_floor": {\n      "title": "Destination Floor",\n      "default": "",\n      "type": "string"\n    },\n    "door_state": {\n      "title": "Door State",\n      "default": 0,\n      "minimum": 0,\n      "maximum": 255,\n      "type": "integer"\n    },\n    "motion_state": {\n      "title": "Motion State",\n      "default": 0,\n      "minimum": 0,\n      "maximum": 255,\n      "type": "integer"\n    },\n    "available_modes": {\n      "title": "Available Modes",\n      "type": "array",\n      "items": {\n        "type": "integer"\n      }\n    },\n    "current_mode": {\n      "title": "Current Mode",\n      "default": 0,\n      "minimum": 0,\n      "maximum": 255,\n      "type": "integer"\n    },\n    "session_id": {\n      "title": "Session Id",\n      "default": "",\n      "type": "string"\n    }\n  },\n  "required": [\n    "lift_time",\n    "lift_name",\n    "available_floors",\n    "current_floor",\n    "destination_floor",\n    "door_state",\n    "motion_state",\n    "available_modes",\n    "current_mode",\n    "session_id"\n  ],\n  "definitions": {\n    "Time": {\n      "title": "Time",\n      "type": "object",\n      "properties": {\n        "sec": {\n          "title": "Sec",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        },\n        "nanosec": {\n          "title": "Nanosec",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ]\n    }\n  }\n}\n```\n\n\n### /lifts/{lift_name}/health\n\n\n```\n{\n  "title": "LiftHealth",\n  "type": "object",\n  "properties": {\n    "health_status": {\n      "title": "Health Status",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "health_message": {\n      "title": "Health Message",\n      "nullable": true,\n      "type": "string"\n    },\n    "id_": {\n      "title": "Id ",\n      "maxLength": 255,\n      "type": "string"\n    }\n  },\n  "required": [\n    "health_status",\n    "id_"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /tasks/{task_id}/state\n\n\n```\n{\n  "title": "TaskState",\n  "type": "object",\n  "properties": {\n    "booking": {\n      "$ref": "#/definitions/Booking"\n    },\n    "category": {\n      "$ref": "#/definitions/Category"\n    },\n    "detail": {\n      "$ref": "#/definitions/Detail"\n    },\n    "unix_millis_start_time": {\n      "title": "Unix Millis Start Time",\n      "type": "integer"\n    },\n    "unix_millis_finish_time": {\n      "title": "Unix Millis Finish Time",\n      "type": "integer"\n    },\n    "original_estimate_millis": {\n      "$ref": "#/definitions/EstimateMillis"\n    },\n    "estimate_millis": {\n      "$ref": "#/definitions/EstimateMillis"\n    },\n    "assigned_to": {\n      "title": "Assigned To",\n      "description": "Which agent (robot) is the task assigned to",\n      "allOf": [\n        {\n          "$ref": "#/definitions/AssignedTo"\n        }\n      ]\n    },\n    "status": {\n      "$ref": "#/definitions/Status"\n    },\n    "dispatch": {\n      "$ref": "#/definitions/Dispatch"\n    },\n    "phases": {\n      "title": "Phases",\n      "description": "A dictionary of the states of the phases of the task. The keys (property names) are phase IDs, which are integers.",\n      "type": "object",\n      "additionalProperties": {\n        "$ref": "#/definitions/Phase"\n      }\n    },\n    "completed": {\n      "title": "Completed",\n      "description": "An array of the IDs of completed phases of this task",\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/Id"\n      }\n    },\n    "active": {\n      "title": "Active",\n      "description": "The ID of the active phase for this task",\n      "allOf": [\n        {\n          "$ref": "#/definitions/Id"\n        }\n      ]\n    },\n    "pending": {\n      "title": "Pending",\n      "description": "An array of the pending phases of this task",\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/Id"\n      }\n    },\n    "interruptions": {\n      "title": "Interruptions",\n      "description": "A dictionary of interruptions that have been applied to this task. The keys (property names) are the unique token of the interruption request.",\n      "type": "object",\n      "additionalProperties": {\n        "$ref": "#/definitions/Interruption"\n      }\n    },\n    "cancellation": {\n      "title": "Cancellation",\n      "description": "If the task was cancelled, this will describe information about the request.",\n      "allOf": [\n        {\n          "$ref": "#/definitions/Cancellation"\n        }\n      ]\n    },\n    "killed": {\n      "title": "Killed",\n      "description": "If the task was killed, this will describe information about the request.",\n      "allOf": [\n        {\n          "$ref": "#/definitions/Killed"\n        }\n      ]\n    }\n  },\n  "required": [\n    "booking"\n  ],\n  "definitions": {\n    "Booking": {\n      "title": "Booking",\n      "type": "object",\n      "properties": {\n        "id": {\n          "title": "Id",\n          "description": "The unique identifier for this task",\n          "type": "string"\n        },\n        "unix_millis_earliest_start_time": {\n          "title": "Unix Millis Earliest Start Time",\n          "type": "integer"\n        },\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "priority": {\n          "title": "Priority",\n          "description": "Priority information about this task",\n          "anyOf": [\n            {\n              "type": "object"\n            },\n            {\n              "type": "string"\n            }\n          ]\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Information about how and why this task was booked",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        },\n        "requester": {\n          "title": "Requester",\n          "description": "(Optional) An identifier for the entity that requested this task",\n          "type": "string"\n        }\n      },\n      "required": [\n        "id"\n      ]\n    },\n    "Category": {\n      "title": "Category",\n      "description": "The category of this task or phase",\n      "type": "string"\n    },\n    "Detail": {\n      "title": "Detail",\n      "description": "Detailed information about a task, phase, or event",\n      "anyOf": [\n        {\n          "type": "object"\n        },\n        {\n          "type": "array",\n          "items": {}\n        },\n        {\n          "type": "string"\n        }\n      ]\n    },\n    "EstimateMillis": {\n      "title": "EstimateMillis",\n      "description": "An estimate, in milliseconds, of how long the subject will take to complete",\n      "minimum": 0,\n      "type": "integer"\n    },\n    "AssignedTo": {\n      "title": "AssignedTo",\n      "type": "object",\n      "properties": {\n        "group": {\n          "title": "Group",\n          "type": "string"\n        },\n        "name": {\n          "title": "Name",\n          "type": "string"\n        }\n      },\n      "required": [\n        "group",\n        "name"\n      ]\n    },\n    "Status": {\n      "title": "Status",\n      "description": "An enumeration.",\n      "enum": [\n        "uninitialized",\n        "blocked",\n        "error",\n        "failed",\n        "queued",\n        "standby",\n        "underway",\n        "delayed",\n        "skipped",\n        "canceled",\n        "killed",\n        "completed"\n      ]\n    },\n    "Status1": {\n      "title": "Status1",\n      "description": "An enumeration.",\n      "enum": [\n        "queued",\n        "selected",\n        "dispatched",\n        "failed_to_assign",\n        "canceled_in_flight"\n      ]\n    },\n    "Assignment": {\n      "title": "Assignment",\n      "type": "object",\n      "properties": {\n        "fleet_name": {\n          "title": "Fleet Name",\n          "type": "string"\n        },\n        "expected_robot_name": {\n          "title": "Expected Robot Name",\n          "type": "string"\n        }\n      }\n    },\n    "Error": {\n      "title": "Error",\n      "type": "object",\n      "properties": {\n        "code": {\n          "title": "Code",\n          "description": "A standard code for the kind of error that has occurred",\n          "minimum": 0,\n          "type": "integer"\n        },\n        "category": {\n          "title": "Category",\n          "description": "The category of the error",\n          "type": "string"\n        },\n        "detail": {\n          "title": "Detail",\n          "description": "Details about the error",\n          "type": "string"\n        }\n      }\n    },\n    "Dispatch": {\n      "title": "Dispatch",\n      "type": "object",\n      "properties": {\n        "status": {\n          "$ref": "#/definitions/Status1"\n        },\n        "assignment": {\n          "$ref": "#/definitions/Assignment"\n        },\n        "errors": {\n          "title": "Errors",\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Error"\n          }\n        }\n      },\n      "required": [\n        "status"\n      ]\n    },\n    "Id": {\n      "title": "Id",\n      "minimum": 0,\n      "type": "integer"\n    },\n    "EventState": {\n      "title": "EventState",\n      "type": "object",\n      "properties": {\n        "id": {\n          "$ref": "#/definitions/Id"\n        },\n        "status": {\n          "$ref": "#/definitions/Status"\n        },\n        "name": {\n          "title": "Name",\n          "description": "The brief name of the event",\n          "type": "string"\n        },\n        "detail": {\n          "title": "Detail",\n          "description": "Detailed information about the event",\n          "allOf": [\n            {\n              "$ref": "#/definitions/Detail"\n            }\n          ]\n        },\n        "deps": {\n          "title": "Deps",\n          "description": "This event may depend on other events. This array contains the IDs of those other event dependencies.",\n          "type": "array",\n          "items": {\n            "type": "integer",\n            "minimum": 0\n          }\n        }\n      },\n      "required": [\n        "id"\n      ]\n    },\n    "Undo": {\n      "title": "Undo",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the undo skip request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the undo skip request",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ]\n    },\n    "SkipPhaseRequest": {\n      "title": "SkipPhaseRequest",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the skip request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the purpose of the skip request",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        },\n        "undo": {\n          "title": "Undo",\n          "description": "Information about an undo skip request that applied to this request",\n          "allOf": [\n            {\n              "$ref": "#/definitions/Undo"\n            }\n          ]\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ]\n    },\n    "Phase": {\n      "title": "Phase",\n      "type": "object",\n      "properties": {\n        "id": {\n          "$ref": "#/definitions/Id"\n        },\n        "category": {\n          "$ref": "#/definitions/Category"\n        },\n        "detail": {\n          "$ref": "#/definitions/Detail"\n        },\n        "unix_millis_start_time": {\n          "title": "Unix Millis Start Time",\n          "type": "integer"\n        },\n        "unix_millis_finish_time": {\n          "title": "Unix Millis Finish Time",\n          "type": "integer"\n        },\n        "original_estimate_millis": {\n          "$ref": "#/definitions/EstimateMillis"\n        },\n        "estimate_millis": {\n          "$ref": "#/definitions/EstimateMillis"\n        },\n        "final_event_id": {\n          "$ref": "#/definitions/Id"\n        },\n        "events": {\n          "title": "Events",\n          "description": "A dictionary of events for this phase. The keys (property names) are the event IDs, which are integers.",\n          "type": "object",\n          "additionalProperties": {\n            "$ref": "#/definitions/EventState"\n          }\n        },\n        "skip_requests": {\n          "title": "Skip Requests",\n          "description": "Information about any skip requests that have been received",\n          "type": "object",\n          "additionalProperties": {\n            "$ref": "#/definitions/SkipPhaseRequest"\n          }\n        }\n      },\n      "required": [\n        "id"\n      ]\n    },\n    "ResumedBy": {\n      "title": "ResumedBy",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the resume request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the resume request",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        }\n      },\n      "required": [\n        "labels"\n      ]\n    },\n    "Interruption": {\n      "title": "Interruption",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the interruption request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the purpose of the interruption",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        },\n        "resumed_by": {\n          "title": "Resumed By",\n          "description": "Information about the resume request that ended this interruption. This field will be missing if the interruption is still active.",\n          "allOf": [\n            {\n              "$ref": "#/definitions/ResumedBy"\n            }\n          ]\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ]\n    },\n    "Cancellation": {\n      "title": "Cancellation",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the cancellation request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the cancel request",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ]\n    },\n    "Killed": {\n      "title": "Killed",\n      "type": "object",\n      "properties": {\n        "unix_millis_request_time": {\n          "title": "Unix Millis Request Time",\n          "description": "The time that the cancellation request arrived",\n          "type": "integer"\n        },\n        "labels": {\n          "title": "Labels",\n          "description": "Labels to describe the kill request",\n          "type": "array",\n          "items": {\n            "type": "string"\n          }\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ]\n    }\n  }\n}\n```\n\n\n### /tasks/{task_id}/log\n\n\n```\n{\n  "title": "TaskEventLog",\n  "type": "object",\n  "properties": {\n    "task_id": {\n      "title": "Task Id",\n      "type": "string"\n    },\n    "log": {\n      "title": "Log",\n      "description": "Log entries related to the overall task",\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/LogEntry"\n      }\n    },\n    "phases": {\n      "title": "Phases",\n      "description": "A dictionary whose keys (property names) are the indices of a phase",\n      "type": "object",\n      "additionalProperties": {\n        "$ref": "#/definitions/Phases"\n      }\n    }\n  },\n  "required": [\n    "task_id"\n  ],\n  "additionalProperties": false,\n  "definitions": {\n    "Tier": {\n      "title": "Tier",\n      "description": "An enumeration.",\n      "enum": [\n        "uninitialized",\n        "info",\n        "warning",\n        "error"\n      ]\n    },\n    "LogEntry": {\n      "title": "LogEntry",\n      "type": "object",\n      "properties": {\n        "seq": {\n          "title": "Seq",\n          "description": "Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.",\n          "exclusiveMaximum": 4294967296,\n          "minimum": 0,\n          "type": "integer"\n        },\n        "tier": {\n          "description": "The importance level of the log entry",\n          "allOf": [\n            {\n              "$ref": "#/definitions/Tier"\n            }\n          ]\n        },\n        "unix_millis_time": {\n          "title": "Unix Millis Time",\n          "type": "integer"\n        },\n        "text": {\n          "title": "Text",\n          "description": "The text of the log entry",\n          "type": "string"\n        }\n      },\n      "required": [\n        "seq",\n        "tier",\n        "unix_millis_time",\n        "text"\n      ]\n    },\n    "Phases": {\n      "title": "Phases",\n      "type": "object",\n      "properties": {\n        "log": {\n          "title": "Log",\n          "description": "Log entries related to the overall phase",\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/LogEntry"\n          }\n        },\n        "events": {\n          "title": "Events",\n          "description": "A dictionary whose keys (property names) are the indices of an event in the phase",\n          "type": "object",\n          "additionalProperties": {\n            "type": "array",\n            "items": {\n              "$ref": "#/definitions/LogEntry"\n            }\n          }\n        }\n      },\n      "additionalProperties": false\n    }\n  }\n}\n```\n\n\n### /dispensers/{guid}/state\n\n\n```\n{\n  "title": "DispenserState",\n  "type": "object",\n  "properties": {\n    "time": {\n      "title": "Time",\n      "default": {\n        "sec": 0,\n        "nanosec": 0\n      },\n      "allOf": [\n        {\n          "$ref": "#/definitions/Time"\n        }\n      ]\n    },\n    "guid": {\n      "title": "Guid",\n      "default": "",\n      "type": "string"\n    },\n    "mode": {\n      "title": "Mode",\n      "default": 0,\n      "minimum": -2147483648,\n      "maximum": 2147483647,\n      "type": "integer"\n    },\n    "request_guid_queue": {\n      "title": "Request Guid Queue",\n      "default": [],\n      "type": "array",\n      "items": {\n        "type": "string"\n      }\n    },\n    "seconds_remaining": {\n      "title": "Seconds Remaining",\n      "default": 0,\n      "type": "number"\n    }\n  },\n  "required": [\n    "time",\n    "guid",\n    "mode",\n    "request_guid_queue",\n    "seconds_remaining"\n  ],\n  "definitions": {\n    "Time": {\n      "title": "Time",\n      "type": "object",\n      "properties": {\n        "sec": {\n          "title": "Sec",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        },\n        "nanosec": {\n          "title": "Nanosec",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ]\n    }\n  }\n}\n```\n\n\n### /dispensers/{guid}/health\n\n\n```\n{\n  "title": "DispenserHealth",\n  "type": "object",\n  "properties": {\n    "health_status": {\n      "title": "Health Status",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "health_message": {\n      "title": "Health Message",\n      "nullable": true,\n      "type": "string"\n    },\n    "id_": {\n      "title": "Id ",\n      "maxLength": 255,\n      "type": "string"\n    }\n  },\n  "required": [\n    "health_status",\n    "id_"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /ingestors/{guid}/state\n\n\n```\n{\n  "title": "IngestorState",\n  "type": "object",\n  "properties": {\n    "time": {\n      "title": "Time",\n      "default": {\n        "sec": 0,\n        "nanosec": 0\n      },\n      "allOf": [\n        {\n          "$ref": "#/definitions/Time"\n        }\n      ]\n    },\n    "guid": {\n      "title": "Guid",\n      "default": "",\n      "type": "string"\n    },\n    "mode": {\n      "title": "Mode",\n      "default": 0,\n      "minimum": -2147483648,\n      "maximum": 2147483647,\n      "type": "integer"\n    },\n    "request_guid_queue": {\n      "title": "Request Guid Queue",\n      "default": [],\n      "type": "array",\n      "items": {\n        "type": "string"\n      }\n    },\n    "seconds_remaining": {\n      "title": "Seconds Remaining",\n      "default": 0,\n      "type": "number"\n    }\n  },\n  "required": [\n    "time",\n    "guid",\n    "mode",\n    "request_guid_queue",\n    "seconds_remaining"\n  ],\n  "definitions": {\n    "Time": {\n      "title": "Time",\n      "type": "object",\n      "properties": {\n        "sec": {\n          "title": "Sec",\n          "default": 0,\n          "minimum": -2147483648,\n          "maximum": 2147483647,\n          "type": "integer"\n        },\n        "nanosec": {\n          "title": "Nanosec",\n          "default": 0,\n          "minimum": 0,\n          "maximum": 4294967295,\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ]\n    }\n  }\n}\n```\n\n\n### /ingestors/{guid}/health\n\n\n```\n{\n  "title": "IngestorHealth",\n  "type": "object",\n  "properties": {\n    "health_status": {\n      "title": "Health Status",\n      "maxLength": 255,\n      "nullable": true,\n      "type": "string"\n    },\n    "health_message": {\n      "title": "Health Message",\n      "nullable": true,\n      "type": "string"\n    },\n    "id_": {\n      "title": "Id ",\n      "maxLength": 255,\n      "type": "string"\n    }\n  },\n  "required": [\n    "health_status",\n    "id_"\n  ],\n  "additionalProperties": false\n}\n```\n\n\n### /fleets/{name}/state\n\n\n```\n{\n  "title": "FleetState",\n  "type": "object",\n  "properties": {\n    "name": {\n      "title": "Name",\n      "type": "string"\n    },\n    "robots": {\n      "title": "Robots",\n      "description": "A dictionary of the states of the robots that belong to this fleet",\n      "type": "object",\n      "additionalProperties": {\n        "$ref": "#/definitions/RobotState"\n      }\n    }\n  },\n  "definitions": {\n    "Status2": {\n      "title": "Status2",\n      "description": "An enumeration.",\n      "enum": [\n        "uninitialized",\n        "offline",\n        "shutdown",\n        "idle",\n        "charging",\n        "working",\n        "error"\n      ]\n    },\n    "Location2D": {\n      "title": "Location2D",\n      "type": "object",\n      "properties": {\n        "map": {\n          "title": "Map",\n          "type": "string"\n        },\n        "x": {\n          "title": "X",\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "type": "number"\n        }\n      },\n      "required": [\n        "map",\n        "x",\n        "y",\n        "yaw"\n      ]\n    },\n    "Issue": {\n      "title": "Issue",\n      "type": "object",\n      "properties": {\n        "category": {\n          "title": "Category",\n          "description": "Category of the robot\'s issue",\n          "type": "string"\n        },\n        "detail": {\n          "title": "Detail",\n          "description": "Detailed information about the issue",\n          "anyOf": [\n            {\n              "type": "object"\n            },\n            {\n              "type": "array",\n              "items": {}\n            },\n            {\n              "type": "string"\n            }\n          ]\n        }\n      }\n    },\n    "RobotState": {\n      "title": "RobotState",\n      "type": "object",\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "status": {\n          "description": "A simple token representing the status of the robot",\n          "allOf": [\n            {\n              "$ref": "#/definitions/Status2"\n            }\n          ]\n        },\n        "task_id": {\n          "title": "Task Id",\n          "description": "The ID of the task this robot is currently working on. Empty string if the robot is not working on a task.",\n          "type": "string"\n        },\n        "unix_millis_time": {\n          "title": "Unix Millis Time",\n          "type": "integer"\n        },\n        "location": {\n          "$ref": "#/definitions/Location2D"\n        },\n        "battery": {\n          "title": "Battery",\n          "description": "State of charge of the battery. Values range from 0.0 (depleted) to 1.0 (fully charged)",\n          "minimum": 0.0,\n          "maximum": 1.0,\n          "type": "number"\n        },\n        "issues": {\n          "title": "Issues",\n          "description": "A list of issues with the robot that operators need to address",\n          "type": "array",\n          "items": {\n            "$ref": "#/definitions/Issue"\n          }\n        }\n      }\n    }\n  }\n}\n```\n\n\n### /fleets/{name}/log\n\n\n```\n{\n  "title": "FleetState",\n  "type": "object",\n  "properties": {\n    "name": {\n      "title": "Name",\n      "type": "string"\n    },\n    "log": {\n      "title": "Log",\n      "description": "Log for the overall fleet",\n      "type": "array",\n      "items": {\n        "$ref": "#/definitions/LogEntry"\n      }\n    },\n    "robots": {\n      "title": "Robots",\n      "description": "Dictionary of logs for the individual robots. The keys (property names) are the robot names.",\n      "type": "object",\n      "additionalProperties": {\n        "type": "array",\n        "items": {\n          "$ref": "#/definitions/LogEntry"\n        }\n      }\n    }\n  },\n  "definitions": {\n    "Tier": {\n      "title": "Tier",\n      "description": "An enumeration.",\n      "enum": [\n        "uninitialized",\n        "info",\n        "warning",\n        "error"\n      ]\n    },\n    "LogEntry": {\n      "title": "LogEntry",\n      "type": "object",\n      "properties": {\n        "seq": {\n          "title": "Seq",\n          "description": "Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.",\n          "exclusiveMaximum": 4294967296,\n          "minimum": 0,\n          "type": "integer"\n        },\n        "tier": {\n          "description": "The importance level of the log entry",\n          "allOf": [\n            {\n              "$ref": "#/definitions/Tier"\n            }\n          ]\n        },\n        "unix_millis_time": {\n          "title": "Unix Millis Time",\n          "type": "integer"\n        },\n        "text": {\n          "title": "Text",\n          "description": "The text of the log entry",\n          "type": "string"\n        }\n      },\n      "required": [\n        "seq",\n        "tier",\n        "unix_millis_time",\n        "text"\n      ]\n    }\n  }\n}\n```\n\n',
        operationId: '_lambda__socket_io_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
        },
      },
    },
    '/user': {
      get: {
        summary: 'Get User',
        description: 'Get the currently logged in user',
        operationId: 'get_user_user_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/User' } } },
          },
        },
      },
    },
    '/permissions': {
      get: {
        summary: 'Get Effective Permissions',
        description: 'Get the effective permissions of the current user',
        operationId: 'get_effective_permissions_permissions_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Effective Permissions Permissions Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Permission' },
                },
              },
            },
          },
        },
      },
    },
    '/time': {
      get: {
        summary: 'Get Time',
        description: 'Get the current rmf time in unix milliseconds',
        operationId: 'get_time_time_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: { title: 'Response Get Time Time Get', type: 'integer' },
              },
            },
          },
        },
      },
    },
    '/alerts': {
      get: {
        tags: ['Alerts'],
        summary: 'Get Alerts',
        operationId: 'get_alerts_alerts_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Alerts Alerts Get',
                  type: 'array',
                  items: {
                    $ref: '#/components/schemas/api_server.models.tortoise_models.alerts.Alert.leaf',
                  },
                },
              },
            },
          },
        },
      },
      post: {
        tags: ['Alerts'],
        summary: 'Create Alert',
        operationId: 'create_alert_alerts_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Alert Id', type: 'string' },
            name: 'alert_id',
            in: 'query',
          },
          {
            required: true,
            schema: { title: 'Category', type: 'string' },
            name: 'category',
            in: 'query',
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.alerts.Alert.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/alerts/{alert_id}': {
      get: {
        tags: ['Alerts'],
        summary: 'Get Alert',
        operationId: 'get_alert_alerts__alert_id__get',
        parameters: [
          {
            required: true,
            schema: { title: 'Alert Id', type: 'string' },
            name: 'alert_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.alerts.Alert.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      post: {
        tags: ['Alerts'],
        summary: 'Acknowledge Alert',
        operationId: 'acknowledge_alert_alerts__alert_id__post',
        parameters: [
          {
            required: true,
            schema: { title: 'Alert Id', type: 'string' },
            name: 'alert_id',
            in: 'path',
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.alerts.Alert.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/beacons': {
      get: {
        tags: ['Beacons'],
        summary: 'Get Beacons',
        operationId: 'get_beacons_beacons_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Beacons Beacons Get',
                  type: 'array',
                  items: {
                    $ref: '#/components/schemas/api_server.models.tortoise_models.beacons.BeaconState.leaf',
                  },
                },
              },
            },
          },
        },
      },
      post: {
        tags: ['Beacons'],
        summary: 'Save Beacon State',
        operationId: 'save_beacon_state_beacons_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Beacon Id', type: 'string' },
            name: 'beacon_id',
            in: 'query',
          },
          {
            required: true,
            schema: { title: 'Online', type: 'boolean' },
            name: 'online',
            in: 'query',
          },
          {
            required: true,
            schema: { title: 'Category', type: 'string' },
            name: 'category',
            in: 'query',
          },
          {
            required: true,
            schema: { title: 'Activated', type: 'boolean' },
            name: 'activated',
            in: 'query',
          },
          {
            required: true,
            schema: { title: 'Level', type: 'string' },
            name: 'level',
            in: 'query',
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.beacons.BeaconState.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/beacons/{beacon_id}': {
      get: {
        tags: ['Beacons'],
        summary: 'Get Beacon',
        operationId: 'get_beacon_beacons__beacon_id__get',
        parameters: [
          {
            required: true,
            schema: { title: 'Beacon Id', type: 'string' },
            name: 'beacon_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.beacons.BeaconState.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/building_map': {
      get: {
        tags: ['Building'],
        summary: 'Get Building Map',
        description: 'Available in socket.io',
        operationId: 'get_building_map_building_map_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/BuildingMap' } },
            },
          },
        },
      },
    },
    '/doors': {
      get: {
        tags: ['Doors'],
        summary: 'Get Doors',
        operationId: 'get_doors_doors_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Doors Doors Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Door' },
                },
              },
            },
          },
        },
      },
    },
    '/doors/{door_name}/state': {
      get: {
        tags: ['Doors'],
        summary: 'Get Door State',
        description: 'Available in socket.io',
        operationId: 'get_door_state_doors__door_name__state_get',
        parameters: [
          {
            required: true,
            schema: { title: 'Door Name', type: 'string' },
            name: 'door_name',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/DoorState' } } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/doors/{door_name}/health': {
      get: {
        tags: ['Doors'],
        summary: 'Get Door Health',
        description: 'Available in socket.io',
        operationId: 'get_door_health_doors__door_name__health_get',
        parameters: [
          {
            required: true,
            schema: { title: 'Door Name', type: 'string' },
            name: 'door_name',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/DoorHealth' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/doors/{door_name}/request': {
      post: {
        tags: ['Doors'],
        summary: 'Post Door Request',
        operationId: 'post_door_request_doors__door_name__request_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Door Name', type: 'string' },
            name: 'door_name',
            in: 'path',
          },
        ],
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/DoorRequest' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/lifts': {
      get: {
        tags: ['Lifts'],
        summary: 'Get Lifts',
        operationId: 'get_lifts_lifts_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Lifts Lifts Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Lift' },
                },
              },
            },
          },
        },
      },
    },
    '/lifts/{lift_name}/state': {
      get: {
        tags: ['Lifts'],
        summary: 'Get Lift State',
        description: 'Available in socket.io',
        operationId: 'get_lift_state_lifts__lift_name__state_get',
        parameters: [
          {
            required: true,
            schema: { title: 'Lift Name', type: 'string' },
            name: 'lift_name',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/LiftState' } } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/lifts/{lift_name}/health': {
      get: {
        tags: ['Lifts'],
        summary: 'Get Lift Health',
        description: 'Available in socket.io',
        operationId: 'get_lift_health_lifts__lift_name__health_get',
        parameters: [
          {
            required: true,
            schema: { title: 'Lift Name', type: 'string' },
            name: 'lift_name',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/LiftHealth' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/lifts/{lift_name}/request': {
      post: {
        tags: ['Lifts'],
        summary: ' Post Lift Request',
        operationId: '_post_lift_request_lifts__lift_name__request_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Lift Name', type: 'string' },
            name: 'lift_name',
            in: 'path',
          },
        ],
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/LiftRequest' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks': {
      get: {
        tags: ['Tasks'],
        summary: 'Query Task States',
        operationId: 'query_task_states_tasks_get',
        parameters: [
          {
            description: 'comma separated list of task ids',
            required: false,
            schema: {
              title: 'Task Id',
              type: 'string',
              description: 'comma separated list of task ids',
            },
            name: 'task_id',
            in: 'query',
          },
          {
            description: 'comma separated list of task categories',
            required: false,
            schema: {
              title: 'Category',
              type: 'string',
              description: 'comma separated list of task categories',
            },
            name: 'category',
            in: 'query',
          },
          {
            description: 'comma separated list of assigned robot names',
            required: false,
            schema: {
              title: 'Assigned To',
              type: 'string',
              description: 'comma separated list of assigned robot names',
            },
            name: 'assigned_to',
            in: 'query',
          },
          {
            description: 'comma separated list of statuses',
            required: false,
            schema: {
              title: 'Status',
              type: 'string',
              description: 'comma separated list of statuses',
            },
            name: 'status',
            in: 'query',
          },
          {
            description:
              '\n        The period of starting time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n        ',
            required: false,
            schema: {
              title: 'Start Time Between',
              type: 'string',
              description:
                '\n        The period of starting time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n        ',
            },
            name: 'start_time_between',
            in: 'query',
          },
          {
            description:
              '\n        The period of finishing time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
            required: false,
            schema: {
              title: 'Finish Time Between',
              type: 'string',
              description:
                '\n        The period of finishing time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
            },
            name: 'finish_time_between',
            in: 'query',
          },
          {
            description: 'defaults to 100',
            required: false,
            schema: {
              title: 'Limit',
              maximum: 1000.0,
              exclusiveMinimum: 0.0,
              type: 'integer',
              description: 'defaults to 100',
            },
            name: 'limit',
            in: 'query',
          },
          {
            description: 'defaults to 0',
            required: false,
            schema: {
              title: 'Offset',
              minimum: 0.0,
              type: 'integer',
              description: 'defaults to 0',
            },
            name: 'offset',
            in: 'query',
          },
          {
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            required: false,
            schema: {
              title: 'Order By',
              type: 'string',
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            },
            name: 'order_by',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Query Task States Tasks Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/TaskState' },
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/{task_id}/state': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Task State',
        description: 'Available in socket.io',
        operationId: 'get_task_state_tasks__task_id__state_get',
        parameters: [
          {
            description: 'task_id',
            required: true,
            schema: { title: 'Task Id', type: 'string', description: 'task_id' },
            name: 'task_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/TaskState' } } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/{task_id}/log': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Task Log',
        description: 'Available in socket.io',
        operationId: 'get_task_log_tasks__task_id__log_get',
        parameters: [
          {
            description: 'task_id',
            required: true,
            schema: { title: 'Task Id', type: 'string', description: 'task_id' },
            name: 'task_id',
            in: 'path',
          },
          {
            description:
              '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
            required: false,
            schema: {
              title: 'Between',
              type: 'string',
              description:
                '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
              default: '-60000',
            },
            name: 'between',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskEventLog' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/activity_discovery': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Activity Discovery',
        operationId: 'post_activity_discovery_tasks_activity_discovery_post',
        requestBody: {
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/ActivityDiscoveryRequest' },
            },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/ActivityDiscovery' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/cancel_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Cancel Task',
        operationId: 'post_cancel_task_tasks_cancel_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/CancelTaskRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskCancelResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/dispatch_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Dispatch Task',
        operationId: 'post_dispatch_task_tasks_dispatch_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/DispatchTaskRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/TaskDispatchResponseItem' },
              },
            },
          },
          '400': {
            description: 'Bad Request',
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/TaskDispatchResponseItem1' },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/robot_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Robot Task',
        operationId: 'post_robot_task_tasks_robot_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/RobotTaskRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/RobotTaskResponse' } },
            },
          },
          '400': {
            description: 'Bad Request',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/RobotTaskResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/interrupt_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Interrupt Task',
        operationId: 'post_interrupt_task_tasks_interrupt_task_post',
        requestBody: {
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/TaskInterruptionRequest' },
            },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/TaskInterruptionResponse' },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/kill_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Kill Task',
        operationId: 'post_kill_task_tasks_kill_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskKillRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskKillResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/resume_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Resume Task',
        operationId: 'post_resume_task_tasks_resume_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskResumeRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskResumeResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/rewind_task': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Rewind Task',
        operationId: 'post_rewind_task_tasks_rewind_task_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskRewindRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskRewindResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/skip_phase': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Skip Phase',
        operationId: 'post_skip_phase_tasks_skip_phase_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskPhaseSkipRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/SkipPhaseResponse' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/task_discovery': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Task Discovery',
        operationId: 'post_task_discovery_tasks_task_discovery_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskDiscoveryRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskDiscovery' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/tasks/undo_skip_phase': {
      post: {
        tags: ['Tasks'],
        summary: 'Post Undo Skip Phase',
        operationId: 'post_undo_skip_phase_tasks_undo_skip_phase_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/UndoPhaseSkipRequest' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: { $ref: '#/components/schemas/UndoPhaseSkipResponse' },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/scheduled_tasks': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Scheduled Tasks',
        operationId: 'get_scheduled_tasks_scheduled_tasks_get',
        parameters: [
          {
            description: 'Only return scheduled tasks that start before given timestamp',
            required: true,
            schema: {
              title: 'Start Before',
              type: 'string',
              description: 'Only return scheduled tasks that start before given timestamp',
              format: 'date-time',
            },
            name: 'start_before',
            in: 'query',
          },
          {
            description: 'Only return scheduled tasks that stop after given timestamp',
            required: true,
            schema: {
              title: 'Until After',
              type: 'string',
              description: 'Only return scheduled tasks that stop after given timestamp',
              format: 'date-time',
            },
            name: 'until_after',
            in: 'query',
          },
          {
            description: 'defaults to 100',
            required: false,
            schema: {
              title: 'Limit',
              maximum: 1000.0,
              exclusiveMinimum: 0.0,
              type: 'integer',
              description: 'defaults to 100',
            },
            name: 'limit',
            in: 'query',
          },
          {
            description: 'defaults to 0',
            required: false,
            schema: {
              title: 'Offset',
              minimum: 0.0,
              type: 'integer',
              description: 'defaults to 0',
            },
            name: 'offset',
            in: 'query',
          },
          {
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            required: false,
            schema: {
              title: 'Order By',
              type: 'string',
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            },
            name: 'order_by',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTask_list',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      post: {
        tags: ['Tasks'],
        summary: 'Post Scheduled Task',
        description:
          'Create a scheduled task. Below are some examples of how the schedules are represented.\nFor more examples, check the docs of the underlying library used [here](https://github.com/dbader/schedule/blob/6eb0b5346b1ce35ece5050e65789fa6e44368175/docs/examples.rst).\n\n| every | to | period | at | description |\n| - | - | - | - | - |\n| 10 | - | minutes | - | Every 10 minutes |\n| - | - | hour | - | Every hour |\n| - | - | day | 10:30 | Every day at 10:30am |\n| - | - | monday | - | Every monday |\n| - | - | wednesday | 13:15 | Every wednesday at 01:15pm |\n| - | - | minute | :17 | Every 17th sec of a mintue |\n| 5 | 10 | seconds | - | Every 5-10 seconds (randomly) |',
        operationId: 'post_scheduled_task_scheduled_tasks_post',
        requestBody: {
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/PostScheduledTaskRequest' },
            },
          },
          required: true,
        },
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTask',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/scheduled_tasks/{task_id}': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Scheduled Task',
        operationId: 'get_scheduled_task_scheduled_tasks__task_id__get',
        parameters: [
          {
            required: true,
            schema: { title: 'Task Id', type: 'integer' },
            name: 'task_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTask',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      delete: {
        tags: ['Tasks'],
        summary: 'Del Scheduled Tasks',
        operationId: 'del_scheduled_tasks_scheduled_tasks__task_id__delete',
        parameters: [
          {
            required: true,
            schema: { title: 'Task Id', type: 'integer' },
            name: 'task_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/scheduled_tasks/{task_id}/clear': {
      put: {
        tags: ['Tasks'],
        summary: 'Del Scheduled Tasks Event',
        operationId: 'del_scheduled_tasks_event_scheduled_tasks__task_id__clear_put',
        parameters: [
          {
            required: true,
            schema: { title: 'Task Id', type: 'integer' },
            name: 'task_id',
            in: 'path',
          },
          {
            required: true,
            schema: { title: 'Event Date', type: 'string', format: 'date-time' },
            name: 'event_date',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/scheduled_tasks/{task_id}/update': {
      post: {
        tags: ['Tasks'],
        summary: 'Update Schedule Task',
        operationId: 'update_schedule_task_scheduled_tasks__task_id__update_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Task Id', type: 'integer' },
            name: 'task_id',
            in: 'path',
          },
          {
            required: false,
            schema: { title: 'Except Date', type: 'string', format: 'date-time' },
            name: 'except_date',
            in: 'query',
          },
        ],
        requestBody: {
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/PostScheduledTaskRequest' },
            },
          },
          required: true,
        },
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTask',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/favorite_tasks': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Favorites Tasks',
        operationId: 'get_favorites_tasks_favorite_tasks_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Favorites Tasks Favorite Tasks Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/TaskFavoritePydantic' },
                },
              },
            },
          },
        },
      },
      post: {
        tags: ['Tasks'],
        summary: 'Post Favorite Task',
        operationId: 'post_favorite_task_favorite_tasks_post',
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskFavoritePydantic' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server.models.tortoise_models.tasks.TaskFavorite.leaf',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/favorite_tasks/{favorite_task_id}': {
      delete: {
        tags: ['Tasks'],
        summary: 'Delete Favorite Task',
        operationId: 'delete_favorite_task_favorite_tasks__favorite_task_id__delete',
        parameters: [
          {
            required: true,
            schema: { title: 'Favorite Task Id', type: 'string' },
            name: 'favorite_task_id',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/dispensers': {
      get: {
        tags: ['Dispensers'],
        summary: 'Get Dispensers',
        operationId: 'get_dispensers_dispensers_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Dispensers Dispensers Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Dispenser' },
                },
              },
            },
          },
        },
      },
    },
    '/dispensers/{guid}/state': {
      get: {
        tags: ['Dispensers'],
        summary: 'Get Dispenser State',
        description: 'Available in socket.io',
        operationId: 'get_dispenser_state_dispensers__guid__state_get',
        parameters: [
          { required: true, schema: { title: 'Guid', type: 'string' }, name: 'guid', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/DispenserState' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/dispensers/{guid}/health': {
      get: {
        tags: ['Dispensers'],
        summary: 'Get Dispenser Health',
        description: 'Available in socket.io',
        operationId: 'get_dispenser_health_dispensers__guid__health_get',
        parameters: [
          { required: true, schema: { title: 'Guid', type: 'string' }, name: 'guid', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/DispenserHealth' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/ingestors': {
      get: {
        tags: ['Ingestors'],
        summary: 'Get Ingestors',
        operationId: 'get_ingestors_ingestors_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Ingestors Ingestors Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Ingestor' },
                },
              },
            },
          },
        },
      },
    },
    '/ingestors/{guid}/state': {
      get: {
        tags: ['Ingestors'],
        summary: 'Get Ingestor State',
        description: 'Available in socket.io',
        operationId: 'get_ingestor_state_ingestors__guid__state_get',
        parameters: [
          { required: true, schema: { title: 'Guid', type: 'string' }, name: 'guid', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/IngestorState' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/ingestors/{guid}/health': {
      get: {
        tags: ['Ingestors'],
        summary: 'Get Ingestor Health',
        description: 'Available in socket.io',
        operationId: 'get_ingestor_health_ingestors__guid__health_get',
        parameters: [
          { required: true, schema: { title: 'Guid', type: 'string' }, name: 'guid', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/IngestorHealth' } },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/fleets': {
      get: {
        tags: ['Fleets'],
        summary: 'Get Fleets',
        operationId: 'get_fleets_fleets_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Fleets Fleets Get',
                  type: 'array',
                  items: {
                    $ref: '#/components/schemas/api_server__models__rmf_api__fleet_state__FleetState',
                  },
                },
              },
            },
          },
        },
      },
    },
    '/fleets/{name}/state': {
      get: {
        tags: ['Fleets'],
        summary: 'Get Fleet State',
        description: 'Available in socket.io',
        operationId: 'get_fleet_state_fleets__name__state_get',
        parameters: [
          { required: true, schema: { title: 'Name', type: 'string' }, name: 'name', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server__models__rmf_api__fleet_state__FleetState',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/fleets/{name}/log': {
      get: {
        tags: ['Fleets'],
        summary: 'Get Fleet Log',
        description: 'Available in socket.io',
        operationId: 'get_fleet_log_fleets__name__log_get',
        parameters: [
          { required: true, schema: { title: 'Name', type: 'string' }, name: 'name', in: 'path' },
          {
            description:
              '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
            required: false,
            schema: {
              title: 'Between',
              type: 'string',
              description:
                '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
              default: '-60000',
            },
            name: 'between',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/api_server__models__rmf_api__fleet_log__FleetState',
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/users': {
      get: {
        tags: ['Admin'],
        summary: 'Get Users',
        description: 'Search users',
        operationId: 'get_users_admin_users_get',
        parameters: [
          {
            description: 'filters username that starts with the value',
            required: false,
            schema: {
              title: 'Username',
              type: 'string',
              description: 'filters username that starts with the value',
            },
            name: 'username',
            in: 'query',
          },
          {
            required: false,
            schema: { title: 'Is Admin', type: 'boolean' },
            name: 'is_admin',
            in: 'query',
          },
          {
            description: 'defaults to 100',
            required: false,
            schema: {
              title: 'Limit',
              maximum: 1000.0,
              exclusiveMinimum: 0.0,
              type: 'integer',
              description: 'defaults to 100',
            },
            name: 'limit',
            in: 'query',
          },
          {
            description: 'defaults to 0',
            required: false,
            schema: {
              title: 'Offset',
              minimum: 0.0,
              type: 'integer',
              description: 'defaults to 0',
            },
            name: 'offset',
            in: 'query',
          },
          {
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            required: false,
            schema: {
              title: 'Order By',
              type: 'string',
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
            },
            name: 'order_by',
            in: 'query',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Users Admin Users Get',
                  type: 'array',
                  items: { type: 'string' },
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      post: {
        tags: ['Admin'],
        summary: 'Create User',
        description: 'Create a user',
        operationId: 'create_user_admin_users_post',
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostUsers' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/users/{username}': {
      get: {
        tags: ['Admin'],
        summary: 'Get User',
        description: 'Get a user',
        operationId: 'get_user_admin_users__username__get',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/User' } } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      delete: {
        tags: ['Admin'],
        summary: 'Delete User',
        description:
          'Delete a user\n\nThis only performs a soft delete, while the user is deleted from the app database,\nit still exists in the idp so they can still log in, the user will then be re-created\nwith the default permissions.',
        operationId: 'delete_user_admin_users__username__delete',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/users/{username}/make_admin': {
      post: {
        tags: ['Admin'],
        summary: 'Make Admin',
        description: 'Make or remove admin privilege from a user',
        operationId: 'make_admin_admin_users__username__make_admin_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
        ],
        requestBody: {
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/PostMakeAdmin' } },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/users/{username}/roles': {
      put: {
        tags: ['Admin'],
        summary: 'Set User Roles',
        description: 'Set the roles of a user',
        operationId: 'set_user_roles_admin_users__username__roles_put',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
        ],
        requestBody: {
          content: {
            'application/json': {
              schema: {
                title: 'Body',
                type: 'array',
                items: { $ref: '#/components/schemas/PostRoles' },
              },
            },
          },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      post: {
        tags: ['Admin'],
        summary: 'Add User Role',
        description: 'Add role to a user',
        operationId: 'add_user_role_admin_users__username__roles_post',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
        ],
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostRoles' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/users/{username}/roles/{role}': {
      delete: {
        tags: ['Admin'],
        summary: 'Delete User Role',
        description: 'Remove role from a user',
        operationId: 'delete_user_role_admin_users__username__roles__role__delete',
        parameters: [
          {
            required: true,
            schema: { title: 'Username', type: 'string' },
            name: 'username',
            in: 'path',
          },
          { required: true, schema: { title: 'Role', type: 'string' }, name: 'role', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/roles': {
      get: {
        tags: ['Admin'],
        summary: 'Get Roles',
        description: 'Get all roles',
        operationId: 'get_roles_admin_roles_get',
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Roles Admin Roles Get',
                  type: 'array',
                  items: { type: 'string' },
                },
              },
            },
          },
        },
      },
      post: {
        tags: ['Admin'],
        summary: 'Create Role',
        description: 'Create a new role',
        operationId: 'create_role_admin_roles_post',
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostRoles' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/roles/{role}': {
      delete: {
        tags: ['Admin'],
        summary: 'Delete Role',
        description: 'Delete a role',
        operationId: 'delete_role_admin_roles__role__delete',
        parameters: [
          { required: true, schema: { title: 'Role', type: 'string' }, name: 'role', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/roles/{role}/permissions': {
      get: {
        tags: ['Admin'],
        summary: 'Get Role Permissions',
        description: 'Get all permissions of a role',
        operationId: 'get_role_permissions_admin_roles__role__permissions_get',
        parameters: [
          { required: true, schema: { title: 'Role', type: 'string' }, name: 'role', in: 'path' },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  title: 'Response Get Role Permissions Admin Roles  Role  Permissions Get',
                  type: 'array',
                  items: { $ref: '#/components/schemas/Permission' },
                },
              },
            },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
      post: {
        tags: ['Admin'],
        summary: 'Add Role Permission',
        description: 'Add a permission to a role',
        operationId: 'add_role_permission_admin_roles__role__permissions_post',
        parameters: [
          { required: true, schema: { title: 'Role', type: 'string' }, name: 'role', in: 'path' },
        ],
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/Permission' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
    '/admin/roles/{role}/permissions/remove': {
      post: {
        tags: ['Admin'],
        summary: 'Remove Role Permission',
        description: 'Delete a permission from a role',
        operationId: 'remove_role_permission_admin_roles__role__permissions_remove_post',
        parameters: [
          { required: true, schema: { title: 'Role', type: 'string' }, name: 'role', in: 'path' },
        ],
        requestBody: {
          content: { 'application/json': { schema: { $ref: '#/components/schemas/Permission' } } },
          required: true,
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: {} } },
          },
          '422': {
            description: 'Validation Error',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/HTTPValidationError' } },
            },
          },
        },
      },
    },
  },
  components: {
    schemas: {
      Activity: {
        title: 'Activity',
        required: ['category', 'detail'],
        type: 'object',
        properties: {
          category: {
            title: 'Category',
            type: 'string',
            description:
              'The category of this activity. There must not be any duplicate activity categories per fleet.',
          },
          detail: {
            title: 'Detail',
            type: 'string',
            description: 'Details about the behavior of the activity.',
          },
          description_schema: {
            title: 'Description Schema',
            type: 'object',
            description: 'The schema for this activity description',
          },
        },
      },
      ActivityDiscovery: {
        title: 'ActivityDiscovery',
        type: 'object',
        properties: {
          data: { title: 'Data', type: 'array', items: { $ref: '#/components/schemas/Datum' } },
        },
      },
      ActivityDiscoveryRequest: {
        title: 'ActivityDiscoveryRequest',
        required: ['type'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['activitiy_discovery_request'],
            type: 'string',
            description: 'Indicate that this is an activity discovery request',
          },
        },
      },
      AffineImage: {
        title: 'AffineImage',
        required: ['name', 'x_offset', 'y_offset', 'yaw', 'scale', 'encoding', 'data'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          x_offset: { title: 'X Offset', type: 'number', default: 0 },
          y_offset: { title: 'Y Offset', type: 'number', default: 0 },
          yaw: { title: 'Yaw', type: 'number', default: 0 },
          scale: { title: 'Scale', type: 'number', default: 0 },
          encoding: { title: 'Encoding', type: 'string', default: '' },
          data: { title: 'Data', type: 'string' },
        },
      },
      AssignedTo: {
        title: 'AssignedTo',
        required: ['group', 'name'],
        type: 'object',
        properties: {
          group: { title: 'Group', type: 'string' },
          name: { title: 'Name', type: 'string' },
        },
      },
      Assignment: {
        title: 'Assignment',
        type: 'object',
        properties: {
          fleet_name: { title: 'Fleet Name', type: 'string' },
          expected_robot_name: { title: 'Expected Robot Name', type: 'string' },
        },
      },
      Booking: {
        title: 'Booking',
        required: ['id'],
        type: 'object',
        properties: {
          id: { title: 'Id', type: 'string', description: 'The unique identifier for this task' },
          unix_millis_earliest_start_time: {
            title: 'Unix Millis Earliest Start Time',
            type: 'integer',
          },
          unix_millis_request_time: { title: 'Unix Millis Request Time', type: 'integer' },
          priority: {
            title: 'Priority',
            anyOf: [{ type: 'object' }, { type: 'string' }],
            description: 'Priority information about this task',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Information about how and why this task was booked',
          },
          requester: {
            title: 'Requester',
            type: 'string',
            description: '(Optional) An identifier for the entity that requested this task',
          },
        },
      },
      BuildingMap: {
        title: 'BuildingMap',
        required: ['name', 'levels', 'lifts'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          levels: { title: 'Levels', type: 'array', items: { $ref: '#/components/schemas/Level' } },
          lifts: {
            title: 'Lifts',
            type: 'array',
            items: { $ref: '#/components/schemas/Lift' },
            default: [],
          },
        },
      },
      CancelTaskRequest: {
        title: 'CancelTaskRequest',
        required: ['type', 'task_id'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['cancel_task_request'],
            type: 'string',
            description: 'Indicate that this is a task cancellation request',
          },
          task_id: {
            title: 'Task Id',
            type: 'string',
            description: 'Specify the task ID to cancel',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the cancellation',
          },
        },
      },
      Cancellation: {
        title: 'Cancellation',
        required: ['unix_millis_request_time', 'labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the cancellation request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the cancel request',
          },
        },
      },
      Category: {
        title: 'Category',
        type: 'string',
        description: 'The category of this task or phase',
      },
      Data: {
        title: 'Data',
        type: 'object',
        properties: {
          fleet_name: {
            title: 'Fleet Name',
            type: 'string',
            description: 'Name of the fleet that supports these tasks',
          },
          tasks: {
            title: 'Tasks',
            type: 'array',
            items: { $ref: '#/components/schemas/Task' },
            description: '(list:replace) List of tasks that the fleet supports',
          },
        },
      },
      Datum: {
        title: 'Datum',
        required: ['fleet_name', 'activities'],
        type: 'object',
        properties: {
          fleet_name: {
            title: 'Fleet Name',
            type: 'string',
            description: 'Name of the fleet that supports these activities',
          },
          activities: {
            title: 'Activities',
            type: 'array',
            items: { $ref: '#/components/schemas/Activity' },
            description: 'List of activities that the fleet supports',
          },
        },
      },
      Detail: {
        title: 'Detail',
        anyOf: [{ type: 'object' }, { type: 'array', items: {} }, { type: 'string' }],
        description: 'Detailed information about a task, phase, or event',
      },
      Dispatch: {
        title: 'Dispatch',
        required: ['status'],
        type: 'object',
        properties: {
          status: { $ref: '#/components/schemas/Status1' },
          assignment: { $ref: '#/components/schemas/Assignment' },
          errors: { title: 'Errors', type: 'array', items: { $ref: '#/components/schemas/Error' } },
        },
      },
      DispatchTaskRequest: {
        title: 'DispatchTaskRequest',
        required: ['type', 'request'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['dispatch_task_request'],
            type: 'string',
            description: 'Indicate that this is a task dispatch request',
          },
          request: { $ref: '#/components/schemas/TaskRequest' },
        },
      },
      Dispenser: {
        title: 'Dispenser',
        required: ['guid'],
        type: 'object',
        properties: { guid: { title: 'Guid', type: 'string' } },
      },
      DispenserHealth: {
        title: 'DispenserHealth',
        required: ['health_status', 'id_'],
        type: 'object',
        properties: {
          health_status: { title: 'Health Status', maxLength: 255, type: 'string', nullable: true },
          health_message: { title: 'Health Message', type: 'string', nullable: true },
          id_: { title: 'Id ', maxLength: 255, type: 'string' },
        },
        additionalProperties: false,
      },
      DispenserState: {
        title: 'DispenserState',
        required: ['time', 'guid', 'mode', 'request_guid_queue', 'seconds_remaining'],
        type: 'object',
        properties: {
          time: {
            title: 'Time',
            allOf: [{ $ref: '#/components/schemas/Time' }],
            default: { sec: 0, nanosec: 0 },
          },
          guid: { title: 'Guid', type: 'string', default: '' },
          mode: {
            title: 'Mode',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            type: 'integer',
            default: 0,
          },
          request_guid_queue: {
            title: 'Request Guid Queue',
            type: 'array',
            items: { type: 'string' },
            default: [],
          },
          seconds_remaining: { title: 'Seconds Remaining', type: 'number', default: 0 },
        },
      },
      Door: {
        title: 'Door',
        required: [
          'name',
          'v1_x',
          'v1_y',
          'v2_x',
          'v2_y',
          'door_type',
          'motion_range',
          'motion_direction',
        ],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          v1_x: { title: 'V1 X', type: 'number', default: 0 },
          v1_y: { title: 'V1 Y', type: 'number', default: 0 },
          v2_x: { title: 'V2 X', type: 'number', default: 0 },
          v2_y: { title: 'V2 Y', type: 'number', default: 0 },
          door_type: {
            title: 'Door Type',
            maximum: 255.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          motion_range: { title: 'Motion Range', type: 'number', default: 0 },
          motion_direction: {
            title: 'Motion Direction',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            type: 'integer',
            default: 0,
          },
        },
      },
      DoorHealth: {
        title: 'DoorHealth',
        required: ['health_status', 'id_'],
        type: 'object',
        properties: {
          health_status: { title: 'Health Status', maxLength: 255, type: 'string', nullable: true },
          health_message: { title: 'Health Message', type: 'string', nullable: true },
          id_: { title: 'Id ', maxLength: 255, type: 'string' },
        },
        additionalProperties: false,
      },
      DoorMode: {
        title: 'DoorMode',
        required: ['value'],
        type: 'object',
        properties: {
          value: {
            title: 'Value',
            maximum: 4294967295.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
        },
      },
      DoorRequest: {
        title: 'DoorRequest',
        required: ['mode'],
        type: 'object',
        properties: {
          mode: {
            title: 'Mode',
            type: 'integer',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/msg/DoorMode.msg',
          },
        },
      },
      DoorState: {
        title: 'DoorState',
        required: ['door_time', 'door_name', 'current_mode'],
        type: 'object',
        properties: {
          door_time: {
            title: 'Door Time',
            allOf: [{ $ref: '#/components/schemas/Time' }],
            default: { sec: 0, nanosec: 0 },
          },
          door_name: { title: 'Door Name', type: 'string', default: '' },
          current_mode: {
            title: 'Current Mode',
            allOf: [{ $ref: '#/components/schemas/DoorMode' }],
            default: { value: 0 },
          },
        },
      },
      Error: {
        title: 'Error',
        type: 'object',
        properties: {
          code: {
            title: 'Code',
            minimum: 0.0,
            type: 'integer',
            description: 'A standard code for the kind of error that has occurred',
          },
          category: { title: 'Category', type: 'string', description: 'The category of the error' },
          detail: { title: 'Detail', type: 'string', description: 'Details about the error' },
        },
      },
      EstimateMillis: {
        title: 'EstimateMillis',
        minimum: 0.0,
        type: 'integer',
        description: 'An estimate, in milliseconds, of how long the subject will take to complete',
      },
      EventState: {
        title: 'EventState',
        required: ['id'],
        type: 'object',
        properties: {
          id: { $ref: '#/components/schemas/Id' },
          status: { $ref: '#/components/schemas/Status' },
          name: { title: 'Name', type: 'string', description: 'The brief name of the event' },
          detail: {
            title: 'Detail',
            allOf: [{ $ref: '#/components/schemas/Detail' }],
            description: 'Detailed information about the event',
          },
          deps: {
            title: 'Deps',
            type: 'array',
            items: { minimum: 0.0, type: 'integer' },
            description:
              'This event may depend on other events. This array contains the IDs of those other event dependencies.',
          },
        },
      },
      Graph: {
        title: 'Graph',
        required: ['name', 'vertices', 'edges', 'params'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          vertices: {
            title: 'Vertices',
            type: 'array',
            items: { $ref: '#/components/schemas/GraphNode' },
            default: [],
          },
          edges: {
            title: 'Edges',
            type: 'array',
            items: { $ref: '#/components/schemas/GraphEdge' },
            default: [],
          },
          params: {
            title: 'Params',
            type: 'array',
            items: { $ref: '#/components/schemas/Param' },
            default: [],
          },
        },
      },
      GraphEdge: {
        title: 'GraphEdge',
        required: ['v1_idx', 'v2_idx', 'params', 'edge_type'],
        type: 'object',
        properties: {
          v1_idx: {
            title: 'V1 Idx',
            maximum: 4294967295.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          v2_idx: {
            title: 'V2 Idx',
            maximum: 4294967295.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          params: {
            title: 'Params',
            type: 'array',
            items: { $ref: '#/components/schemas/Param' },
            default: [],
          },
          edge_type: {
            title: 'Edge Type',
            maximum: 255.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
        },
      },
      GraphNode: {
        title: 'GraphNode',
        required: ['x', 'y', 'name', 'params'],
        type: 'object',
        properties: {
          x: { title: 'X', type: 'number', default: 0 },
          y: { title: 'Y', type: 'number', default: 0 },
          name: { title: 'Name', type: 'string', default: '' },
          params: {
            title: 'Params',
            type: 'array',
            items: { $ref: '#/components/schemas/Param' },
            default: [],
          },
        },
      },
      HTTPValidationError: {
        title: 'HTTPValidationError',
        type: 'object',
        properties: {
          detail: {
            title: 'Detail',
            type: 'array',
            items: { $ref: '#/components/schemas/ValidationError' },
          },
        },
      },
      Id: { title: 'Id', minimum: 0.0, type: 'integer' },
      Ingestor: {
        title: 'Ingestor',
        required: ['guid'],
        type: 'object',
        properties: { guid: { title: 'Guid', type: 'string' } },
      },
      IngestorHealth: {
        title: 'IngestorHealth',
        required: ['health_status', 'id_'],
        type: 'object',
        properties: {
          health_status: { title: 'Health Status', maxLength: 255, type: 'string', nullable: true },
          health_message: { title: 'Health Message', type: 'string', nullable: true },
          id_: { title: 'Id ', maxLength: 255, type: 'string' },
        },
        additionalProperties: false,
      },
      IngestorState: {
        title: 'IngestorState',
        required: ['time', 'guid', 'mode', 'request_guid_queue', 'seconds_remaining'],
        type: 'object',
        properties: {
          time: {
            title: 'Time',
            allOf: [{ $ref: '#/components/schemas/Time' }],
            default: { sec: 0, nanosec: 0 },
          },
          guid: { title: 'Guid', type: 'string', default: '' },
          mode: {
            title: 'Mode',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            type: 'integer',
            default: 0,
          },
          request_guid_queue: {
            title: 'Request Guid Queue',
            type: 'array',
            items: { type: 'string' },
            default: [],
          },
          seconds_remaining: { title: 'Seconds Remaining', type: 'number', default: 0 },
        },
      },
      Interruption: {
        title: 'Interruption',
        required: ['unix_millis_request_time', 'labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the interruption request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the interruption',
          },
          resumed_by: {
            title: 'Resumed By',
            allOf: [{ $ref: '#/components/schemas/ResumedBy' }],
            description:
              'Information about the resume request that ended this interruption. This field will be missing if the interruption is still active.',
          },
        },
      },
      Issue: {
        title: 'Issue',
        type: 'object',
        properties: {
          category: {
            title: 'Category',
            type: 'string',
            description: "Category of the robot's issue",
          },
          detail: {
            title: 'Detail',
            anyOf: [{ type: 'object' }, { type: 'array', items: {} }, { type: 'string' }],
            description: 'Detailed information about the issue',
          },
        },
      },
      Killed: {
        title: 'Killed',
        required: ['unix_millis_request_time', 'labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the cancellation request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the kill request',
          },
        },
      },
      Level: {
        title: 'Level',
        required: ['name', 'elevation', 'images', 'places', 'doors', 'nav_graphs', 'wall_graph'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          elevation: { title: 'Elevation', type: 'number', default: 0 },
          images: {
            title: 'Images',
            type: 'array',
            items: { $ref: '#/components/schemas/AffineImage' },
          },
          places: {
            title: 'Places',
            type: 'array',
            items: { $ref: '#/components/schemas/Place' },
            default: [],
          },
          doors: {
            title: 'Doors',
            type: 'array',
            items: { $ref: '#/components/schemas/Door' },
            default: [],
          },
          nav_graphs: {
            title: 'Nav Graphs',
            type: 'array',
            items: { $ref: '#/components/schemas/Graph' },
            default: [],
          },
          wall_graph: {
            title: 'Wall Graph',
            allOf: [{ $ref: '#/components/schemas/Graph' }],
            default: { name: '', vertices: [], edges: [], params: [] },
          },
        },
      },
      Lift: {
        title: 'Lift',
        required: [
          'name',
          'levels',
          'doors',
          'wall_graph',
          'ref_x',
          'ref_y',
          'ref_yaw',
          'width',
          'depth',
        ],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          levels: { title: 'Levels', type: 'array', items: { type: 'string' }, default: [] },
          doors: {
            title: 'Doors',
            type: 'array',
            items: { $ref: '#/components/schemas/Door' },
            default: [],
          },
          wall_graph: {
            title: 'Wall Graph',
            allOf: [{ $ref: '#/components/schemas/Graph' }],
            default: { name: '', vertices: [], edges: [], params: [] },
          },
          ref_x: { title: 'Ref X', type: 'number', default: 0 },
          ref_y: { title: 'Ref Y', type: 'number', default: 0 },
          ref_yaw: { title: 'Ref Yaw', type: 'number', default: 0 },
          width: { title: 'Width', type: 'number', default: 0 },
          depth: { title: 'Depth', type: 'number', default: 0 },
        },
      },
      LiftHealth: {
        title: 'LiftHealth',
        required: ['health_status', 'id_'],
        type: 'object',
        properties: {
          health_status: { title: 'Health Status', maxLength: 255, type: 'string', nullable: true },
          health_message: { title: 'Health Message', type: 'string', nullable: true },
          id_: { title: 'Id ', maxLength: 255, type: 'string' },
        },
        additionalProperties: false,
      },
      LiftRequest: {
        title: 'LiftRequest',
        required: ['request_type', 'door_mode', 'destination'],
        type: 'object',
        properties: {
          request_type: {
            title: 'Request Type',
            type: 'integer',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg',
          },
          door_mode: {
            title: 'Door Mode',
            type: 'integer',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg',
          },
          destination: { title: 'Destination', type: 'string' },
        },
      },
      LiftState: {
        title: 'LiftState',
        required: [
          'lift_time',
          'lift_name',
          'available_floors',
          'current_floor',
          'destination_floor',
          'door_state',
          'motion_state',
          'available_modes',
          'current_mode',
          'session_id',
        ],
        type: 'object',
        properties: {
          lift_time: {
            title: 'Lift Time',
            allOf: [{ $ref: '#/components/schemas/Time' }],
            default: { sec: 0, nanosec: 0 },
          },
          lift_name: { title: 'Lift Name', type: 'string', default: '' },
          available_floors: {
            title: 'Available Floors',
            type: 'array',
            items: { type: 'string' },
            default: [],
          },
          current_floor: { title: 'Current Floor', type: 'string', default: '' },
          destination_floor: { title: 'Destination Floor', type: 'string', default: '' },
          door_state: {
            title: 'Door State',
            maximum: 255.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          motion_state: {
            title: 'Motion State',
            maximum: 255.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          available_modes: { title: 'Available Modes', type: 'array', items: { type: 'integer' } },
          current_mode: {
            title: 'Current Mode',
            maximum: 255.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
          session_id: { title: 'Session Id', type: 'string', default: '' },
        },
      },
      Location2D: {
        title: 'Location2D',
        required: ['map', 'x', 'y', 'yaw'],
        type: 'object',
        properties: {
          map: { title: 'Map', type: 'string' },
          x: { title: 'X', type: 'number' },
          y: { title: 'Y', type: 'number' },
          yaw: { title: 'Yaw', type: 'number' },
        },
      },
      LogEntry: {
        title: 'LogEntry',
        required: ['seq', 'tier', 'unix_millis_time', 'text'],
        type: 'object',
        properties: {
          seq: {
            title: 'Seq',
            exclusiveMaximum: 4294967296.0,
            minimum: 0.0,
            type: 'integer',
            description:
              'Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.',
          },
          tier: {
            allOf: [{ $ref: '#/components/schemas/Tier' }],
            description: 'The importance level of the log entry',
          },
          unix_millis_time: { title: 'Unix Millis Time', type: 'integer' },
          text: { title: 'Text', type: 'string', description: 'The text of the log entry' },
        },
      },
      Param: {
        title: 'Param',
        required: ['name', 'type', 'value_int', 'value_float', 'value_string', 'value_bool'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          type: { title: 'Type', maximum: 4294967295.0, minimum: 0.0, type: 'integer', default: 0 },
          value_int: {
            title: 'Value Int',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            type: 'integer',
            default: 0,
          },
          value_float: { title: 'Value Float', type: 'number', default: 0 },
          value_string: { title: 'Value String', type: 'string', default: '' },
          value_bool: { title: 'Value Bool', type: 'boolean', default: false },
        },
      },
      Permission: {
        title: 'Permission',
        required: ['authz_grp', 'action'],
        type: 'object',
        properties: {
          authz_grp: { title: 'Authz Grp', type: 'string' },
          action: { title: 'Action', type: 'string' },
        },
      },
      Phase: {
        title: 'Phase',
        required: ['id'],
        type: 'object',
        properties: {
          id: { $ref: '#/components/schemas/Id' },
          category: { $ref: '#/components/schemas/Category' },
          detail: { $ref: '#/components/schemas/Detail' },
          unix_millis_start_time: { title: 'Unix Millis Start Time', type: 'integer' },
          unix_millis_finish_time: { title: 'Unix Millis Finish Time', type: 'integer' },
          original_estimate_millis: { $ref: '#/components/schemas/EstimateMillis' },
          estimate_millis: { $ref: '#/components/schemas/EstimateMillis' },
          final_event_id: { $ref: '#/components/schemas/Id' },
          events: {
            title: 'Events',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/EventState' },
            description:
              'A dictionary of events for this phase. The keys (property names) are the event IDs, which are integers.',
          },
          skip_requests: {
            title: 'Skip Requests',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/SkipPhaseRequest' },
            description: 'Information about any skip requests that have been received',
          },
        },
      },
      Phases: {
        title: 'Phases',
        type: 'object',
        properties: {
          log: {
            title: 'Log',
            type: 'array',
            items: { $ref: '#/components/schemas/LogEntry' },
            description: 'Log entries related to the overall phase',
          },
          events: {
            title: 'Events',
            type: 'object',
            additionalProperties: {
              type: 'array',
              items: { $ref: '#/components/schemas/LogEntry' },
            },
            description:
              'A dictionary whose keys (property names) are the indices of an event in the phase',
          },
        },
        additionalProperties: false,
      },
      Place: {
        title: 'Place',
        required: ['name', 'x', 'y', 'yaw', 'position_tolerance', 'yaw_tolerance'],
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string', default: '' },
          x: { title: 'X', type: 'number', default: 0 },
          y: { title: 'Y', type: 'number', default: 0 },
          yaw: { title: 'Yaw', type: 'number', default: 0 },
          position_tolerance: { title: 'Position Tolerance', type: 'number', default: 0 },
          yaw_tolerance: { title: 'Yaw Tolerance', type: 'number', default: 0 },
        },
      },
      PostMakeAdmin: {
        title: 'PostMakeAdmin',
        required: ['admin'],
        type: 'object',
        properties: { admin: { title: 'Admin', type: 'boolean' } },
      },
      PostRoles: {
        title: 'PostRoles',
        required: ['name'],
        type: 'object',
        properties: { name: { title: 'Name', type: 'string' } },
      },
      PostScheduledTaskRequest: {
        title: 'PostScheduledTaskRequest',
        required: ['task_request', 'schedules'],
        type: 'object',
        properties: {
          task_request: { $ref: '#/components/schemas/TaskRequest' },
          schedules: {
            title: 'Schedules',
            type: 'array',
            items: {
              $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTaskSchedule.leaf',
            },
          },
        },
      },
      PostUsers: {
        title: 'PostUsers',
        required: ['username'],
        type: 'object',
        properties: {
          username: { title: 'Username', type: 'string' },
          is_admin: { title: 'Is Admin', type: 'boolean', default: false },
        },
      },
      ResumedBy: {
        title: 'ResumedBy',
        required: ['labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the resume request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the resume request',
          },
        },
      },
      RobotState: {
        title: 'RobotState',
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string' },
          status: {
            allOf: [{ $ref: '#/components/schemas/Status2' }],
            description: 'A simple token representing the status of the robot',
          },
          task_id: {
            title: 'Task Id',
            type: 'string',
            description:
              'The ID of the task this robot is currently working on. Empty string if the robot is not working on a task.',
          },
          unix_millis_time: { title: 'Unix Millis Time', type: 'integer' },
          location: { $ref: '#/components/schemas/Location2D' },
          battery: {
            title: 'Battery',
            maximum: 1.0,
            minimum: 0.0,
            type: 'number',
            description:
              'State of charge of the battery. Values range from 0.0 (depleted) to 1.0 (fully charged)',
          },
          issues: {
            title: 'Issues',
            type: 'array',
            items: { $ref: '#/components/schemas/Issue' },
            description: 'A list of issues with the robot that operators need to address',
          },
        },
      },
      RobotTaskRequest: {
        title: 'RobotTaskRequest',
        required: ['type', 'robot', 'fleet', 'request'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            type: 'string',
            description: 'Indicate that this is a task dispatch request',
          },
          robot: { title: 'Robot', type: 'string', description: 'The name of the robot' },
          fleet: { title: 'Fleet', type: 'string', description: 'The fleet the robot belongs to' },
          request: { $ref: '#/components/schemas/TaskRequest' },
        },
      },
      RobotTaskResponse: {
        title: 'RobotTaskResponse',
        allOf: [{ $ref: '#/components/schemas/TaskDispatchResponse' }],
        description: 'Response to a robot task request',
      },
      SimpleResponse: {
        title: 'SimpleResponse',
        anyOf: [
          { $ref: '#/components/schemas/SimpleResponseItem' },
          { $ref: '#/components/schemas/SimpleResponseItem1' },
        ],
        description:
          'Template for defining a response message that only indicates success and describes any errors',
      },
      SimpleResponseItem: {
        title: 'SimpleResponseItem',
        required: ['success'],
        type: 'object',
        properties: {
          success: {
            $ref: '#/components/schemas/api_server__models__rmf_api__simple_response__Success',
          },
        },
      },
      SimpleResponseItem1: {
        title: 'SimpleResponseItem1',
        required: ['success', 'errors'],
        type: 'object',
        properties: {
          success: {
            $ref: '#/components/schemas/api_server__models__rmf_api__simple_response__Failure',
          },
          errors: {
            title: 'Errors',
            type: 'array',
            items: { $ref: '#/components/schemas/Error' },
            description: 'If the request failed, these error messages will explain why',
          },
        },
      },
      SkipPhaseRequest: {
        title: 'SkipPhaseRequest',
        required: ['unix_millis_request_time', 'labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the skip request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the skip request',
          },
          undo: {
            title: 'Undo',
            allOf: [{ $ref: '#/components/schemas/Undo' }],
            description: 'Information about an undo skip request that applied to this request',
          },
        },
      },
      SkipPhaseResponse: {
        title: 'SkipPhaseResponse',
        allOf: [{ $ref: '#/components/schemas/TokenResponse' }],
        description: 'Response to a request for a phase to be skipped',
      },
      Status: {
        title: 'Status',
        enum: [
          'uninitialized',
          'blocked',
          'error',
          'failed',
          'queued',
          'standby',
          'underway',
          'delayed',
          'skipped',
          'canceled',
          'killed',
          'completed',
        ],
        description: 'An enumeration.',
      },
      Status1: {
        title: 'Status1',
        enum: ['queued', 'selected', 'dispatched', 'failed_to_assign', 'canceled_in_flight'],
        description: 'An enumeration.',
      },
      Status2: {
        title: 'Status2',
        enum: ['uninitialized', 'offline', 'shutdown', 'idle', 'charging', 'working', 'error'],
        description: 'An enumeration.',
      },
      Task: {
        title: 'Task',
        required: ['category', 'detail'],
        type: 'object',
        properties: {
          category: {
            title: 'Category',
            type: 'string',
            description:
              'The category of this task. There must not be any duplicate task categories per fleet.',
          },
          detail: {
            title: 'Detail',
            type: 'string',
            description: 'Details about the behavior of the task.',
          },
          description_schema: {
            title: 'Description Schema',
            type: 'object',
            description: 'The schema for this task description',
          },
        },
      },
      TaskCancelResponse: {
        title: 'TaskCancelResponse',
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        description: 'Response to a request to cancel a task',
      },
      TaskDiscovery: {
        title: 'TaskDiscovery',
        required: ['type', 'data'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['task_discovery_update'],
            type: 'string',
            description: 'Indicate that this is an task discovery update',
          },
          data: { $ref: '#/components/schemas/Data' },
        },
      },
      TaskDiscoveryRequest: {
        title: 'TaskDiscoveryRequest',
        required: ['type'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['task_discovery_request'],
            type: 'string',
            description: 'Indicate that this is a task discovery request',
          },
        },
      },
      TaskDispatchResponse: {
        title: 'TaskDispatchResponse',
        anyOf: [
          { $ref: '#/components/schemas/TaskDispatchResponseItem' },
          { $ref: '#/components/schemas/TaskDispatchResponseItem1' },
        ],
        description: 'Response to a task dispatch request',
      },
      TaskDispatchResponseItem: {
        title: 'TaskDispatchResponseItem',
        required: ['success', 'state'],
        type: 'object',
        properties: {
          success: { title: 'Success', enum: [true], type: 'boolean' },
          state: { $ref: '#/components/schemas/TaskState' },
        },
      },
      TaskDispatchResponseItem1: {
        title: 'TaskDispatchResponseItem1',
        type: 'object',
        properties: {
          success: { title: 'Success', enum: [false], type: 'boolean' },
          errors: {
            title: 'Errors',
            type: 'array',
            items: { $ref: '#/components/schemas/Error' },
            description: 'Any error messages explaining why the request failed',
          },
        },
      },
      TaskEventLog: {
        title: 'TaskEventLog',
        required: ['task_id'],
        type: 'object',
        properties: {
          task_id: { title: 'Task Id', type: 'string' },
          log: {
            title: 'Log',
            type: 'array',
            items: { $ref: '#/components/schemas/LogEntry' },
            description: 'Log entries related to the overall task',
          },
          phases: {
            title: 'Phases',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/Phases' },
            description: 'A dictionary whose keys (property names) are the indices of a phase',
          },
        },
        additionalProperties: false,
      },
      TaskFavoritePydantic: {
        title: 'TaskFavoritePydantic',
        required: ['id', 'name', 'unix_millis_earliest_start_time', 'category', 'user'],
        type: 'object',
        properties: {
          id: { title: 'Id', type: 'string' },
          name: { title: 'Name', type: 'string' },
          unix_millis_earliest_start_time: {
            title: 'Unix Millis Earliest Start Time',
            type: 'integer',
          },
          priority: { title: 'Priority', type: 'object' },
          category: { title: 'Category', type: 'string' },
          description: { title: 'Description', type: 'object' },
          user: { title: 'User', type: 'string' },
        },
      },
      TaskInterruptionRequest: {
        title: 'TaskInterruptionRequest',
        required: ['type', 'task_id'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['interrupt_task_request'],
            type: 'string',
            description: 'Indicate that this is a task interruption request',
          },
          task_id: {
            title: 'Task Id',
            type: 'string',
            description: 'Specify the task ID to interrupt',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the interruption',
          },
        },
      },
      TaskInterruptionResponse: {
        title: 'TaskInterruptionResponse',
        allOf: [{ $ref: '#/components/schemas/TokenResponse' }],
        description: 'Response to a request for a task to be interrupted',
      },
      TaskKillRequest: {
        title: 'TaskKillRequest',
        required: ['type', 'task_id'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['kill_task_request'],
            type: 'string',
            description: 'Indicate that this is a task kill request',
          },
          task_id: { title: 'Task Id', type: 'string', description: 'Specify the task ID to kill' },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the kill',
          },
        },
      },
      TaskKillResponse: {
        title: 'TaskKillResponse',
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        description: 'Response to a request to kill a task',
      },
      TaskPhaseSkipRequest: {
        title: 'TaskPhaseSkipRequest',
        required: ['type', 'task_id', 'phase_id'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['skip_phase_request'],
            type: 'string',
            description: 'Indicate that this is a phase skip request',
          },
          task_id: {
            title: 'Task Id',
            type: 'string',
            description: 'Specify the task ID whose phase should be skipped',
          },
          phase_id: {
            title: 'Phase Id',
            minimum: 0.0,
            type: 'integer',
            description: 'Specify the phase that should be skipped',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the skip',
          },
        },
      },
      TaskRequest: {
        title: 'TaskRequest',
        required: ['category', 'description'],
        type: 'object',
        properties: {
          unix_millis_earliest_start_time: {
            title: 'Unix Millis Earliest Start Time',
            type: 'integer',
            description: '(Optional) The earliest time that this task may start',
          },
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: '(Optional) The time that this request was initiated',
          },
          priority: {
            title: 'Priority',
            type: 'object',
            description:
              '(Optional) The priority of this task. This must match a priority schema supported by a fleet.',
          },
          category: { title: 'Category', type: 'string' },
          description: {
            title: 'Description',
            description:
              'A description of the task. This must match a schema supported by a fleet for the category of this task request.',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the purpose of the task dispatch request',
          },
          requester: {
            title: 'Requester',
            type: 'string',
            description: '(Optional) An identifier for the entity that requested this task',
          },
        },
      },
      TaskResumeRequest: {
        title: 'TaskResumeRequest',
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['resume_task_request'],
            type: 'string',
            description: 'Indicate that this is a task resuming request',
          },
          for_task: {
            title: 'For Task',
            type: 'string',
            description: 'Specify task ID to resume.',
          },
          for_tokens: {
            title: 'For Tokens',
            minItems: 1,
            type: 'array',
            items: { type: 'string' },
            description:
              'A list of tokens of interruption requests which should be resumed. The interruption request associated with each token will be discarded.',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels describing this request',
          },
        },
      },
      TaskResumeResponse: {
        title: 'TaskResumeResponse',
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        description: 'Response to a request to resume a task',
      },
      TaskRewindRequest: {
        title: 'TaskRewindRequest',
        required: ['type', 'task_id', 'phase_id'],
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['rewind_task_request'],
            type: 'string',
            description: 'Indicate that this is a task rewind request',
          },
          task_id: {
            title: 'Task Id',
            type: 'string',
            description: 'Specify the ID of the task that should rewind',
          },
          phase_id: {
            title: 'Phase Id',
            minimum: 0.0,
            type: 'integer',
            description:
              'Specify the phase that should be rewound to. The task will restart at the beginning of this phase.',
          },
        },
      },
      TaskRewindResponse: {
        title: 'TaskRewindResponse',
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        description: 'Response to a request to rewind a task',
      },
      TaskState: {
        title: 'TaskState',
        required: ['booking'],
        type: 'object',
        properties: {
          booking: { $ref: '#/components/schemas/Booking' },
          category: { $ref: '#/components/schemas/Category' },
          detail: { $ref: '#/components/schemas/Detail' },
          unix_millis_start_time: { title: 'Unix Millis Start Time', type: 'integer' },
          unix_millis_finish_time: { title: 'Unix Millis Finish Time', type: 'integer' },
          original_estimate_millis: { $ref: '#/components/schemas/EstimateMillis' },
          estimate_millis: { $ref: '#/components/schemas/EstimateMillis' },
          assigned_to: {
            title: 'Assigned To',
            allOf: [{ $ref: '#/components/schemas/AssignedTo' }],
            description: 'Which agent (robot) is the task assigned to',
          },
          status: { $ref: '#/components/schemas/Status' },
          dispatch: { $ref: '#/components/schemas/Dispatch' },
          phases: {
            title: 'Phases',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/Phase' },
            description:
              'A dictionary of the states of the phases of the task. The keys (property names) are phase IDs, which are integers.',
          },
          completed: {
            title: 'Completed',
            type: 'array',
            items: { $ref: '#/components/schemas/Id' },
            description: 'An array of the IDs of completed phases of this task',
          },
          active: {
            title: 'Active',
            allOf: [{ $ref: '#/components/schemas/Id' }],
            description: 'The ID of the active phase for this task',
          },
          pending: {
            title: 'Pending',
            type: 'array',
            items: { $ref: '#/components/schemas/Id' },
            description: 'An array of the pending phases of this task',
          },
          interruptions: {
            title: 'Interruptions',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/Interruption' },
            description:
              'A dictionary of interruptions that have been applied to this task. The keys (property names) are the unique token of the interruption request.',
          },
          cancellation: {
            title: 'Cancellation',
            allOf: [{ $ref: '#/components/schemas/Cancellation' }],
            description:
              'If the task was cancelled, this will describe information about the request.',
          },
          killed: {
            title: 'Killed',
            allOf: [{ $ref: '#/components/schemas/Killed' }],
            description:
              'If the task was killed, this will describe information about the request.',
          },
        },
      },
      Tier: {
        title: 'Tier',
        enum: ['uninitialized', 'info', 'warning', 'error'],
        description: 'An enumeration.',
      },
      Time: {
        title: 'Time',
        required: ['sec', 'nanosec'],
        type: 'object',
        properties: {
          sec: {
            title: 'Sec',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            type: 'integer',
            default: 0,
          },
          nanosec: {
            title: 'Nanosec',
            maximum: 4294967295.0,
            minimum: 0.0,
            type: 'integer',
            default: 0,
          },
        },
      },
      TokenResponse: {
        title: 'TokenResponse',
        anyOf: [
          { $ref: '#/components/schemas/TokenResponseItem' },
          { $ref: '#/components/schemas/TokenResponseItem1' },
        ],
        description:
          'Template for defining a response message that provides a token upon success or errors upon failure',
      },
      TokenResponseItem: {
        title: 'TokenResponseItem',
        required: ['success', 'token'],
        type: 'object',
        properties: {
          success: {
            $ref: '#/components/schemas/api_server__models__rmf_api__token_response__Success',
          },
          token: {
            title: 'Token',
            type: 'string',
            description:
              'A token for the request. The value of this token is unique within the scope of this request and can be used by other requests to reference this request.',
          },
        },
      },
      TokenResponseItem1: {
        title: 'TokenResponseItem1',
        required: ['success', 'errors'],
        type: 'object',
        properties: {
          success: {
            $ref: '#/components/schemas/api_server__models__rmf_api__token_response__Failure',
          },
          errors: {
            title: 'Errors',
            type: 'array',
            items: { $ref: '#/components/schemas/Error' },
            description: 'Any error messages explaining why the request failed.',
          },
        },
      },
      Undo: {
        title: 'Undo',
        required: ['unix_millis_request_time', 'labels'],
        type: 'object',
        properties: {
          unix_millis_request_time: {
            title: 'Unix Millis Request Time',
            type: 'integer',
            description: 'The time that the undo skip request arrived',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels to describe the undo skip request',
          },
        },
      },
      UndoPhaseSkipRequest: {
        title: 'UndoPhaseSkipRequest',
        type: 'object',
        properties: {
          type: {
            title: 'Type',
            enum: ['undo_phase_skip_request'],
            type: 'string',
            description: 'Indicate that this is a request to undo a phase skip request',
          },
          for_task: {
            title: 'For Task',
            type: 'string',
            description: 'Specify the relevant task ID',
          },
          for_tokens: {
            title: 'For Tokens',
            minItems: 1,
            type: 'array',
            items: { type: 'string' },
            description:
              'A list of the tokens of skip requests which should be undone. The skips associated with each token will be discarded.',
          },
          labels: {
            title: 'Labels',
            type: 'array',
            items: { type: 'string' },
            description: 'Labels describing this request',
          },
        },
      },
      UndoPhaseSkipResponse: {
        title: 'UndoPhaseSkipResponse',
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        description: 'Response to an undo phase skip request',
      },
      User: {
        title: 'User',
        required: ['username', 'is_admin', 'roles'],
        type: 'object',
        properties: {
          username: { title: 'Username', type: 'string' },
          is_admin: { title: 'Is Admin', type: 'boolean', default: false },
          roles: { title: 'Roles', type: 'array', items: { type: 'string' }, default: [] },
        },
      },
      ValidationError: {
        title: 'ValidationError',
        required: ['loc', 'msg', 'type'],
        type: 'object',
        properties: {
          loc: {
            title: 'Location',
            type: 'array',
            items: { anyOf: [{ type: 'string' }, { type: 'integer' }] },
          },
          msg: { title: 'Message', type: 'string' },
          type: { title: 'Error Type', type: 'string' },
        },
      },
      'api_server.models.tortoise_models.alerts.Alert.leaf': {
        title: 'Alert',
        required: ['id', 'original_id', 'category', 'unix_millis_created_time'],
        type: 'object',
        properties: {
          id: { title: 'Id', maxLength: 255, type: 'string' },
          original_id: { title: 'Original Id', maxLength: 255, type: 'string' },
          category: {
            title: 'Category',
            maxLength: 7,
            type: 'string',
            description: 'Default: default<br/>Task: task<br/>Fleet: fleet<br/>Robot: robot',
          },
          unix_millis_created_time: {
            title: 'Unix Millis Created Time',
            maximum: 9.223372036854776e18,
            minimum: -9.223372036854776e18,
            type: 'integer',
          },
          acknowledged_by: {
            title: 'Acknowledged By',
            maxLength: 255,
            type: 'string',
            nullable: true,
          },
          unix_millis_acknowledged_time: {
            title: 'Unix Millis Acknowledged Time',
            maximum: 9.223372036854776e18,
            minimum: -9.223372036854776e18,
            type: 'integer',
            nullable: true,
          },
        },
        additionalProperties: false,
        description: 'General alert that can be triggered by events.',
      },
      'api_server.models.tortoise_models.beacons.BeaconState.leaf': {
        title: 'BeaconState',
        required: ['id', 'online', 'activated'],
        type: 'object',
        properties: {
          id: { title: 'Id', maxLength: 255, type: 'string' },
          online: { title: 'Online', type: 'boolean' },
          category: { title: 'Category', maxLength: 255, type: 'string', nullable: true },
          activated: { title: 'Activated', type: 'boolean' },
          level: { title: 'Level', maxLength: 255, type: 'string', nullable: true },
        },
        additionalProperties: false,
      },
      'api_server.models.tortoise_models.scheduled_task.ScheduledTask': {
        title: 'ScheduledTask',
        required: ['id', 'created_by', 'schedules'],
        type: 'object',
        properties: {
          id: { title: 'Id', maximum: 2147483647.0, minimum: 1.0, type: 'integer' },
          task_request: { title: 'Task Request' },
          created_by: { title: 'Created By', maxLength: 255, type: 'string' },
          last_ran: { title: 'Last Ran', type: 'string', format: 'date-time', nullable: true },
          except_dates: { title: 'Except Dates' },
          schedules: {
            title: 'Schedules',
            type: 'array',
            items: {
              $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTaskSchedule.leaf',
            },
          },
        },
        additionalProperties: false,
      },
      'api_server.models.tortoise_models.scheduled_task.ScheduledTaskSchedule.leaf': {
        title: 'ScheduledTaskSchedule',
        required: ['period'],
        type: 'object',
        properties: {
          every: {
            title: 'Every',
            maximum: 32767.0,
            minimum: -32768.0,
            type: 'integer',
            nullable: true,
          },
          start_from: { title: 'Start From', type: 'string', format: 'date-time', nullable: true },
          until: { title: 'Until', type: 'string', format: 'date-time', nullable: true },
          period: {
            title: 'Period',
            maxLength: 9,
            type: 'string',
            description:
              'Monday: monday<br/>Tuesday: tuesday<br/>Wednesday: wednesday<br/>Thursday: thursday<br/>Friday: friday<br/>Saturday: saturday<br/>Sunday: sunday<br/>Day: day<br/>Hour: hour<br/>Minute: minute',
          },
          at: { title: 'At', maxLength: 255, type: 'string', nullable: true },
        },
        additionalProperties: false,
        description:
          'The schedules for a scheduled task request.<br/>A scheduled task may have multiple schedules.',
      },
      'api_server.models.tortoise_models.scheduled_task.ScheduledTask_list': {
        title: 'ScheduledTask_list',
        type: 'array',
        items: {
          $ref: '#/components/schemas/api_server.models.tortoise_models.scheduled_task.ScheduledTask',
        },
      },
      'api_server.models.tortoise_models.tasks.TaskFavorite.leaf': {
        title: 'TaskFavorite',
        required: ['id', 'name', 'category', 'user'],
        type: 'object',
        properties: {
          id: { title: 'Id', maxLength: 255, type: 'string' },
          name: { title: 'Name', maxLength: 255, type: 'string' },
          unix_millis_earliest_start_time: {
            title: 'Unix Millis Earliest Start Time',
            type: 'string',
            format: 'date-time',
            nullable: true,
          },
          priority: { title: 'Priority' },
          category: { title: 'Category', maxLength: 255, type: 'string' },
          description: { title: 'Description' },
          user: { title: 'User', maxLength: 255, type: 'string' },
        },
        additionalProperties: false,
      },
      api_server__models__rmf_api__fleet_log__FleetState: {
        title: 'FleetState',
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string' },
          log: {
            title: 'Log',
            type: 'array',
            items: { $ref: '#/components/schemas/LogEntry' },
            description: 'Log for the overall fleet',
          },
          robots: {
            title: 'Robots',
            type: 'object',
            additionalProperties: {
              type: 'array',
              items: { $ref: '#/components/schemas/LogEntry' },
            },
            description:
              'Dictionary of logs for the individual robots. The keys (property names) are the robot names.',
          },
        },
      },
      api_server__models__rmf_api__fleet_state__FleetState: {
        title: 'FleetState',
        type: 'object',
        properties: {
          name: { title: 'Name', type: 'string' },
          robots: {
            title: 'Robots',
            type: 'object',
            additionalProperties: { $ref: '#/components/schemas/RobotState' },
            description: 'A dictionary of the states of the robots that belong to this fleet',
          },
        },
      },
      api_server__models__rmf_api__simple_response__Failure: {
        title: 'Failure',
        enum: [false],
        description: 'An enumeration.',
      },
      api_server__models__rmf_api__simple_response__Success: {
        title: 'Success',
        enum: [true],
        description: 'An enumeration.',
      },
      api_server__models__rmf_api__token_response__Failure: {
        title: 'Failure',
        enum: [false],
        description: 'An enumeration.',
      },
      api_server__models__rmf_api__token_response__Success: {
        title: 'Success',
        enum: [true],
        description: 'An enumeration.',
      },
    },
  },
};
