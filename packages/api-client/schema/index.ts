export default {
  openapi: '3.0.3',
  info: { title: 'RMF API Server', version: '0.1.0' },
  paths: {
    '/socket.io': {
      get: {
        summary: 'Socket.io endpoint',
        description:
          '# NOTE: This endpoint is here for documentation purposes only, this is _not_ a REST endpoint.\n\n## About\nThis exposes a minimal pubsub system built on top of socket.io.\nIt works similar to a normal socket.io endpoint, except that are 2 special\nrooms which control subscriptions.\n\n## Rooms\n### subscribe\nClients must send a message to this room to start receiving messages on other rooms.\nThe message must be of the form:\n\n```\n{\n    "room": "<room_name>"\n}\n```\n\n### unsubscribe\nClients can send a message to this room to stop receiving messages on other rooms.\nThe message must be of the form:\n\n```\n{\n    "room": "<room_name>"\n}\n```\n### /alerts\n\n\n```\n{\n  "additionalProperties": false,\n  "description": "General alert that can be triggered by events.",\n  "properties": {\n    "id": {\n      "maxLength": 255,\n      "title": "Id",\n      "type": "string"\n    },\n    "original_id": {\n      "maxLength": 255,\n      "title": "Original Id",\n      "type": "string"\n    },\n    "category": {\n      "description": "Default: default<br/>Task: task<br/>Fleet: fleet<br/>Robot: robot",\n      "maxLength": 7,\n      "title": "Category",\n      "type": "string"\n    },\n    "unix_millis_created_time": {\n      "maximum": 9223372036854775807,\n      "minimum": -9223372036854775808,\n      "title": "Unix Millis Created Time",\n      "type": "integer"\n    },\n    "acknowledged_by": {\n      "anyOf": [\n        {\n          "maxLength": 255,\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "nullable": true,\n      "title": "Acknowledged By"\n    },\n    "unix_millis_acknowledged_time": {\n      "anyOf": [\n        {\n          "maximum": 9223372036854775807,\n          "minimum": -9223372036854775808,\n          "type": "integer"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "nullable": true,\n      "title": "Unix Millis Acknowledged Time"\n    }\n  },\n  "required": [\n    "id",\n    "original_id",\n    "category",\n    "unix_millis_created_time",\n    "acknowledged_by",\n    "unix_millis_acknowledged_time"\n  ],\n  "title": "Alert",\n  "type": "object"\n}\n```\n\n\n### /beacons\n\n\n```\n{\n  "additionalProperties": false,\n  "properties": {\n    "id": {\n      "maxLength": 255,\n      "title": "Id",\n      "type": "string"\n    },\n    "online": {\n      "title": "Online",\n      "type": "boolean"\n    },\n    "category": {\n      "anyOf": [\n        {\n          "maxLength": 255,\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "nullable": true,\n      "title": "Category"\n    },\n    "activated": {\n      "title": "Activated",\n      "type": "boolean"\n    },\n    "level": {\n      "anyOf": [\n        {\n          "maxLength": 255,\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "nullable": true,\n      "title": "Level"\n    }\n  },\n  "required": [\n    "id",\n    "online",\n    "category",\n    "activated",\n    "level"\n  ],\n  "title": "BeaconState",\n  "type": "object"\n}\n```\n\n\n### /building_map\n\n\n```\n{\n  "$defs": {\n    "AffineImage": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "x_offset": {\n          "title": "X Offset",\n          "type": "number"\n        },\n        "y_offset": {\n          "title": "Y Offset",\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "type": "number"\n        },\n        "scale": {\n          "title": "Scale",\n          "type": "number"\n        },\n        "encoding": {\n          "title": "Encoding",\n          "type": "string"\n        },\n        "data": {\n          "title": "Data",\n          "type": "string"\n        }\n      },\n      "required": [\n        "name",\n        "x_offset",\n        "y_offset",\n        "yaw",\n        "scale",\n        "encoding",\n        "data"\n      ],\n      "title": "AffineImage",\n      "type": "object"\n    },\n    "Door": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "v1_x": {\n          "title": "V1 X",\n          "type": "number"\n        },\n        "v1_y": {\n          "title": "V1 Y",\n          "type": "number"\n        },\n        "v2_x": {\n          "title": "V2 X",\n          "type": "number"\n        },\n        "v2_y": {\n          "title": "V2 Y",\n          "type": "number"\n        },\n        "door_type": {\n          "maximum": 255,\n          "minimum": 0,\n          "title": "Door Type",\n          "type": "integer"\n        },\n        "motion_range": {\n          "title": "Motion Range",\n          "type": "number"\n        },\n        "motion_direction": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Motion Direction",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "name",\n        "v1_x",\n        "v1_y",\n        "v2_x",\n        "v2_y",\n        "door_type",\n        "motion_range",\n        "motion_direction"\n      ],\n      "title": "Door",\n      "type": "object"\n    },\n    "Graph": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "vertices": {\n          "items": {\n            "$ref": "#/$defs/GraphNode"\n          },\n          "title": "Vertices",\n          "type": "array"\n        },\n        "edges": {\n          "items": {\n            "$ref": "#/$defs/GraphEdge"\n          },\n          "title": "Edges",\n          "type": "array"\n        },\n        "params": {\n          "items": {\n            "$ref": "#/$defs/Param"\n          },\n          "title": "Params",\n          "type": "array"\n        }\n      },\n      "required": [\n        "name",\n        "vertices",\n        "edges",\n        "params"\n      ],\n      "title": "Graph",\n      "type": "object"\n    },\n    "GraphEdge": {\n      "properties": {\n        "v1_idx": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "V1 Idx",\n          "type": "integer"\n        },\n        "v2_idx": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "V2 Idx",\n          "type": "integer"\n        },\n        "params": {\n          "items": {\n            "$ref": "#/$defs/Param"\n          },\n          "title": "Params",\n          "type": "array"\n        },\n        "edge_type": {\n          "maximum": 255,\n          "minimum": 0,\n          "title": "Edge Type",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "v1_idx",\n        "v2_idx",\n        "params",\n        "edge_type"\n      ],\n      "title": "GraphEdge",\n      "type": "object"\n    },\n    "GraphNode": {\n      "properties": {\n        "x": {\n          "title": "X",\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "type": "number"\n        },\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "params": {\n          "items": {\n            "$ref": "#/$defs/Param"\n          },\n          "title": "Params",\n          "type": "array"\n        }\n      },\n      "required": [\n        "x",\n        "y",\n        "name",\n        "params"\n      ],\n      "title": "GraphNode",\n      "type": "object"\n    },\n    "Level": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "elevation": {\n          "title": "Elevation",\n          "type": "number"\n        },\n        "images": {\n          "items": {\n            "$ref": "#/$defs/AffineImage"\n          },\n          "title": "Images",\n          "type": "array"\n        },\n        "places": {\n          "items": {\n            "$ref": "#/$defs/Place"\n          },\n          "title": "Places",\n          "type": "array"\n        },\n        "doors": {\n          "items": {\n            "$ref": "#/$defs/Door"\n          },\n          "title": "Doors",\n          "type": "array"\n        },\n        "nav_graphs": {\n          "items": {\n            "$ref": "#/$defs/Graph"\n          },\n          "title": "Nav Graphs",\n          "type": "array"\n        },\n        "wall_graph": {\n          "$ref": "#/$defs/Graph"\n        }\n      },\n      "required": [\n        "name",\n        "elevation",\n        "images",\n        "places",\n        "doors",\n        "nav_graphs",\n        "wall_graph"\n      ],\n      "title": "Level",\n      "type": "object"\n    },\n    "Lift": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "levels": {\n          "items": {\n            "type": "string"\n          },\n          "title": "Levels",\n          "type": "array"\n        },\n        "doors": {\n          "items": {\n            "$ref": "#/$defs/Door"\n          },\n          "title": "Doors",\n          "type": "array"\n        },\n        "wall_graph": {\n          "$ref": "#/$defs/Graph"\n        },\n        "ref_x": {\n          "title": "Ref X",\n          "type": "number"\n        },\n        "ref_y": {\n          "title": "Ref Y",\n          "type": "number"\n        },\n        "ref_yaw": {\n          "title": "Ref Yaw",\n          "type": "number"\n        },\n        "width": {\n          "title": "Width",\n          "type": "number"\n        },\n        "depth": {\n          "title": "Depth",\n          "type": "number"\n        }\n      },\n      "required": [\n        "name",\n        "levels",\n        "doors",\n        "wall_graph",\n        "ref_x",\n        "ref_y",\n        "ref_yaw",\n        "width",\n        "depth"\n      ],\n      "title": "Lift",\n      "type": "object"\n    },\n    "Param": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "type": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Type",\n          "type": "integer"\n        },\n        "value_int": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Value Int",\n          "type": "integer"\n        },\n        "value_float": {\n          "title": "Value Float",\n          "type": "number"\n        },\n        "value_string": {\n          "title": "Value String",\n          "type": "string"\n        },\n        "value_bool": {\n          "title": "Value Bool",\n          "type": "boolean"\n        }\n      },\n      "required": [\n        "name",\n        "type",\n        "value_int",\n        "value_float",\n        "value_string",\n        "value_bool"\n      ],\n      "title": "Param",\n      "type": "object"\n    },\n    "Place": {\n      "properties": {\n        "name": {\n          "title": "Name",\n          "type": "string"\n        },\n        "x": {\n          "title": "X",\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "type": "number"\n        },\n        "position_tolerance": {\n          "title": "Position Tolerance",\n          "type": "number"\n        },\n        "yaw_tolerance": {\n          "title": "Yaw Tolerance",\n          "type": "number"\n        }\n      },\n      "required": [\n        "name",\n        "x",\n        "y",\n        "yaw",\n        "position_tolerance",\n        "yaw_tolerance"\n      ],\n      "title": "Place",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "name": {\n      "title": "Name",\n      "type": "string"\n    },\n    "levels": {\n      "items": {\n        "$ref": "#/$defs/Level"\n      },\n      "title": "Levels",\n      "type": "array"\n    },\n    "lifts": {\n      "items": {\n        "$ref": "#/$defs/Lift"\n      },\n      "title": "Lifts",\n      "type": "array"\n    }\n  },\n  "required": [\n    "name",\n    "levels",\n    "lifts"\n  ],\n  "title": "BuildingMap",\n  "type": "object"\n}\n```\n\n\n### /doors/{door_name}/state\n\n\n```\n{\n  "$defs": {\n    "DoorMode": {\n      "properties": {\n        "value": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Value",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "value"\n      ],\n      "title": "DoorMode",\n      "type": "object"\n    },\n    "Time": {\n      "properties": {\n        "sec": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Sec",\n          "type": "integer"\n        },\n        "nanosec": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Nanosec",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ],\n      "title": "Time",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "door_time": {\n      "$ref": "#/$defs/Time"\n    },\n    "door_name": {\n      "title": "Door Name",\n      "type": "string"\n    },\n    "current_mode": {\n      "$ref": "#/$defs/DoorMode"\n    }\n  },\n  "required": [\n    "door_time",\n    "door_name",\n    "current_mode"\n  ],\n  "title": "DoorState",\n  "type": "object"\n}\n```\n\n\n### /doors/{door_name}/health\n\n\n```\n{\n  "$defs": {\n    "HealthStatus": {\n      "enum": [\n        "Healthy",\n        "Unhealthy",\n        "Dead"\n      ],\n      "title": "HealthStatus",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "id_": {\n      "title": "Id ",\n      "type": "string"\n    },\n    "health_status": {\n      "$ref": "#/$defs/HealthStatus"\n    },\n    "health_message": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "title": "Health Message"\n    }\n  },\n  "required": [\n    "id_",\n    "health_status",\n    "health_message"\n  ],\n  "title": "DoorHealth",\n  "type": "object"\n}\n```\n\n\n### /lifts/{lift_name}/state\n\n\n```\n{\n  "$defs": {\n    "Time": {\n      "properties": {\n        "sec": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Sec",\n          "type": "integer"\n        },\n        "nanosec": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Nanosec",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ],\n      "title": "Time",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "lift_time": {\n      "$ref": "#/$defs/Time"\n    },\n    "lift_name": {\n      "title": "Lift Name",\n      "type": "string"\n    },\n    "available_floors": {\n      "items": {\n        "type": "string"\n      },\n      "title": "Available Floors",\n      "type": "array"\n    },\n    "current_floor": {\n      "title": "Current Floor",\n      "type": "string"\n    },\n    "destination_floor": {\n      "title": "Destination Floor",\n      "type": "string"\n    },\n    "door_state": {\n      "maximum": 255,\n      "minimum": 0,\n      "title": "Door State",\n      "type": "integer"\n    },\n    "motion_state": {\n      "maximum": 255,\n      "minimum": 0,\n      "title": "Motion State",\n      "type": "integer"\n    },\n    "available_modes": {\n      "items": {\n        "type": "integer"\n      },\n      "title": "Available Modes",\n      "type": "array"\n    },\n    "current_mode": {\n      "maximum": 255,\n      "minimum": 0,\n      "title": "Current Mode",\n      "type": "integer"\n    },\n    "session_id": {\n      "title": "Session Id",\n      "type": "string"\n    }\n  },\n  "required": [\n    "lift_time",\n    "lift_name",\n    "available_floors",\n    "current_floor",\n    "destination_floor",\n    "door_state",\n    "motion_state",\n    "available_modes",\n    "current_mode",\n    "session_id"\n  ],\n  "title": "LiftState",\n  "type": "object"\n}\n```\n\n\n### /lifts/{lift_name}/health\n\n\n```\n{\n  "$defs": {\n    "HealthStatus": {\n      "enum": [\n        "Healthy",\n        "Unhealthy",\n        "Dead"\n      ],\n      "title": "HealthStatus",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "id_": {\n      "title": "Id ",\n      "type": "string"\n    },\n    "health_status": {\n      "$ref": "#/$defs/HealthStatus"\n    },\n    "health_message": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "title": "Health Message"\n    }\n  },\n  "required": [\n    "id_",\n    "health_status",\n    "health_message"\n  ],\n  "title": "LiftHealth",\n  "type": "object"\n}\n```\n\n\n### /tasks/{task_id}/state\n\n\n```\n{\n  "$defs": {\n    "AssignedTo": {\n      "properties": {\n        "group": {\n          "title": "Group",\n          "type": "string"\n        },\n        "name": {\n          "title": "Name",\n          "type": "string"\n        }\n      },\n      "required": [\n        "group",\n        "name"\n      ],\n      "title": "AssignedTo",\n      "type": "object"\n    },\n    "Assignment": {\n      "properties": {\n        "fleet_name": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Fleet Name"\n        },\n        "expected_robot_name": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Expected Robot Name"\n        }\n      },\n      "title": "Assignment",\n      "type": "object"\n    },\n    "Booking": {\n      "properties": {\n        "id": {\n          "description": "The unique identifier for this task",\n          "title": "Id",\n          "type": "string"\n        },\n        "unix_millis_earliest_start_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Unix Millis Earliest Start Time"\n        },\n        "unix_millis_request_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Unix Millis Request Time"\n        },\n        "priority": {\n          "anyOf": [\n            {\n              "type": "object"\n            },\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Priority information about this task",\n          "title": "Priority"\n        },\n        "labels": {\n          "anyOf": [\n            {\n              "items": {\n                "type": "string"\n              },\n              "type": "array"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Information about how and why this task was booked",\n          "title": "Labels"\n        },\n        "requester": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "(Optional) An identifier for the entity that requested this task",\n          "title": "Requester"\n        }\n      },\n      "required": [\n        "id"\n      ],\n      "title": "Booking",\n      "type": "object"\n    },\n    "Cancellation": {\n      "properties": {\n        "unix_millis_request_time": {\n          "description": "The time that the cancellation request arrived",\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "labels": {\n          "description": "Labels to describe the cancel request",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ],\n      "title": "Cancellation",\n      "type": "object"\n    },\n    "Category": {\n      "title": "Category",\n      "type": "string"\n    },\n    "Detail": {\n      "anyOf": [\n        {\n          "type": "object"\n        },\n        {\n          "items": {},\n          "type": "array"\n        },\n        {\n          "type": "string"\n        }\n      ],\n      "title": "Detail"\n    },\n    "Dispatch": {\n      "properties": {\n        "status": {\n          "$ref": "#/$defs/Status2"\n        },\n        "assignment": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Assignment"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "errors": {\n          "anyOf": [\n            {\n              "items": {\n                "$ref": "#/$defs/Error"\n              },\n              "type": "array"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Errors"\n        }\n      },\n      "required": [\n        "status"\n      ],\n      "title": "Dispatch",\n      "type": "object"\n    },\n    "Error": {\n      "properties": {\n        "code": {\n          "anyOf": [\n            {\n              "minimum": 0,\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "A standard code for the kind of error that has occurred",\n          "title": "Code"\n        },\n        "category": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "The category of the error",\n          "title": "Category"\n        },\n        "detail": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Details about the error",\n          "title": "Detail"\n        }\n      },\n      "title": "Error",\n      "type": "object"\n    },\n    "EstimateMillis": {\n      "minimum": 0,\n      "title": "EstimateMillis",\n      "type": "integer"\n    },\n    "EventState": {\n      "properties": {\n        "id": {\n          "$ref": "#/$defs/Id"\n        },\n        "status": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Status"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "name": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "The brief name of the event",\n          "title": "Name"\n        },\n        "detail": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Detail"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Detailed information about the event"\n        },\n        "deps": {\n          "anyOf": [\n            {\n              "items": {\n                "minimum": 0,\n                "type": "integer"\n              },\n              "type": "array"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "This event may depend on other events. This array contains the IDs of those other event dependencies.",\n          "title": "Deps"\n        }\n      },\n      "required": [\n        "id"\n      ],\n      "title": "EventState",\n      "type": "object"\n    },\n    "Id": {\n      "minimum": 0,\n      "title": "Id",\n      "type": "integer"\n    },\n    "Interruption": {\n      "properties": {\n        "unix_millis_request_time": {\n          "description": "The time that the interruption request arrived",\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "labels": {\n          "description": "Labels to describe the purpose of the interruption",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        },\n        "resumed_by": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/ResumedBy"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Information about the resume request that ended this interruption. This field will be missing if the interruption is still active."\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ],\n      "title": "Interruption",\n      "type": "object"\n    },\n    "Killed": {\n      "properties": {\n        "unix_millis_request_time": {\n          "description": "The time that the cancellation request arrived",\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "labels": {\n          "description": "Labels to describe the kill request",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ],\n      "title": "Killed",\n      "type": "object"\n    },\n    "Phase": {\n      "properties": {\n        "id": {\n          "$ref": "#/$defs/Id"\n        },\n        "category": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Category"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "detail": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Detail"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "unix_millis_start_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Unix Millis Start Time"\n        },\n        "unix_millis_finish_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Unix Millis Finish Time"\n        },\n        "original_estimate_millis": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/EstimateMillis"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "estimate_millis": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/EstimateMillis"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "final_event_id": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Id"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "events": {\n          "anyOf": [\n            {\n              "additionalProperties": {\n                "$ref": "#/$defs/EventState"\n              },\n              "type": "object"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "A dictionary of events for this phase. The keys (property names) are the event IDs, which are integers.",\n          "title": "Events"\n        },\n        "skip_requests": {\n          "anyOf": [\n            {\n              "additionalProperties": {\n                "$ref": "#/$defs/SkipPhaseRequest"\n              },\n              "type": "object"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Information about any skip requests that have been received",\n          "title": "Skip Requests"\n        }\n      },\n      "required": [\n        "id"\n      ],\n      "title": "Phase",\n      "type": "object"\n    },\n    "ResumedBy": {\n      "properties": {\n        "unix_millis_request_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "The time that the resume request arrived",\n          "title": "Unix Millis Request Time"\n        },\n        "labels": {\n          "description": "Labels to describe the resume request",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        }\n      },\n      "required": [\n        "labels"\n      ],\n      "title": "ResumedBy",\n      "type": "object"\n    },\n    "SkipPhaseRequest": {\n      "properties": {\n        "unix_millis_request_time": {\n          "description": "The time that the skip request arrived",\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "labels": {\n          "description": "Labels to describe the purpose of the skip request",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        },\n        "undo": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Undo"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Information about an undo skip request that applied to this request"\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ],\n      "title": "SkipPhaseRequest",\n      "type": "object"\n    },\n    "Status": {\n      "enum": [\n        "uninitialized",\n        "blocked",\n        "error",\n        "failed",\n        "queued",\n        "standby",\n        "underway",\n        "delayed",\n        "skipped",\n        "canceled",\n        "killed",\n        "completed"\n      ],\n      "title": "Status",\n      "type": "string"\n    },\n    "Status2": {\n      "enum": [\n        "queued",\n        "selected",\n        "dispatched",\n        "failed_to_assign",\n        "canceled_in_flight"\n      ],\n      "title": "Status2",\n      "type": "string"\n    },\n    "Undo": {\n      "properties": {\n        "unix_millis_request_time": {\n          "description": "The time that the undo skip request arrived",\n          "title": "Unix Millis Request Time",\n          "type": "integer"\n        },\n        "labels": {\n          "description": "Labels to describe the undo skip request",\n          "items": {\n            "type": "string"\n          },\n          "title": "Labels",\n          "type": "array"\n        }\n      },\n      "required": [\n        "unix_millis_request_time",\n        "labels"\n      ],\n      "title": "Undo",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "booking": {\n      "$ref": "#/$defs/Booking"\n    },\n    "category": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Category"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "detail": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Detail"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "unix_millis_start_time": {\n      "anyOf": [\n        {\n          "type": "integer"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "title": "Unix Millis Start Time"\n    },\n    "unix_millis_finish_time": {\n      "anyOf": [\n        {\n          "type": "integer"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "title": "Unix Millis Finish Time"\n    },\n    "original_estimate_millis": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/EstimateMillis"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "estimate_millis": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/EstimateMillis"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "assigned_to": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/AssignedTo"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "Which agent (robot) is the task assigned to"\n    },\n    "status": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Status"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "dispatch": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Dispatch"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null\n    },\n    "phases": {\n      "anyOf": [\n        {\n          "additionalProperties": {\n            "$ref": "#/$defs/Phase"\n          },\n          "type": "object"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "A dictionary of the states of the phases of the task. The keys (property names) are phase IDs, which are integers.",\n      "title": "Phases"\n    },\n    "completed": {\n      "anyOf": [\n        {\n          "items": {\n            "$ref": "#/$defs/Id"\n          },\n          "type": "array"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "An array of the IDs of completed phases of this task",\n      "title": "Completed"\n    },\n    "active": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Id"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "The ID of the active phase for this task"\n    },\n    "pending": {\n      "anyOf": [\n        {\n          "items": {\n            "$ref": "#/$defs/Id"\n          },\n          "type": "array"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "An array of the pending phases of this task",\n      "title": "Pending"\n    },\n    "interruptions": {\n      "anyOf": [\n        {\n          "additionalProperties": {\n            "$ref": "#/$defs/Interruption"\n          },\n          "type": "object"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "A dictionary of interruptions that have been applied to this task. The keys (property names) are the unique token of the interruption request.",\n      "title": "Interruptions"\n    },\n    "cancellation": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Cancellation"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "If the task was cancelled, this will describe information about the request."\n    },\n    "killed": {\n      "anyOf": [\n        {\n          "$ref": "#/$defs/Killed"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "If the task was killed, this will describe information about the request."\n    }\n  },\n  "required": [\n    "booking"\n  ],\n  "title": "TaskState",\n  "type": "object"\n}\n```\n\n\n### /tasks/{task_id}/log\n\n\n```\n{\n  "$defs": {\n    "LogEntry": {\n      "properties": {\n        "seq": {\n          "description": "Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.",\n          "exclusiveMaximum": 4294967296,\n          "minimum": 0,\n          "title": "Seq",\n          "type": "integer"\n        },\n        "tier": {\n          "allOf": [\n            {\n              "$ref": "#/$defs/Tier"\n            }\n          ],\n          "description": "The importance level of the log entry"\n        },\n        "unix_millis_time": {\n          "title": "Unix Millis Time",\n          "type": "integer"\n        },\n        "text": {\n          "description": "The text of the log entry",\n          "title": "Text",\n          "type": "string"\n        }\n      },\n      "required": [\n        "seq",\n        "tier",\n        "unix_millis_time",\n        "text"\n      ],\n      "title": "LogEntry",\n      "type": "object"\n    },\n    "Phases": {\n      "additionalProperties": false,\n      "properties": {\n        "log": {\n          "anyOf": [\n            {\n              "items": {\n                "$ref": "#/$defs/LogEntry"\n              },\n              "type": "array"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Log entries related to the overall phase",\n          "title": "Log"\n        },\n        "events": {\n          "anyOf": [\n            {\n              "additionalProperties": {\n                "items": {\n                  "$ref": "#/$defs/LogEntry"\n                },\n                "type": "array"\n              },\n              "type": "object"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "A dictionary whose keys (property names) are the indices of an event in the phase",\n          "title": "Events"\n        }\n      },\n      "title": "Phases",\n      "type": "object"\n    },\n    "Tier": {\n      "enum": [\n        "uninitialized",\n        "info",\n        "warning",\n        "error"\n      ],\n      "title": "Tier",\n      "type": "string"\n    }\n  },\n  "additionalProperties": false,\n  "properties": {\n    "task_id": {\n      "title": "Task Id",\n      "type": "string"\n    },\n    "log": {\n      "anyOf": [\n        {\n          "items": {\n            "$ref": "#/$defs/LogEntry"\n          },\n          "type": "array"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "Log entries related to the overall task",\n      "title": "Log"\n    },\n    "phases": {\n      "anyOf": [\n        {\n          "additionalProperties": {\n            "$ref": "#/$defs/Phases"\n          },\n          "type": "object"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "A dictionary whose keys (property names) are the indices of a phase",\n      "title": "Phases"\n    }\n  },\n  "required": [\n    "task_id"\n  ],\n  "title": "TaskEventLog",\n  "type": "object"\n}\n```\n\n\n### /dispensers/{guid}/state\n\n\n```\n{\n  "$defs": {\n    "Time": {\n      "properties": {\n        "sec": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Sec",\n          "type": "integer"\n        },\n        "nanosec": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Nanosec",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ],\n      "title": "Time",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "time": {\n      "$ref": "#/$defs/Time"\n    },\n    "guid": {\n      "title": "Guid",\n      "type": "string"\n    },\n    "mode": {\n      "maximum": 2147483647,\n      "minimum": -2147483648,\n      "title": "Mode",\n      "type": "integer"\n    },\n    "request_guid_queue": {\n      "items": {\n        "type": "string"\n      },\n      "title": "Request Guid Queue",\n      "type": "array"\n    },\n    "seconds_remaining": {\n      "title": "Seconds Remaining",\n      "type": "number"\n    }\n  },\n  "required": [\n    "time",\n    "guid",\n    "mode",\n    "request_guid_queue",\n    "seconds_remaining"\n  ],\n  "title": "DispenserState",\n  "type": "object"\n}\n```\n\n\n### /dispensers/{guid}/health\n\n\n```\n{\n  "$defs": {\n    "HealthStatus": {\n      "enum": [\n        "Healthy",\n        "Unhealthy",\n        "Dead"\n      ],\n      "title": "HealthStatus",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "id_": {\n      "title": "Id ",\n      "type": "string"\n    },\n    "health_status": {\n      "$ref": "#/$defs/HealthStatus"\n    },\n    "health_message": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "title": "Health Message"\n    }\n  },\n  "required": [\n    "id_",\n    "health_status",\n    "health_message"\n  ],\n  "title": "DispenserHealth",\n  "type": "object"\n}\n```\n\n\n### /ingestors/{guid}/state\n\n\n```\n{\n  "$defs": {\n    "Time": {\n      "properties": {\n        "sec": {\n          "maximum": 2147483647,\n          "minimum": -2147483648,\n          "title": "Sec",\n          "type": "integer"\n        },\n        "nanosec": {\n          "maximum": 4294967295,\n          "minimum": 0,\n          "title": "Nanosec",\n          "type": "integer"\n        }\n      },\n      "required": [\n        "sec",\n        "nanosec"\n      ],\n      "title": "Time",\n      "type": "object"\n    }\n  },\n  "properties": {\n    "time": {\n      "$ref": "#/$defs/Time"\n    },\n    "guid": {\n      "title": "Guid",\n      "type": "string"\n    },\n    "mode": {\n      "maximum": 2147483647,\n      "minimum": -2147483648,\n      "title": "Mode",\n      "type": "integer"\n    },\n    "request_guid_queue": {\n      "items": {\n        "type": "string"\n      },\n      "title": "Request Guid Queue",\n      "type": "array"\n    },\n    "seconds_remaining": {\n      "title": "Seconds Remaining",\n      "type": "number"\n    }\n  },\n  "required": [\n    "time",\n    "guid",\n    "mode",\n    "request_guid_queue",\n    "seconds_remaining"\n  ],\n  "title": "IngestorState",\n  "type": "object"\n}\n```\n\n\n### /ingestors/{guid}/health\n\n\n```\n{\n  "$defs": {\n    "HealthStatus": {\n      "enum": [\n        "Healthy",\n        "Unhealthy",\n        "Dead"\n      ],\n      "title": "HealthStatus",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "id_": {\n      "title": "Id ",\n      "type": "string"\n    },\n    "health_status": {\n      "$ref": "#/$defs/HealthStatus"\n    },\n    "health_message": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "title": "Health Message"\n    }\n  },\n  "required": [\n    "id_",\n    "health_status",\n    "health_message"\n  ],\n  "title": "IngestorHealth",\n  "type": "object"\n}\n```\n\n\n### /fleets/{name}/state\n\n\n```\n{\n  "$defs": {\n    "Issue": {\n      "properties": {\n        "category": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Category of the robot\'s issue",\n          "title": "Category"\n        },\n        "detail": {\n          "anyOf": [\n            {\n              "type": "object"\n            },\n            {\n              "items": {},\n              "type": "array"\n            },\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "Detailed information about the issue",\n          "title": "Detail"\n        }\n      },\n      "title": "Issue",\n      "type": "object"\n    },\n    "Location2D": {\n      "properties": {\n        "map": {\n          "title": "Map",\n          "type": "string"\n        },\n        "x": {\n          "title": "X",\n          "type": "number"\n        },\n        "y": {\n          "title": "Y",\n          "type": "number"\n        },\n        "yaw": {\n          "title": "Yaw",\n          "type": "number"\n        }\n      },\n      "required": [\n        "map",\n        "x",\n        "y",\n        "yaw"\n      ],\n      "title": "Location2D",\n      "type": "object"\n    },\n    "RobotState": {\n      "properties": {\n        "name": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Name"\n        },\n        "status": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Status"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "A simple token representing the status of the robot"\n        },\n        "task_id": {\n          "anyOf": [\n            {\n              "type": "string"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "The ID of the task this robot is currently working on. Empty string if the robot is not working on a task.",\n          "title": "Task Id"\n        },\n        "unix_millis_time": {\n          "anyOf": [\n            {\n              "type": "integer"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "title": "Unix Millis Time"\n        },\n        "location": {\n          "anyOf": [\n            {\n              "$ref": "#/$defs/Location2D"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null\n        },\n        "battery": {\n          "anyOf": [\n            {\n              "maximum": 1.0,\n              "minimum": 0.0,\n              "type": "number"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "State of charge of the battery. Values range from 0.0 (depleted) to 1.0 (fully charged)",\n          "title": "Battery"\n        },\n        "issues": {\n          "anyOf": [\n            {\n              "items": {\n                "$ref": "#/$defs/Issue"\n              },\n              "type": "array"\n            },\n            {\n              "type": "null"\n            }\n          ],\n          "default": null,\n          "description": "A list of issues with the robot that operators need to address",\n          "title": "Issues"\n        }\n      },\n      "title": "RobotState",\n      "type": "object"\n    },\n    "Status": {\n      "enum": [\n        "uninitialized",\n        "offline",\n        "shutdown",\n        "idle",\n        "charging",\n        "working",\n        "error"\n      ],\n      "title": "Status",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "name": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "title": "Name"\n    },\n    "robots": {\n      "anyOf": [\n        {\n          "additionalProperties": {\n            "$ref": "#/$defs/RobotState"\n          },\n          "type": "object"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "A dictionary of the states of the robots that belong to this fleet",\n      "title": "Robots"\n    }\n  },\n  "title": "FleetState",\n  "type": "object"\n}\n```\n\n\n### /fleets/{name}/log\n\n\n```\n{\n  "$defs": {\n    "LogEntry": {\n      "properties": {\n        "seq": {\n          "description": "Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.",\n          "exclusiveMaximum": 4294967296,\n          "minimum": 0,\n          "title": "Seq",\n          "type": "integer"\n        },\n        "tier": {\n          "allOf": [\n            {\n              "$ref": "#/$defs/Tier"\n            }\n          ],\n          "description": "The importance level of the log entry"\n        },\n        "unix_millis_time": {\n          "title": "Unix Millis Time",\n          "type": "integer"\n        },\n        "text": {\n          "description": "The text of the log entry",\n          "title": "Text",\n          "type": "string"\n        }\n      },\n      "required": [\n        "seq",\n        "tier",\n        "unix_millis_time",\n        "text"\n      ],\n      "title": "LogEntry",\n      "type": "object"\n    },\n    "Tier": {\n      "enum": [\n        "uninitialized",\n        "info",\n        "warning",\n        "error"\n      ],\n      "title": "Tier",\n      "type": "string"\n    }\n  },\n  "properties": {\n    "name": {\n      "anyOf": [\n        {\n          "type": "string"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "title": "Name"\n    },\n    "log": {\n      "anyOf": [\n        {\n          "items": {\n            "$ref": "#/$defs/LogEntry"\n          },\n          "type": "array"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "Log for the overall fleet",\n      "title": "Log"\n    },\n    "robots": {\n      "anyOf": [\n        {\n          "additionalProperties": {\n            "items": {\n              "$ref": "#/$defs/LogEntry"\n            },\n            "type": "array"\n          },\n          "type": "object"\n        },\n        {\n          "type": "null"\n        }\n      ],\n      "default": null,\n      "description": "Dictionary of logs for the individual robots. The keys (property names) are the robot names.",\n      "title": "Robots"\n    }\n  },\n  "title": "FleetLog",\n  "type": "object"\n}\n```\n\n',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
    },
    '/permissions': {
      get: {
        summary: 'Get Effective Permissions',
        description: 'Get the effective permissions of the current user',
        operationId: 'get_effective_permissions_permissions_get',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Permission' },
                  title: 'Response Get Effective Permissions Permissions Get',
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
                schema: { type: 'integer', title: 'Response Get Time Time Get' },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: {
                    $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__alerts__Alert__leaf',
                  },
                  title: 'Response Get Alerts Alerts Get',
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
        summary: 'Create Alert',
        operationId: 'create_alert_alerts_post',
        parameters: [
          {
            name: 'alert_id',
            in: 'query',
            required: true,
            schema: { type: 'string', title: 'Alert Id' },
          },
          {
            name: 'category',
            in: 'query',
            required: true,
            schema: { type: 'string', title: 'Category' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__alerts__Alert__leaf',
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
            name: 'alert_id',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Alert Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__alerts__Alert__leaf',
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
            name: 'alert_id',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Alert Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__alerts__Alert__leaf',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: {
                    $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__beacons__BeaconState__leaf',
                  },
                  title: 'Response Get Beacons Beacons Get',
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
        tags: ['Beacons'],
        summary: 'Save Beacon State',
        operationId: 'save_beacon_state_beacons_post',
        parameters: [
          {
            name: 'beacon_id',
            in: 'query',
            required: true,
            schema: { type: 'string', title: 'Beacon Id' },
          },
          {
            name: 'online',
            in: 'query',
            required: true,
            schema: { type: 'boolean', title: 'Online' },
          },
          {
            name: 'category',
            in: 'query',
            required: true,
            schema: { type: 'string', title: 'Category' },
          },
          {
            name: 'activated',
            in: 'query',
            required: true,
            schema: { type: 'boolean', title: 'Activated' },
          },
          {
            name: 'level',
            in: 'query',
            required: true,
            schema: { type: 'string', title: 'Level' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__beacons__BeaconState__leaf',
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
            name: 'beacon_id',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Beacon Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  $ref: '#/components/schemas/tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__beacons__BeaconState__leaf',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/BuildingMap' } },
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
    '/doors': {
      get: {
        tags: ['Doors'],
        summary: 'Get Doors',
        operationId: 'get_doors_doors_get',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Door' },
                  title: 'Response Get Doors Doors Get',
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
    '/doors/{door_name}/state': {
      get: {
        tags: ['Doors'],
        summary: 'Get Door State',
        description: 'Available in socket.io',
        operationId: 'get_door_state_doors__door_name__state_get',
        parameters: [
          {
            name: 'door_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Door Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'door_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Door Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'door_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Door Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/DoorRequest' } } },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Lift' },
                  title: 'Response Get Lifts Lifts Get',
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
    '/lifts/{lift_name}/state': {
      get: {
        tags: ['Lifts'],
        summary: 'Get Lift State',
        description: 'Available in socket.io',
        operationId: 'get_lift_state_lifts__lift_name__state_get',
        parameters: [
          {
            name: 'lift_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Lift Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'lift_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Lift Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'lift_name',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Lift Name' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/LiftRequest' } } },
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
    '/tasks/{task_id}/request': {
      get: {
        tags: ['Tasks'],
        summary: 'Get Task Request',
        operationId: 'get_task_request_tasks__task_id__request_get',
        parameters: [
          {
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'string', description: 'task_id', title: 'Task Id' },
            description: 'task_id',
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskRequest' } },
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
    '/tasks': {
      get: {
        tags: ['Tasks'],
        summary: 'Query Task States',
        operationId: 'query_task_states_tasks_get',
        parameters: [
          {
            name: 'task_id',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description: 'comma separated list of task ids',
              title: 'Task Id',
            },
            description: 'comma separated list of task ids',
          },
          {
            name: 'category',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description: 'comma separated list of task categories',
              title: 'Category',
            },
            description: 'comma separated list of task categories',
          },
          {
            name: 'assigned_to',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description: 'comma separated list of assigned robot names',
              title: 'Assigned To',
            },
            description: 'comma separated list of assigned robot names',
          },
          {
            name: 'status',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description: 'comma separated list of statuses',
              title: 'Status',
            },
            description: 'comma separated list of statuses',
          },
          {
            name: 'label',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description:
                'comma separated list of labels, each item must be in the form <key>=<value>, multiple items will filter tasks with all the labels',
              title: 'Label',
            },
            description:
              'comma separated list of labels, each item must be in the form <key>=<value>, multiple items will filter tasks with all the labels',
          },
          {
            name: 'start_time_between',
            in: 'query',
            required: false,
            schema: {
              type: 'string',
              description:
                '\n        The period of starting time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n        ',
              title: 'Start Time Between',
            },
            description:
              '\n        The period of starting time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n        ',
          },
          {
            name: 'finish_time_between',
            in: 'query',
            required: false,
            schema: {
              type: 'string',
              description:
                '\n        The period of finishing time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
              title: 'Finish Time Between',
            },
            description:
              '\n        The period of finishing time to fetch, in unix millis.\n\n        This must be a comma separated string, \'X,Y\' to fetch between X millis and Y millis inclusive.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
          },
          {
            name: 'limit',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', maximum: 1000, exclusiveMinimum: 0 }, { type: 'null' }],
              description: 'defaults to 100',
              title: 'Limit',
            },
            description: 'defaults to 100',
          },
          {
            name: 'offset',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', minimum: 0 }, { type: 'null' }],
              description: 'defaults to 0',
              title: 'Offset',
            },
            description: 'defaults to 0',
          },
          {
            name: 'order_by',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
              title: 'Order By',
            },
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/TaskState' },
                  title: 'Response Query Task States Tasks Get',
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'string', description: 'task_id', title: 'Task Id' },
            description: 'task_id',
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'string', description: 'task_id', title: 'Task Id' },
            description: 'task_id',
          },
          {
            name: 'between',
            in: 'query',
            required: false,
            schema: {
              type: 'string',
              description:
                '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
              default: '-60000',
              title: 'Between',
            },
            description:
              '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/ActivityDiscoveryRequest' },
            },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/CancelTaskRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/DispatchTaskRequest' } },
          },
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskDispatchResponse' } },
            },
          },
          '400': {
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/TaskDispatchResponse' } },
            },
            description: 'Bad Request',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/RobotTaskRequest' } },
          },
        },
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/RobotTaskResponse' } },
            },
          },
          '400': {
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/RobotTaskResponse' } },
            },
            description: 'Bad Request',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/TaskInterruptionRequest' },
            },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskKillRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskResumeRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskRewindRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskPhaseSkipRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskDiscoveryRequest' } },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/UndoPhaseSkipRequest' } },
          },
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
      post: {
        tags: ['Tasks'],
        summary: 'Post Scheduled Task',
        description:
          'Create a scheduled task. Below are some examples of how the schedules are represented.\nFor more examples, check the docs of the underlying library used [here](https://github.com/dbader/schedule/blob/6eb0b5346b1ce35ece5050e65789fa6e44368175/docs/examples.rst).\n\n| every | to | period | at | description |\n| - | - | - | - | - |\n| 10 | - | minutes | - | Every 10 minutes |\n| - | - | hour | - | Every hour |\n| - | - | day | 10:30 | Every day at 10:30am |\n| - | - | monday | - | Every monday |\n| - | - | wednesday | 13:15 | Every wednesday at 01:15pm |\n| - | - | minute | :17 | Every 17th sec of a mintue |\n| 5 | 10 | seconds | - | Every 5-10 seconds (randomly) |',
        operationId: 'post_scheduled_task_scheduled_tasks_post',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/PostScheduledTaskRequest' },
            },
          },
        },
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/ScheduledTask' } },
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
      get: {
        tags: ['Tasks'],
        summary: 'Get Scheduled Tasks',
        operationId: 'get_scheduled_tasks_scheduled_tasks_get',
        parameters: [
          {
            name: 'start_before',
            in: 'query',
            required: true,
            schema: {
              type: 'string',
              format: 'date-time',
              description: 'Only return scheduled tasks that start before given timestamp',
              title: 'Start Before',
            },
            description: 'Only return scheduled tasks that start before given timestamp',
          },
          {
            name: 'until_after',
            in: 'query',
            required: true,
            schema: {
              type: 'string',
              format: 'date-time',
              description: 'Only return scheduled tasks that stop after given timestamp',
              title: 'Until After',
            },
            description: 'Only return scheduled tasks that stop after given timestamp',
          },
          {
            name: 'limit',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', maximum: 1000, exclusiveMinimum: 0 }, { type: 'null' }],
              description: 'defaults to 100',
              title: 'Limit',
            },
            description: 'defaults to 100',
          },
          {
            name: 'offset',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', minimum: 0 }, { type: 'null' }],
              description: 'defaults to 0',
              title: 'Offset',
            },
            description: 'defaults to 0',
          },
          {
            name: 'order_by',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
              title: 'Order By',
            },
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/ScheduledTask' },
                  title: 'Response Get Scheduled Tasks Scheduled Tasks Get',
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'integer', title: 'Task Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/ScheduledTask' } },
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'integer', title: 'Task Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'integer', title: 'Task Id' },
          },
          {
            name: 'event_date',
            in: 'query',
            required: true,
            schema: { type: 'string', format: 'date-time', title: 'Event Date' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'task_id',
            in: 'path',
            required: true,
            schema: { type: 'integer', title: 'Task Id' },
          },
          {
            name: 'except_date',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string', format: 'date-time' }, { type: 'null' }],
              title: 'Except Date',
            },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: { $ref: '#/components/schemas/PostScheduledTaskRequest' },
            },
          },
        },
        responses: {
          '201': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/ScheduledTask' } },
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
      post: {
        tags: ['Tasks'],
        summary: 'Post Favorite Task',
        operationId: 'post_favorite_task_favorite_tasks_post',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/TaskFavoritePydantic' } },
          },
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
      get: {
        tags: ['Tasks'],
        summary: 'Get Favorites Tasks',
        operationId: 'get_favorites_tasks_favorite_tasks_get',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/TaskFavoritePydantic' },
                  title: 'Response Get Favorites Tasks Favorite Tasks Get',
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
            name: 'favorite_task_id',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Favorite Task Id' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Dispenser' },
                  title: 'Response Get Dispensers Dispensers Get',
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
    '/dispensers/{guid}/state': {
      get: {
        tags: ['Dispensers'],
        summary: 'Get Dispenser State',
        description: 'Available in socket.io',
        operationId: 'get_dispenser_state_dispensers__guid__state_get',
        parameters: [
          { name: 'guid', in: 'path', required: true, schema: { type: 'string', title: 'Guid' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
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
          { name: 'guid', in: 'path', required: true, schema: { type: 'string', title: 'Guid' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Ingestor' },
                  title: 'Response Get Ingestors Ingestors Get',
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
    '/ingestors/{guid}/state': {
      get: {
        tags: ['Ingestors'],
        summary: 'Get Ingestor State',
        description: 'Available in socket.io',
        operationId: 'get_ingestor_state_ingestors__guid__state_get',
        parameters: [
          { name: 'guid', in: 'path', required: true, schema: { type: 'string', title: 'Guid' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
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
          { name: 'guid', in: 'path', required: true, schema: { type: 'string', title: 'Guid' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/FleetState' },
                  title: 'Response Get Fleets Fleets Get',
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
    '/fleets/{name}/state': {
      get: {
        tags: ['Fleets'],
        summary: 'Get Fleet State',
        description: 'Available in socket.io',
        operationId: 'get_fleet_state_fleets__name__state_get',
        parameters: [
          { name: 'name', in: 'path', required: true, schema: { type: 'string', title: 'Name' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': { schema: { $ref: '#/components/schemas/FleetState' } },
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
          { name: 'name', in: 'path', required: true, schema: { type: 'string', title: 'Name' } },
          {
            name: 'between',
            in: 'query',
            required: false,
            schema: {
              type: 'string',
              description:
                '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
              default: '-60000',
              title: 'Between',
            },
            description:
              '\n        The period of time to fetch, in unix millis.\n\n        This can be either a comma separated string or a string prefixed with \'-\' to fetch the last X millis.\n\n        Example:\n            "1000,2000" - Fetches logs between unix millis 1000 and 2000.\n            "-60000" - Fetches logs in the last minute.\n        ',
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: { 'application/json': { schema: { $ref: '#/components/schemas/FleetLog' } } },
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
            name: 'username',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description: 'filters username that starts with the value',
              title: 'Username',
            },
            description: 'filters username that starts with the value',
          },
          {
            name: 'is_admin',
            in: 'query',
            required: false,
            schema: { anyOf: [{ type: 'boolean' }, { type: 'null' }], title: 'Is Admin' },
          },
          {
            name: 'limit',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', maximum: 1000, exclusiveMinimum: 0 }, { type: 'null' }],
              description: 'defaults to 100',
              title: 'Limit',
            },
            description: 'defaults to 100',
          },
          {
            name: 'offset',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'integer', minimum: 0 }, { type: 'null' }],
              description: 'defaults to 0',
              title: 'Offset',
            },
            description: 'defaults to 0',
          },
          {
            name: 'order_by',
            in: 'query',
            required: false,
            schema: {
              anyOf: [{ type: 'string' }, { type: 'null' }],
              description:
                "common separated list of fields to order by, prefix with '-' to sort descendingly.",
              title: 'Order By',
            },
            description:
              "common separated list of fields to order by, prefix with '-' to sort descendingly.",
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { type: 'string' },
                  title: 'Response Get Users Admin Users Get',
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
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostUsers' } } },
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
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': { schema: { $ref: '#/components/schemas/PostMakeAdmin' } },
          },
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
      post: {
        tags: ['Admin'],
        summary: 'Add User Role',
        description: 'Add role to a user',
        operationId: 'add_user_role_admin_users__username__roles_post',
        parameters: [
          {
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostRoles' } } },
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
      put: {
        tags: ['Admin'],
        summary: 'Set User Roles',
        description: 'Set the roles of a user',
        operationId: 'set_user_roles_admin_users__username__roles_put',
        parameters: [
          {
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: {
            'application/json': {
              schema: {
                type: 'array',
                items: { $ref: '#/components/schemas/PostRoles' },
                title: 'Body',
              },
            },
          },
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
            name: 'username',
            in: 'path',
            required: true,
            schema: { type: 'string', title: 'Username' },
          },
          { name: 'role', in: 'path', required: true, schema: { type: 'string', title: 'Role' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
    '/admin/roles': {
      get: {
        tags: ['Admin'],
        summary: 'Get Roles',
        description: 'Get all roles',
        operationId: 'get_roles_admin_roles_get',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { type: 'string' },
                  title: 'Response Get Roles Admin Roles Get',
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
        summary: 'Create Role',
        description: 'Create a new role',
        operationId: 'create_role_admin_roles_post',
        parameters: [
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/PostRoles' } } },
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
          { name: 'role', in: 'path', required: true, schema: { type: 'string', title: 'Role' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
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
    '/admin/roles/{role}/permissions': {
      get: {
        tags: ['Admin'],
        summary: 'Get Role Permissions',
        description: 'Get all permissions of a role',
        operationId: 'get_role_permissions_admin_roles__role__permissions_get',
        parameters: [
          { name: 'role', in: 'path', required: true, schema: { type: 'string', title: 'Role' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        responses: {
          '200': {
            description: 'Successful Response',
            content: {
              'application/json': {
                schema: {
                  type: 'array',
                  items: { $ref: '#/components/schemas/Permission' },
                  title: 'Response Get Role Permissions Admin Roles  Role  Permissions Get',
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
          { name: 'role', in: 'path', required: true, schema: { type: 'string', title: 'Role' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/Permission' } } },
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
          { name: 'role', in: 'path', required: true, schema: { type: 'string', title: 'Role' } },
          {
            name: 'authorization',
            in: 'header',
            required: false,
            schema: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Authorization' },
          },
        ],
        requestBody: {
          required: true,
          content: { 'application/json': { schema: { $ref: '#/components/schemas/Permission' } } },
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
        properties: {
          category: {
            type: 'string',
            title: 'Category',
            description:
              'The category of this activity. There must not be any duplicate activity categories per fleet.',
          },
          detail: {
            type: 'string',
            title: 'Detail',
            description: 'Details about the behavior of the activity.',
          },
          description_schema: {
            anyOf: [{ type: 'object' }, { type: 'null' }],
            title: 'Description Schema',
            description: 'The schema for this activity description',
          },
        },
        type: 'object',
        required: ['category', 'detail'],
        title: 'Activity',
      },
      ActivityDiscovery: {
        properties: {
          data: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Datum' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Data',
          },
        },
        type: 'object',
        title: 'ActivityDiscovery',
      },
      ActivityDiscoveryRequest: {
        properties: {
          type: {
            const: 'activitiy_discovery_request',
            title: 'Type',
            description: 'Indicate that this is an activity discovery request',
          },
        },
        type: 'object',
        required: ['type'],
        title: 'ActivityDiscoveryRequest',
      },
      AffineImage: {
        properties: {
          name: { type: 'string', title: 'Name' },
          x_offset: { type: 'number', title: 'X Offset' },
          y_offset: { type: 'number', title: 'Y Offset' },
          yaw: { type: 'number', title: 'Yaw' },
          scale: { type: 'number', title: 'Scale' },
          encoding: { type: 'string', title: 'Encoding' },
          data: { type: 'string', title: 'Data' },
        },
        type: 'object',
        required: ['name', 'x_offset', 'y_offset', 'yaw', 'scale', 'encoding', 'data'],
        title: 'AffineImage',
      },
      AssignedTo: {
        properties: {
          group: { type: 'string', title: 'Group' },
          name: { type: 'string', title: 'Name' },
        },
        type: 'object',
        required: ['group', 'name'],
        title: 'AssignedTo',
      },
      Assignment: {
        properties: {
          fleet_name: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Fleet Name' },
          expected_robot_name: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Expected Robot Name',
          },
        },
        type: 'object',
        title: 'Assignment',
      },
      Booking: {
        properties: {
          id: { type: 'string', title: 'Id', description: 'The unique identifier for this task' },
          unix_millis_earliest_start_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Earliest Start Time',
          },
          unix_millis_request_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Request Time',
          },
          priority: {
            anyOf: [{ type: 'object' }, { type: 'string' }, { type: 'null' }],
            title: 'Priority',
            description: 'Priority information about this task',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Information about how and why this task was booked',
          },
          requester: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Requester',
            description: '(Optional) An identifier for the entity that requested this task',
          },
        },
        type: 'object',
        required: ['id'],
        title: 'Booking',
      },
      BuildingMap: {
        properties: {
          name: { type: 'string', title: 'Name' },
          levels: { items: { $ref: '#/components/schemas/Level' }, type: 'array', title: 'Levels' },
          lifts: { items: { $ref: '#/components/schemas/Lift' }, type: 'array', title: 'Lifts' },
        },
        type: 'object',
        required: ['name', 'levels', 'lifts'],
        title: 'BuildingMap',
      },
      CancelTaskRequest: {
        properties: {
          type: {
            const: 'cancel_task_request',
            title: 'Type',
            description: 'Indicate that this is a task cancellation request',
          },
          task_id: {
            type: 'string',
            title: 'Task Id',
            description: 'Specify the task ID to cancel',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels to describe the purpose of the cancellation',
          },
        },
        type: 'object',
        required: ['type', 'task_id'],
        title: 'CancelTaskRequest',
      },
      Cancellation: {
        properties: {
          unix_millis_request_time: {
            type: 'integer',
            title: 'Unix Millis Request Time',
            description: 'The time that the cancellation request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the cancel request',
          },
        },
        type: 'object',
        required: ['unix_millis_request_time', 'labels'],
        title: 'Cancellation',
      },
      Category: { type: 'string', title: 'Category' },
      Data: {
        properties: {
          fleet_name: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Fleet Name',
            description: 'Name of the fleet that supports these tasks',
          },
          tasks: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Task' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Tasks',
            description: '(list:replace) List of tasks that the fleet supports',
          },
        },
        type: 'object',
        title: 'Data',
      },
      Datum: {
        properties: {
          fleet_name: {
            type: 'string',
            title: 'Fleet Name',
            description: 'Name of the fleet that supports these activities',
          },
          activities: {
            items: { $ref: '#/components/schemas/Activity' },
            type: 'array',
            title: 'Activities',
            description: 'List of activities that the fleet supports',
          },
        },
        type: 'object',
        required: ['fleet_name', 'activities'],
        title: 'Datum',
      },
      Detail: {
        anyOf: [{ type: 'object' }, { items: {}, type: 'array' }, { type: 'string' }],
        title: 'Detail',
      },
      Dispatch: {
        properties: {
          status: { $ref: '#/components/schemas/Status2' },
          assignment: { anyOf: [{ $ref: '#/components/schemas/Assignment' }, { type: 'null' }] },
          errors: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Error' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Errors',
          },
        },
        type: 'object',
        required: ['status'],
        title: 'Dispatch',
      },
      DispatchTaskRequest: {
        properties: {
          type: {
            const: 'dispatch_task_request',
            title: 'Type',
            description: 'Indicate that this is a task dispatch request',
          },
          request: { $ref: '#/components/schemas/TaskRequest' },
        },
        type: 'object',
        required: ['type', 'request'],
        title: 'DispatchTaskRequest',
      },
      Dispenser: {
        properties: { guid: { type: 'string', title: 'Guid' } },
        type: 'object',
        required: ['guid'],
        title: 'Dispenser',
      },
      DispenserHealth: {
        properties: {
          id_: { type: 'string', title: 'Id ' },
          health_status: { $ref: '#/components/schemas/HealthStatus' },
          health_message: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Health Message',
          },
        },
        type: 'object',
        required: ['id_', 'health_status', 'health_message'],
        title: 'DispenserHealth',
      },
      DispenserState: {
        properties: {
          time: { $ref: '#/components/schemas/Time' },
          guid: { type: 'string', title: 'Guid' },
          mode: { type: 'integer', maximum: 2147483647.0, minimum: -2147483648.0, title: 'Mode' },
          request_guid_queue: {
            items: { type: 'string' },
            type: 'array',
            title: 'Request Guid Queue',
          },
          seconds_remaining: { type: 'number', title: 'Seconds Remaining' },
        },
        type: 'object',
        required: ['time', 'guid', 'mode', 'request_guid_queue', 'seconds_remaining'],
        title: 'DispenserState',
      },
      Door: {
        properties: {
          name: { type: 'string', title: 'Name' },
          v1_x: { type: 'number', title: 'V1 X' },
          v1_y: { type: 'number', title: 'V1 Y' },
          v2_x: { type: 'number', title: 'V2 X' },
          v2_y: { type: 'number', title: 'V2 Y' },
          door_type: { type: 'integer', maximum: 255.0, minimum: 0.0, title: 'Door Type' },
          motion_range: { type: 'number', title: 'Motion Range' },
          motion_direction: {
            type: 'integer',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            title: 'Motion Direction',
          },
        },
        type: 'object',
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
        title: 'Door',
      },
      DoorHealth: {
        properties: {
          id_: { type: 'string', title: 'Id ' },
          health_status: { $ref: '#/components/schemas/HealthStatus' },
          health_message: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Health Message',
          },
        },
        type: 'object',
        required: ['id_', 'health_status', 'health_message'],
        title: 'DoorHealth',
      },
      DoorMode: {
        properties: {
          value: { type: 'integer', maximum: 4294967295.0, minimum: 0.0, title: 'Value' },
        },
        type: 'object',
        required: ['value'],
        title: 'DoorMode',
      },
      DoorRequest: {
        properties: {
          mode: {
            type: 'integer',
            title: 'Mode',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_door_msgs/msg/DoorMode.msg',
          },
        },
        type: 'object',
        required: ['mode'],
        title: 'DoorRequest',
      },
      DoorState: {
        properties: {
          door_time: { $ref: '#/components/schemas/Time' },
          door_name: { type: 'string', title: 'Door Name' },
          current_mode: { $ref: '#/components/schemas/DoorMode' },
        },
        type: 'object',
        required: ['door_time', 'door_name', 'current_mode'],
        title: 'DoorState',
      },
      Error: {
        properties: {
          code: {
            anyOf: [{ type: 'integer', minimum: 0.0 }, { type: 'null' }],
            title: 'Code',
            description: 'A standard code for the kind of error that has occurred',
          },
          category: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Category',
            description: 'The category of the error',
          },
          detail: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Detail',
            description: 'Details about the error',
          },
        },
        type: 'object',
        title: 'Error',
      },
      EstimateMillis: { type: 'integer', minimum: 0.0, title: 'EstimateMillis' },
      EventState: {
        properties: {
          id: { $ref: '#/components/schemas/Id' },
          status: {
            anyOf: [
              { $ref: '#/components/schemas/api_server__models__rmf_api__task_state__Status' },
              { type: 'null' },
            ],
          },
          name: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Name',
            description: 'The brief name of the event',
          },
          detail: {
            anyOf: [{ $ref: '#/components/schemas/Detail' }, { type: 'null' }],
            description: 'Detailed information about the event',
          },
          deps: {
            anyOf: [{ items: { type: 'integer', minimum: 0.0 }, type: 'array' }, { type: 'null' }],
            title: 'Deps',
            description:
              'This event may depend on other events. This array contains the IDs of those other event dependencies.',
          },
        },
        type: 'object',
        required: ['id'],
        title: 'EventState',
      },
      Failure: { const: false, title: 'Failure' },
      FleetLog: {
        properties: {
          name: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Name' },
          log: {
            anyOf: [
              { items: { $ref: '#/components/schemas/LogEntry' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Log',
            description: 'Log for the overall fleet',
          },
          robots: {
            anyOf: [
              {
                additionalProperties: {
                  items: { $ref: '#/components/schemas/LogEntry' },
                  type: 'array',
                },
                type: 'object',
              },
              { type: 'null' },
            ],
            title: 'Robots',
            description:
              'Dictionary of logs for the individual robots. The keys (property names) are the robot names.',
          },
        },
        type: 'object',
        title: 'FleetLog',
      },
      FleetState: {
        properties: {
          name: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Name' },
          robots: {
            anyOf: [
              { additionalProperties: { $ref: '#/components/schemas/RobotState' }, type: 'object' },
              { type: 'null' },
            ],
            title: 'Robots',
            description: 'A dictionary of the states of the robots that belong to this fleet',
          },
        },
        type: 'object',
        title: 'FleetState',
      },
      Graph: {
        properties: {
          name: { type: 'string', title: 'Name' },
          vertices: {
            items: { $ref: '#/components/schemas/GraphNode' },
            type: 'array',
            title: 'Vertices',
          },
          edges: {
            items: { $ref: '#/components/schemas/GraphEdge' },
            type: 'array',
            title: 'Edges',
          },
          params: { items: { $ref: '#/components/schemas/Param' }, type: 'array', title: 'Params' },
        },
        type: 'object',
        required: ['name', 'vertices', 'edges', 'params'],
        title: 'Graph',
      },
      GraphEdge: {
        properties: {
          v1_idx: { type: 'integer', maximum: 4294967295.0, minimum: 0.0, title: 'V1 Idx' },
          v2_idx: { type: 'integer', maximum: 4294967295.0, minimum: 0.0, title: 'V2 Idx' },
          params: { items: { $ref: '#/components/schemas/Param' }, type: 'array', title: 'Params' },
          edge_type: { type: 'integer', maximum: 255.0, minimum: 0.0, title: 'Edge Type' },
        },
        type: 'object',
        required: ['v1_idx', 'v2_idx', 'params', 'edge_type'],
        title: 'GraphEdge',
      },
      GraphNode: {
        properties: {
          x: { type: 'number', title: 'X' },
          y: { type: 'number', title: 'Y' },
          name: { type: 'string', title: 'Name' },
          params: { items: { $ref: '#/components/schemas/Param' }, type: 'array', title: 'Params' },
        },
        type: 'object',
        required: ['x', 'y', 'name', 'params'],
        title: 'GraphNode',
      },
      HTTPValidationError: {
        properties: {
          detail: {
            items: { $ref: '#/components/schemas/ValidationError' },
            type: 'array',
            title: 'Detail',
          },
        },
        type: 'object',
        title: 'HTTPValidationError',
      },
      HealthStatus: {
        type: 'string',
        enum: ['Healthy', 'Unhealthy', 'Dead'],
        title: 'HealthStatus',
      },
      Id: { type: 'integer', minimum: 0.0, title: 'Id' },
      Ingestor: {
        properties: { guid: { type: 'string', title: 'Guid' } },
        type: 'object',
        required: ['guid'],
        title: 'Ingestor',
      },
      IngestorHealth: {
        properties: {
          id_: { type: 'string', title: 'Id ' },
          health_status: { $ref: '#/components/schemas/HealthStatus' },
          health_message: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Health Message',
          },
        },
        type: 'object',
        required: ['id_', 'health_status', 'health_message'],
        title: 'IngestorHealth',
      },
      IngestorState: {
        properties: {
          time: { $ref: '#/components/schemas/Time' },
          guid: { type: 'string', title: 'Guid' },
          mode: { type: 'integer', maximum: 2147483647.0, minimum: -2147483648.0, title: 'Mode' },
          request_guid_queue: {
            items: { type: 'string' },
            type: 'array',
            title: 'Request Guid Queue',
          },
          seconds_remaining: { type: 'number', title: 'Seconds Remaining' },
        },
        type: 'object',
        required: ['time', 'guid', 'mode', 'request_guid_queue', 'seconds_remaining'],
        title: 'IngestorState',
      },
      Interruption: {
        properties: {
          unix_millis_request_time: {
            type: 'integer',
            title: 'Unix Millis Request Time',
            description: 'The time that the interruption request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the purpose of the interruption',
          },
          resumed_by: {
            anyOf: [{ $ref: '#/components/schemas/ResumedBy' }, { type: 'null' }],
            description:
              'Information about the resume request that ended this interruption. This field will be missing if the interruption is still active.',
          },
        },
        type: 'object',
        required: ['unix_millis_request_time', 'labels'],
        title: 'Interruption',
      },
      Issue: {
        properties: {
          category: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Category',
            description: "Category of the robot's issue",
          },
          detail: {
            anyOf: [
              { type: 'object' },
              { items: {}, type: 'array' },
              { type: 'string' },
              { type: 'null' },
            ],
            title: 'Detail',
            description: 'Detailed information about the issue',
          },
        },
        type: 'object',
        title: 'Issue',
      },
      Killed: {
        properties: {
          unix_millis_request_time: {
            type: 'integer',
            title: 'Unix Millis Request Time',
            description: 'The time that the cancellation request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the kill request',
          },
        },
        type: 'object',
        required: ['unix_millis_request_time', 'labels'],
        title: 'Killed',
      },
      Level: {
        properties: {
          name: { type: 'string', title: 'Name' },
          elevation: { type: 'number', title: 'Elevation' },
          images: {
            items: { $ref: '#/components/schemas/AffineImage' },
            type: 'array',
            title: 'Images',
          },
          places: { items: { $ref: '#/components/schemas/Place' }, type: 'array', title: 'Places' },
          doors: { items: { $ref: '#/components/schemas/Door' }, type: 'array', title: 'Doors' },
          nav_graphs: {
            items: { $ref: '#/components/schemas/Graph' },
            type: 'array',
            title: 'Nav Graphs',
          },
          wall_graph: { $ref: '#/components/schemas/Graph' },
        },
        type: 'object',
        required: ['name', 'elevation', 'images', 'places', 'doors', 'nav_graphs', 'wall_graph'],
        title: 'Level',
      },
      Lift: {
        properties: {
          name: { type: 'string', title: 'Name' },
          levels: { items: { type: 'string' }, type: 'array', title: 'Levels' },
          doors: { items: { $ref: '#/components/schemas/Door' }, type: 'array', title: 'Doors' },
          wall_graph: { $ref: '#/components/schemas/Graph' },
          ref_x: { type: 'number', title: 'Ref X' },
          ref_y: { type: 'number', title: 'Ref Y' },
          ref_yaw: { type: 'number', title: 'Ref Yaw' },
          width: { type: 'number', title: 'Width' },
          depth: { type: 'number', title: 'Depth' },
        },
        type: 'object',
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
        title: 'Lift',
      },
      LiftHealth: {
        properties: {
          id_: { type: 'string', title: 'Id ' },
          health_status: { $ref: '#/components/schemas/HealthStatus' },
          health_message: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Health Message',
          },
        },
        type: 'object',
        required: ['id_', 'health_status', 'health_message'],
        title: 'LiftHealth',
      },
      LiftRequest: {
        properties: {
          request_type: {
            type: 'integer',
            title: 'Request Type',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg',
          },
          door_mode: {
            type: 'integer',
            title: 'Door Mode',
            description:
              'https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg',
          },
          destination: { type: 'string', title: 'Destination' },
        },
        type: 'object',
        required: ['request_type', 'door_mode', 'destination'],
        title: 'LiftRequest',
      },
      LiftState: {
        properties: {
          lift_time: { $ref: '#/components/schemas/Time' },
          lift_name: { type: 'string', title: 'Lift Name' },
          available_floors: { items: { type: 'string' }, type: 'array', title: 'Available Floors' },
          current_floor: { type: 'string', title: 'Current Floor' },
          destination_floor: { type: 'string', title: 'Destination Floor' },
          door_state: { type: 'integer', maximum: 255.0, minimum: 0.0, title: 'Door State' },
          motion_state: { type: 'integer', maximum: 255.0, minimum: 0.0, title: 'Motion State' },
          available_modes: { items: { type: 'integer' }, type: 'array', title: 'Available Modes' },
          current_mode: { type: 'integer', maximum: 255.0, minimum: 0.0, title: 'Current Mode' },
          session_id: { type: 'string', title: 'Session Id' },
        },
        type: 'object',
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
        title: 'LiftState',
      },
      Location2D: {
        properties: {
          map: { type: 'string', title: 'Map' },
          x: { type: 'number', title: 'X' },
          y: { type: 'number', title: 'Y' },
          yaw: { type: 'number', title: 'Yaw' },
        },
        type: 'object',
        required: ['map', 'x', 'y', 'yaw'],
        title: 'Location2D',
      },
      LogEntry: {
        properties: {
          seq: {
            type: 'integer',
            exclusiveMaximum: 4294967296.0,
            minimum: 0.0,
            title: 'Seq',
            description:
              'Sequence number for this entry. Each entry has a unique sequence number which monotonically increase, until integer overflow causes a wrap around.',
          },
          tier: {
            allOf: [{ $ref: '#/components/schemas/Tier' }],
            description: 'The importance level of the log entry',
          },
          unix_millis_time: { type: 'integer', title: 'Unix Millis Time' },
          text: { type: 'string', title: 'Text', description: 'The text of the log entry' },
        },
        type: 'object',
        required: ['seq', 'tier', 'unix_millis_time', 'text'],
        title: 'LogEntry',
      },
      Param: {
        properties: {
          name: { type: 'string', title: 'Name' },
          type: { type: 'integer', maximum: 4294967295.0, minimum: 0.0, title: 'Type' },
          value_int: {
            type: 'integer',
            maximum: 2147483647.0,
            minimum: -2147483648.0,
            title: 'Value Int',
          },
          value_float: { type: 'number', title: 'Value Float' },
          value_string: { type: 'string', title: 'Value String' },
          value_bool: { type: 'boolean', title: 'Value Bool' },
        },
        type: 'object',
        required: ['name', 'type', 'value_int', 'value_float', 'value_string', 'value_bool'],
        title: 'Param',
      },
      Period: {
        type: 'string',
        enum: [
          'monday',
          'tuesday',
          'wednesday',
          'thursday',
          'friday',
          'saturday',
          'sunday',
          'day',
          'hour',
          'minute',
        ],
        title: 'Period',
      },
      Permission: {
        properties: {
          authz_grp: { type: 'string', title: 'Authz Grp' },
          action: { type: 'string', title: 'Action' },
        },
        type: 'object',
        required: ['authz_grp', 'action'],
        title: 'Permission',
      },
      Phase: {
        properties: {
          id: { $ref: '#/components/schemas/Id' },
          category: { anyOf: [{ $ref: '#/components/schemas/Category' }, { type: 'null' }] },
          detail: { anyOf: [{ $ref: '#/components/schemas/Detail' }, { type: 'null' }] },
          unix_millis_start_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Start Time',
          },
          unix_millis_finish_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Finish Time',
          },
          original_estimate_millis: {
            anyOf: [{ $ref: '#/components/schemas/EstimateMillis' }, { type: 'null' }],
          },
          estimate_millis: {
            anyOf: [{ $ref: '#/components/schemas/EstimateMillis' }, { type: 'null' }],
          },
          final_event_id: { anyOf: [{ $ref: '#/components/schemas/Id' }, { type: 'null' }] },
          events: {
            anyOf: [
              { additionalProperties: { $ref: '#/components/schemas/EventState' }, type: 'object' },
              { type: 'null' },
            ],
            title: 'Events',
            description:
              'A dictionary of events for this phase. The keys (property names) are the event IDs, which are integers.',
          },
          skip_requests: {
            anyOf: [
              {
                additionalProperties: { $ref: '#/components/schemas/SkipPhaseRequest' },
                type: 'object',
              },
              { type: 'null' },
            ],
            title: 'Skip Requests',
            description: 'Information about any skip requests that have been received',
          },
        },
        type: 'object',
        required: ['id'],
        title: 'Phase',
      },
      Phases: {
        properties: {
          log: {
            anyOf: [
              { items: { $ref: '#/components/schemas/LogEntry' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Log',
            description: 'Log entries related to the overall phase',
          },
          events: {
            anyOf: [
              {
                additionalProperties: {
                  items: { $ref: '#/components/schemas/LogEntry' },
                  type: 'array',
                },
                type: 'object',
              },
              { type: 'null' },
            ],
            title: 'Events',
            description:
              'A dictionary whose keys (property names) are the indices of an event in the phase',
          },
        },
        additionalProperties: false,
        type: 'object',
        title: 'Phases',
      },
      Place: {
        properties: {
          name: { type: 'string', title: 'Name' },
          x: { type: 'number', title: 'X' },
          y: { type: 'number', title: 'Y' },
          yaw: { type: 'number', title: 'Yaw' },
          position_tolerance: { type: 'number', title: 'Position Tolerance' },
          yaw_tolerance: { type: 'number', title: 'Yaw Tolerance' },
        },
        type: 'object',
        required: ['name', 'x', 'y', 'yaw', 'position_tolerance', 'yaw_tolerance'],
        title: 'Place',
      },
      PostMakeAdmin: {
        properties: { admin: { type: 'boolean', title: 'Admin' } },
        type: 'object',
        required: ['admin'],
        title: 'PostMakeAdmin',
      },
      PostRoles: {
        properties: { name: { type: 'string', title: 'Name' } },
        type: 'object',
        required: ['name'],
        title: 'PostRoles',
      },
      PostScheduledTaskRequest: {
        properties: {
          task_request: { $ref: '#/components/schemas/TaskRequest' },
          schedules: {
            items: { $ref: '#/components/schemas/ScheduledTaskSchedule' },
            type: 'array',
            title: 'Schedules',
          },
        },
        type: 'object',
        required: ['task_request', 'schedules'],
        title: 'PostScheduledTaskRequest',
      },
      PostUsers: {
        properties: {
          username: { type: 'string', title: 'Username' },
          is_admin: { type: 'boolean', title: 'Is Admin', default: false },
        },
        type: 'object',
        required: ['username'],
        title: 'PostUsers',
      },
      ResumedBy: {
        properties: {
          unix_millis_request_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Request Time',
            description: 'The time that the resume request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the resume request',
          },
        },
        type: 'object',
        required: ['labels'],
        title: 'ResumedBy',
      },
      RobotState: {
        properties: {
          name: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'Name' },
          status: {
            anyOf: [
              { $ref: '#/components/schemas/api_server__models__rmf_api__robot_state__Status' },
              { type: 'null' },
            ],
            description: 'A simple token representing the status of the robot',
          },
          task_id: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Task Id',
            description:
              'The ID of the task this robot is currently working on. Empty string if the robot is not working on a task.',
          },
          unix_millis_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Time',
          },
          location: { anyOf: [{ $ref: '#/components/schemas/Location2D' }, { type: 'null' }] },
          battery: {
            anyOf: [{ type: 'number', maximum: 1.0, minimum: 0.0 }, { type: 'null' }],
            title: 'Battery',
            description:
              'State of charge of the battery. Values range from 0.0 (depleted) to 1.0 (fully charged)',
          },
          issues: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Issue' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Issues',
            description: 'A list of issues with the robot that operators need to address',
          },
        },
        type: 'object',
        title: 'RobotState',
      },
      RobotTaskRequest: {
        properties: {
          type: {
            type: 'string',
            title: 'Type',
            description: 'Indicate that this is a task dispatch request',
          },
          robot: { type: 'string', title: 'Robot', description: 'The name of the robot' },
          fleet: { type: 'string', title: 'Fleet', description: 'The fleet the robot belongs to' },
          request: { $ref: '#/components/schemas/TaskRequest' },
        },
        type: 'object',
        required: ['type', 'robot', 'fleet', 'request'],
        title: 'RobotTaskRequest',
      },
      RobotTaskResponse: {
        allOf: [{ $ref: '#/components/schemas/TaskDispatchResponse' }],
        title: 'RobotTaskResponse',
      },
      ScheduledTask: {
        properties: {
          id: { type: 'integer', title: 'Id' },
          task_request: { $ref: '#/components/schemas/TaskRequest' },
          created_by: { type: 'string', title: 'Created By' },
          schedules: {
            items: { $ref: '#/components/schemas/ScheduledTaskSchedule' },
            type: 'array',
            title: 'Schedules',
          },
          last_ran: {
            anyOf: [{ type: 'string', format: 'date-time' }, { type: 'null' }],
            title: 'Last Ran',
          },
          except_dates: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Except Dates',
          },
        },
        type: 'object',
        required: ['id', 'task_request', 'created_by', 'schedules'],
        title: 'ScheduledTask',
      },
      ScheduledTaskSchedule: {
        properties: {
          every: { anyOf: [{ type: 'integer' }, { type: 'null' }], title: 'Every' },
          start_from: {
            anyOf: [{ type: 'string', format: 'date-time' }, { type: 'null' }],
            title: 'Start From',
          },
          until: {
            anyOf: [{ type: 'string', format: 'date-time' }, { type: 'null' }],
            title: 'Until',
          },
          period: { $ref: '#/components/schemas/Period' },
          at: { anyOf: [{ type: 'string' }, { type: 'null' }], title: 'At' },
        },
        type: 'object',
        required: ['period'],
        title: 'ScheduledTaskSchedule',
      },
      SimpleResponse: {
        anyOf: [
          { $ref: '#/components/schemas/SimpleResponse1' },
          { $ref: '#/components/schemas/SimpleResponse2' },
        ],
        title: 'SimpleResponse',
      },
      SimpleResponse1: {
        properties: { success: { $ref: '#/components/schemas/Success' } },
        type: 'object',
        required: ['success'],
        title: 'SimpleResponse1',
      },
      SimpleResponse2: {
        properties: {
          success: { $ref: '#/components/schemas/Failure' },
          errors: {
            items: { $ref: '#/components/schemas/Error' },
            type: 'array',
            title: 'Errors',
            description: 'If the request failed, these error messages will explain why',
          },
        },
        type: 'object',
        required: ['success', 'errors'],
        title: 'SimpleResponse2',
      },
      SkipPhaseRequest: {
        properties: {
          unix_millis_request_time: {
            type: 'integer',
            title: 'Unix Millis Request Time',
            description: 'The time that the skip request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the purpose of the skip request',
          },
          undo: {
            anyOf: [{ $ref: '#/components/schemas/Undo' }, { type: 'null' }],
            description: 'Information about an undo skip request that applied to this request',
          },
        },
        type: 'object',
        required: ['unix_millis_request_time', 'labels'],
        title: 'SkipPhaseRequest',
      },
      SkipPhaseResponse: {
        allOf: [{ $ref: '#/components/schemas/TokenResponse' }],
        title: 'SkipPhaseResponse',
      },
      Status2: {
        type: 'string',
        enum: ['queued', 'selected', 'dispatched', 'failed_to_assign', 'canceled_in_flight'],
        title: 'Status2',
      },
      Success: { const: true, title: 'Success' },
      Task: {
        properties: {
          category: {
            type: 'string',
            title: 'Category',
            description:
              'The category of this task. There must not be any duplicate task categories per fleet.',
          },
          detail: {
            type: 'string',
            title: 'Detail',
            description: 'Details about the behavior of the task.',
          },
          description_schema: {
            anyOf: [{ type: 'object' }, { type: 'null' }],
            title: 'Description Schema',
            description: 'The schema for this task description',
          },
        },
        type: 'object',
        required: ['category', 'detail'],
        title: 'Task',
      },
      TaskCancelResponse: {
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        title: 'TaskCancelResponse',
      },
      TaskDiscovery: {
        properties: {
          type: {
            const: 'task_discovery_update',
            title: 'Type',
            description: 'Indicate that this is an task discovery update',
          },
          data: { $ref: '#/components/schemas/Data' },
        },
        type: 'object',
        required: ['type', 'data'],
        title: 'TaskDiscovery',
      },
      TaskDiscoveryRequest: {
        properties: {
          type: {
            const: 'task_discovery_request',
            title: 'Type',
            description: 'Indicate that this is a task discovery request',
          },
        },
        type: 'object',
        required: ['type'],
        title: 'TaskDiscoveryRequest',
      },
      TaskDispatchResponse: {
        anyOf: [
          { $ref: '#/components/schemas/TaskDispatchResponse1' },
          { $ref: '#/components/schemas/TaskDispatchResponse2' },
        ],
        title: 'TaskDispatchResponse',
      },
      TaskDispatchResponse1: {
        properties: {
          success: { const: true, title: 'Success' },
          state: { $ref: '#/components/schemas/TaskState' },
        },
        type: 'object',
        required: ['success', 'state'],
        title: 'TaskDispatchResponse1',
      },
      TaskDispatchResponse2: {
        properties: {
          success: { anyOf: [{ const: false }, { type: 'null' }], title: 'Success' },
          errors: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Error' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Errors',
            description: 'Any error messages explaining why the request failed',
          },
        },
        type: 'object',
        title: 'TaskDispatchResponse2',
      },
      TaskEventLog: {
        properties: {
          task_id: { type: 'string', title: 'Task Id' },
          log: {
            anyOf: [
              { items: { $ref: '#/components/schemas/LogEntry' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Log',
            description: 'Log entries related to the overall task',
          },
          phases: {
            anyOf: [
              { additionalProperties: { $ref: '#/components/schemas/Phases' }, type: 'object' },
              { type: 'null' },
            ],
            title: 'Phases',
            description: 'A dictionary whose keys (property names) are the indices of a phase',
          },
        },
        additionalProperties: false,
        type: 'object',
        required: ['task_id'],
        title: 'TaskEventLog',
      },
      TaskFavoritePydantic: {
        properties: {
          id: { type: 'string', title: 'Id' },
          name: { type: 'string', title: 'Name' },
          unix_millis_earliest_start_time: {
            type: 'integer',
            title: 'Unix Millis Earliest Start Time',
          },
          priority: { anyOf: [{ type: 'object' }, { type: 'null' }], title: 'Priority' },
          category: { type: 'string', title: 'Category' },
          description: { anyOf: [{ type: 'object' }, { type: 'null' }], title: 'Description' },
          user: { type: 'string', title: 'User' },
        },
        type: 'object',
        required: [
          'id',
          'name',
          'unix_millis_earliest_start_time',
          'priority',
          'category',
          'description',
          'user',
        ],
        title: 'TaskFavoritePydantic',
      },
      TaskInterruptionRequest: {
        properties: {
          type: {
            const: 'interrupt_task_request',
            title: 'Type',
            description: 'Indicate that this is a task interruption request',
          },
          task_id: {
            type: 'string',
            title: 'Task Id',
            description: 'Specify the task ID to interrupt',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels to describe the purpose of the interruption',
          },
        },
        type: 'object',
        required: ['type', 'task_id'],
        title: 'TaskInterruptionRequest',
      },
      TaskInterruptionResponse: {
        allOf: [{ $ref: '#/components/schemas/TokenResponse' }],
        title: 'TaskInterruptionResponse',
      },
      TaskKillRequest: {
        properties: {
          type: {
            const: 'kill_task_request',
            title: 'Type',
            description: 'Indicate that this is a task kill request',
          },
          task_id: { type: 'string', title: 'Task Id', description: 'Specify the task ID to kill' },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels to describe the purpose of the kill',
          },
        },
        type: 'object',
        required: ['type', 'task_id'],
        title: 'TaskKillRequest',
      },
      TaskKillResponse: {
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        title: 'TaskKillResponse',
      },
      TaskPhaseSkipRequest: {
        properties: {
          type: {
            const: 'skip_phase_request',
            title: 'Type',
            description: 'Indicate that this is a phase skip request',
          },
          task_id: {
            type: 'string',
            title: 'Task Id',
            description: 'Specify the task ID whose phase should be skipped',
          },
          phase_id: {
            type: 'integer',
            minimum: 0.0,
            title: 'Phase Id',
            description: 'Specify the phase that should be skipped',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels to describe the purpose of the skip',
          },
        },
        type: 'object',
        required: ['type', 'task_id', 'phase_id'],
        title: 'TaskPhaseSkipRequest',
      },
      TaskRequest: {
        properties: {
          unix_millis_earliest_start_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Earliest Start Time',
            description: '(Optional) The earliest time that this task may start',
          },
          unix_millis_request_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Request Time',
            description: '(Optional) The time that this request was initiated',
          },
          priority: {
            anyOf: [{ type: 'object' }, { type: 'null' }],
            title: 'Priority',
            description:
              '(Optional) The priority of this task. This must match a priority schema supported by a fleet.',
          },
          category: { type: 'string', title: 'Category' },
          description: {
            title: 'Description',
            description:
              'A description of the task. This must match a schema supported by a fleet for the category of this task request.',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels to describe the purpose of the task dispatch request',
          },
          requester: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Requester',
            description: '(Optional) An identifier for the entity that requested this task',
          },
          fleet_name: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'Fleet Name',
            description:
              '(Optional) The name of the fleet that should perform this task. If specified, other fleets will not bid for this task.',
          },
        },
        type: 'object',
        required: ['category', 'description'],
        title: 'TaskRequest',
      },
      TaskResumeRequest: {
        properties: {
          type: {
            anyOf: [{ const: 'resume_task_request' }, { type: 'null' }],
            title: 'Type',
            description: 'Indicate that this is a task resuming request',
          },
          for_task: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'For Task',
            description: 'Specify task ID to resume.',
          },
          for_tokens: {
            anyOf: [{ items: { type: 'string' }, type: 'array', minItems: 1 }, { type: 'null' }],
            title: 'For Tokens',
            description:
              'A list of tokens of interruption requests which should be resumed. The interruption request associated with each token will be discarded.',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels describing this request',
          },
        },
        type: 'object',
        title: 'TaskResumeRequest',
      },
      TaskResumeResponse: {
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        title: 'TaskResumeResponse',
      },
      TaskRewindRequest: {
        properties: {
          type: {
            const: 'rewind_task_request',
            title: 'Type',
            description: 'Indicate that this is a task rewind request',
          },
          task_id: {
            type: 'string',
            title: 'Task Id',
            description: 'Specify the ID of the task that should rewind',
          },
          phase_id: {
            type: 'integer',
            minimum: 0.0,
            title: 'Phase Id',
            description:
              'Specify the phase that should be rewound to. The task will restart at the beginning of this phase.',
          },
        },
        type: 'object',
        required: ['type', 'task_id', 'phase_id'],
        title: 'TaskRewindRequest',
      },
      TaskRewindResponse: {
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        title: 'TaskRewindResponse',
      },
      TaskState: {
        properties: {
          booking: { $ref: '#/components/schemas/Booking' },
          category: { anyOf: [{ $ref: '#/components/schemas/Category' }, { type: 'null' }] },
          detail: { anyOf: [{ $ref: '#/components/schemas/Detail' }, { type: 'null' }] },
          unix_millis_start_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Start Time',
          },
          unix_millis_finish_time: {
            anyOf: [{ type: 'integer' }, { type: 'null' }],
            title: 'Unix Millis Finish Time',
          },
          original_estimate_millis: {
            anyOf: [{ $ref: '#/components/schemas/EstimateMillis' }, { type: 'null' }],
          },
          estimate_millis: {
            anyOf: [{ $ref: '#/components/schemas/EstimateMillis' }, { type: 'null' }],
          },
          assigned_to: {
            anyOf: [{ $ref: '#/components/schemas/AssignedTo' }, { type: 'null' }],
            description: 'Which agent (robot) is the task assigned to',
          },
          status: {
            anyOf: [
              { $ref: '#/components/schemas/api_server__models__rmf_api__task_state__Status' },
              { type: 'null' },
            ],
          },
          dispatch: { anyOf: [{ $ref: '#/components/schemas/Dispatch' }, { type: 'null' }] },
          phases: {
            anyOf: [
              { additionalProperties: { $ref: '#/components/schemas/Phase' }, type: 'object' },
              { type: 'null' },
            ],
            title: 'Phases',
            description:
              'A dictionary of the states of the phases of the task. The keys (property names) are phase IDs, which are integers.',
          },
          completed: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Id' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Completed',
            description: 'An array of the IDs of completed phases of this task',
          },
          active: {
            anyOf: [{ $ref: '#/components/schemas/Id' }, { type: 'null' }],
            description: 'The ID of the active phase for this task',
          },
          pending: {
            anyOf: [
              { items: { $ref: '#/components/schemas/Id' }, type: 'array' },
              { type: 'null' },
            ],
            title: 'Pending',
            description: 'An array of the pending phases of this task',
          },
          interruptions: {
            anyOf: [
              {
                additionalProperties: { $ref: '#/components/schemas/Interruption' },
                type: 'object',
              },
              { type: 'null' },
            ],
            title: 'Interruptions',
            description:
              'A dictionary of interruptions that have been applied to this task. The keys (property names) are the unique token of the interruption request.',
          },
          cancellation: {
            anyOf: [{ $ref: '#/components/schemas/Cancellation' }, { type: 'null' }],
            description:
              'If the task was cancelled, this will describe information about the request.',
          },
          killed: {
            anyOf: [{ $ref: '#/components/schemas/Killed' }, { type: 'null' }],
            description:
              'If the task was killed, this will describe information about the request.',
          },
        },
        type: 'object',
        required: ['booking'],
        title: 'TaskState',
      },
      Tier: { type: 'string', enum: ['uninitialized', 'info', 'warning', 'error'], title: 'Tier' },
      Time: {
        properties: {
          sec: { type: 'integer', maximum: 2147483647.0, minimum: -2147483648.0, title: 'Sec' },
          nanosec: { type: 'integer', maximum: 4294967295.0, minimum: 0.0, title: 'Nanosec' },
        },
        type: 'object',
        required: ['sec', 'nanosec'],
        title: 'Time',
      },
      TokenResponse: {
        anyOf: [
          { $ref: '#/components/schemas/TokenResponse1' },
          { $ref: '#/components/schemas/TokenResponse2' },
        ],
        title: 'TokenResponse',
      },
      TokenResponse1: {
        properties: {
          success: { $ref: '#/components/schemas/Success' },
          token: {
            type: 'string',
            title: 'Token',
            description:
              'A token for the request. The value of this token is unique within the scope of this request and can be used by other requests to reference this request.',
          },
        },
        type: 'object',
        required: ['success', 'token'],
        title: 'TokenResponse1',
      },
      TokenResponse2: {
        properties: {
          success: { $ref: '#/components/schemas/Failure' },
          errors: {
            items: { $ref: '#/components/schemas/Error' },
            type: 'array',
            title: 'Errors',
            description: 'Any error messages explaining why the request failed.',
          },
        },
        type: 'object',
        required: ['success', 'errors'],
        title: 'TokenResponse2',
      },
      Undo: {
        properties: {
          unix_millis_request_time: {
            type: 'integer',
            title: 'Unix Millis Request Time',
            description: 'The time that the undo skip request arrived',
          },
          labels: {
            items: { type: 'string' },
            type: 'array',
            title: 'Labels',
            description: 'Labels to describe the undo skip request',
          },
        },
        type: 'object',
        required: ['unix_millis_request_time', 'labels'],
        title: 'Undo',
      },
      UndoPhaseSkipRequest: {
        properties: {
          type: {
            anyOf: [{ const: 'undo_phase_skip_request' }, { type: 'null' }],
            title: 'Type',
            description: 'Indicate that this is a request to undo a phase skip request',
          },
          for_task: {
            anyOf: [{ type: 'string' }, { type: 'null' }],
            title: 'For Task',
            description: 'Specify the relevant task ID',
          },
          for_tokens: {
            anyOf: [{ items: { type: 'string' }, type: 'array', minItems: 1 }, { type: 'null' }],
            title: 'For Tokens',
            description:
              'A list of the tokens of skip requests which should be undone. The skips associated with each token will be discarded.',
          },
          labels: {
            anyOf: [{ items: { type: 'string' }, type: 'array' }, { type: 'null' }],
            title: 'Labels',
            description: 'Labels describing this request',
          },
        },
        type: 'object',
        title: 'UndoPhaseSkipRequest',
      },
      UndoPhaseSkipResponse: {
        allOf: [{ $ref: '#/components/schemas/SimpleResponse' }],
        title: 'UndoPhaseSkipResponse',
      },
      User: {
        properties: {
          username: { type: 'string', title: 'Username' },
          is_admin: { type: 'boolean', title: 'Is Admin', default: false },
          roles: { items: { type: 'string' }, type: 'array', title: 'Roles', default: [] },
        },
        type: 'object',
        required: ['username', 'is_admin', 'roles'],
        title: 'User',
      },
      ValidationError: {
        properties: {
          loc: {
            items: { anyOf: [{ type: 'string' }, { type: 'integer' }] },
            type: 'array',
            title: 'Location',
          },
          msg: { type: 'string', title: 'Message' },
          type: { type: 'string', title: 'Error Type' },
        },
        type: 'object',
        required: ['loc', 'msg', 'type'],
        title: 'ValidationError',
      },
      api_server__models__rmf_api__robot_state__Status: {
        type: 'string',
        enum: ['uninitialized', 'offline', 'shutdown', 'idle', 'charging', 'working', 'error'],
        title: 'Status',
      },
      api_server__models__rmf_api__task_state__Status: {
        type: 'string',
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
        title: 'Status',
      },
      tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__alerts__Alert__leaf:
        {
          properties: {
            id: { type: 'string', maxLength: 255, title: 'Id' },
            original_id: { type: 'string', maxLength: 255, title: 'Original Id' },
            category: {
              type: 'string',
              maxLength: 7,
              title: 'Category',
              description: 'Default: default<br/>Task: task<br/>Fleet: fleet<br/>Robot: robot',
            },
            unix_millis_created_time: {
              type: 'integer',
              maximum: 9.223372036854776e18,
              minimum: -9.223372036854776e18,
              title: 'Unix Millis Created Time',
            },
            acknowledged_by: {
              anyOf: [{ type: 'string', maxLength: 255 }, { type: 'null' }],
              title: 'Acknowledged By',
              nullable: true,
            },
            unix_millis_acknowledged_time: {
              anyOf: [
                { type: 'integer', maximum: 9.223372036854776e18, minimum: -9.223372036854776e18 },
                { type: 'null' },
              ],
              title: 'Unix Millis Acknowledged Time',
              nullable: true,
            },
          },
          additionalProperties: false,
          type: 'object',
          required: [
            'id',
            'original_id',
            'category',
            'unix_millis_created_time',
            'acknowledged_by',
            'unix_millis_acknowledged_time',
          ],
          title: 'Alert',
          description: 'General alert that can be triggered by events.',
        },
      tortoise__contrib__pydantic__creator__api_server__models__tortoise_models__beacons__BeaconState__leaf:
        {
          properties: {
            id: { type: 'string', maxLength: 255, title: 'Id' },
            online: { type: 'boolean', title: 'Online' },
            category: {
              anyOf: [{ type: 'string', maxLength: 255 }, { type: 'null' }],
              title: 'Category',
              nullable: true,
            },
            activated: { type: 'boolean', title: 'Activated' },
            level: {
              anyOf: [{ type: 'string', maxLength: 255 }, { type: 'null' }],
              title: 'Level',
              nullable: true,
            },
          },
          additionalProperties: false,
          type: 'object',
          required: ['id', 'online', 'category', 'activated', 'level'],
          title: 'BeaconState',
        },
    },
  },
};
