# This is a generated file, do not edit

from typing import List

import pydantic


class Station(pydantic.BaseModel):
    task_id: str = ""  # string
    robot_type: str = ""  # string
    place_name: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "task_id",
                "robot_type",
                "place_name",
            ],
        }


# # task_id is intended to be a pseudo-random string generated
# # by the caller which can be used to identify this task as it
# # moves between the queues to completion (or failure).
# string task_id
#
# # robot_type can be used to specify a particular robot fleet
# # for this request
# string robot_type
#
# # the place name where the robot is requested to station (park)
# string place_name
