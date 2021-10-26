# This is a generated file, do not edit

from typing import List

import pydantic


class Station(pydantic.BaseModel):
    task_id: str
    robot_type: str
    place_name: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        task_id: str = "",  # string
        robot_type: str = "",  # string
        place_name: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            task_id=task_id,
            robot_type=robot_type,
            place_name=place_name,
            **kwargs,
        )


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
