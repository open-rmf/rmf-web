# This is a generated file, do not edit

from typing import List

import pydantic


class Tow(pydantic.BaseModel):
    task_id: str = ""  # string
    object_type: str = ""  # string
    is_object_id_known: bool = False  # bool
    object_id: str = ""  # string
    pickup_place_name: str = ""  # string
    is_dropoff_place_known: bool = False  # bool
    dropoff_place_name: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "task_id",
                "object_type",
                "is_object_id_known",
                "object_id",
                "pickup_place_name",
                "is_dropoff_place_known",
                "dropoff_place_name",
            ],
        }


# # task_id is intended to be a pseudo-random string generated
# # by the caller which can be used to identify this task as it
# # moves between the queues to completion (or failure).
# string task_id
#
# string object_type
#
# bool is_object_id_known
# string object_id
#
# string pickup_place_name
#
# bool is_dropoff_place_known
# string dropoff_place_name
