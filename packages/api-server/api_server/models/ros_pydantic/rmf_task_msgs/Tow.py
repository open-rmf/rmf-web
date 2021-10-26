# This is a generated file, do not edit

from typing import List

import pydantic


class Tow(pydantic.BaseModel):
    task_id: str
    object_type: str
    is_object_id_known: bool
    object_id: str
    pickup_place_name: str
    is_dropoff_place_known: bool
    dropoff_place_name: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        task_id: str = "",  # string
        object_type: str = "",  # string
        is_object_id_known: bool = False,  # bool
        object_id: str = "",  # string
        pickup_place_name: str = "",  # string
        is_dropoff_place_known: bool = False,  # bool
        dropoff_place_name: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            task_id=task_id,
            object_type=object_type,
            is_object_id_known=is_object_id_known,
            object_id=object_id,
            pickup_place_name=pickup_place_name,
            is_dropoff_place_known=is_dropoff_place_known,
            dropoff_place_name=dropoff_place_name,
            **kwargs,
        )


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
