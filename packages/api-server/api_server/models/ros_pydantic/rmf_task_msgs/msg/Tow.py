# This is a generated file, do not edit

from typing import Annotated

import pydantic


class Tow(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    object_type: str
    is_object_id_known: bool
    object_id: str
    pickup_place_name: str
    is_dropoff_place_known: bool
    dropoff_place_name: str
