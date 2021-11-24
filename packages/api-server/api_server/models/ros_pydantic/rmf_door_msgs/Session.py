# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class Session(pydantic.BaseModel):
    request_time: Time = Time()  # builtin_interfaces/Time
    requester_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "request_time",
                "requester_id",
            ],
        }


#
# builtin_interfaces/Time request_time
# string requester_id
