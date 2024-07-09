# This is a generated file, do not edit

from typing import List

import pydantic


class ApiService_Request(pydantic.BaseModel):
    json_msg: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "json_msg",
            ],
        }


#
# # The JSON message that represents the request
# string json_msg
#
