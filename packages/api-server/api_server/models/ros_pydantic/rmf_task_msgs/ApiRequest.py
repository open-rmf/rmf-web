# This is a generated file, do not edit

from typing import List

import pydantic


class ApiRequest(pydantic.BaseModel):
    json_msg: str = ""  # string
    request_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "json_msg",
                "request_id",
            ],
        }


#
# # The JSON message that represents the request
# string json_msg
#
# # The unique ID assigned to this request
# string request_id
