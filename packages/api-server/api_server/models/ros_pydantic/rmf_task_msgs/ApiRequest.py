# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ApiRequest(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    json_msg: str  # string
    request_id: str  # string


#
# # The JSON message that represents the request
# string json_msg
#
# # The unique ID assigned to this request
# string request_id
