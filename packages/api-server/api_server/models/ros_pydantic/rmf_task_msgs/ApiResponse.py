# This is a generated file, do not edit

from typing import List

import pydantic


class ApiResponse(pydantic.BaseModel):
    type: pydantic.conint(ge=0, le=255) = 0  # uint8
    json_msg: str = ""  # string
    request_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "type",
                "json_msg",
                "request_id",
            ],
        }


#
# # This response type means the message was not initialized correctly and will
# # result in an error
# uint8 TYPE_UNINITIALIZED = 0
#
# # This response type means the request is being acknowledged which will grant it
# # some extra time before the API Node has its response timeout. This can be used
# # to extend the lifetime of a request which may take a long time to complete.
# # Each time an acknowledgment is sent the lifetime will be extended.
# uint8 TYPE_ACKNOWLEDGE = 1
#
# # This response type means this message is responding to the request and
# # therefore fulfilling the request.
# uint8 TYPE_RESPONDING = 2
#
# # The type of response this is: Acknowledging or Responding
# # (Uninitialized will result in the API Node issuing an error response)
# uint8 type
#
# # The JSON message that represents the response
# string json_msg
#
# # The unique ID of the request that this response is targeted at
# string request_id
