# This is a generated file, do not edit

from typing import List

import pydantic


class AlertResponse(pydantic.BaseModel):
    id: str = ""  # string
    response: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "id",
                "response",
            ],
        }


# # The unique ID of the Alert this response is for
# string id
#
# # This response must be one of the available options
# # in the Alert.
# string response
