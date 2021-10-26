# This is a generated file, do not edit

from typing import List

import pydantic


class ChargerCancel(pydantic.BaseModel):
    charger_name: str
    request_id: str

    class Config:
        orm_mode = True

    def __init__(
        self,
        charger_name: str = "",  # string
        request_id: str = "",  # string
        **kwargs,
    ):
        super().__init__(
            charger_name=charger_name,
            request_id=request_id,
            **kwargs,
        )


# string charger_name  # the charger that should process this message
#
# # A unique ID for each request. It is advised that you prefix this
# # with the sender's node name. This is used for error tracking
# # later on
# string request_id
