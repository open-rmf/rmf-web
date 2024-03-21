# This is a generated file, do not edit

from typing import Annotated

import pydantic


class ChargerCancel(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    charger_name: str  # string
    request_id: str  # string


# string charger_name  # the charger that should process this message
#
# # A unique ID for each request. It is advised that you prefix this
# # with the sender's node name. This is used for error tracking
# # later on
# string request_id
