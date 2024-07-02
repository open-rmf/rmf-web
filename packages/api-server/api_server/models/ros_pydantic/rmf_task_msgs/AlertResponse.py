# This is a generated file, do not edit

from typing import Annotated

import pydantic


class AlertResponse(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: str  # string
    response: str  # string


# # The unique ID of the Alert this response is for
# string id
#
# # This response must be one of the available options
# # in the Alert.
# string response
