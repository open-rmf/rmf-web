# This is a generated file, do not edit

from typing import Annotated

import pydantic


class CancelTask_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    success: bool  # bool
    message: str  # string


#
#
# # Confirmation that this service call is processed
# bool success
#
# # This will provide a verbose message regarding task cancellation
# string message
#
