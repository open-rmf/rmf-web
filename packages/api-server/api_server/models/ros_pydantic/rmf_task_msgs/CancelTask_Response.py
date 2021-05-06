# This is a generated file, do not edit

from typing import List

import pydantic


class CancelTask_Response(pydantic.BaseModel):
    success: bool = False  # bool
    message: str = ""  # string


#
#
# # Confirmation that this service call is processed
# bool success
#
# # This will provide a verbose message regarding task cancellation
# string message
#
