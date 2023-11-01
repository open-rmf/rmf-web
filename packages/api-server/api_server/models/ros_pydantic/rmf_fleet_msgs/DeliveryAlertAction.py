# This is a generated file, do not edit

from typing import List

import pydantic


class DeliveryAlertAction(pydantic.BaseModel):
    value: pydantic.conint(ge=0, le=4294967295) = 0  # uint32

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "value",
            ],
        }


# uint32 value
# uint32 WAITING=0
# uint32 CANCEL=1
# uint32 OVERRIDE=2
# uint32 RESUME=3
