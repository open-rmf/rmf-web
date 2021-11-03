# This is a generated file, do not edit

from typing import List

import pydantic


class LiftClearance_Response(pydantic.BaseModel):
    decision: pydantic.conint(ge=0, le=4294967295) = 0  # uint32

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "decision",
            ],
        }


#
#
# uint32 decision
# uint32 DECISION_CLEAR = 1
# uint32 DECISION_CROWDED = 2
