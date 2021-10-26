# This is a generated file, do not edit

from typing import List

import pydantic


class LiftClearance_Response(pydantic.BaseModel):
    decision: pydantic.conint(ge=0, le=4294967295)

    class Config:
        orm_mode = True

    def __init__(
        self,
        decision: pydantic.conint(ge=0, le=4294967295) = 0,  # uint32
        **kwargs,
    ):
        super().__init__(
            decision=decision,
            **kwargs,
        )


#
#
# uint32 decision
# uint32 DECISION_CLEAR = 1
# uint32 DECISION_CROWDED = 2
