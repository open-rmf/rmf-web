# This is a generated file, do not edit

from typing import Annotated

import pydantic


class LiftClearance_Response(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    decision: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32


#
#
# uint32 decision
# uint32 DECISION_CLEAR = 1
# uint32 DECISION_CROWDED = 2
