# This is a generated file, do not edit

from typing import Annotated

import pydantic


class GetBuildingMap_Request(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    structure_needs_at_least_one_member: Annotated[int, pydantic.Field(ge=0, le=255)]
