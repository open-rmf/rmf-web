# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .Param import Param as rmf_building_map_msgs_msg_Param


class GraphEdge(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    v1_idx: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    v2_idx: Annotated[int, pydantic.Field(ge=0, le=4294967295)]
    params: list[rmf_building_map_msgs_msg_Param]
    edge_type: Annotated[int, pydantic.Field(ge=0, le=255)]
