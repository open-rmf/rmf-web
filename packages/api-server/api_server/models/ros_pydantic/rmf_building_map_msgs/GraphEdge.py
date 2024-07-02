# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphEdge(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    v1_idx: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32
    v2_idx: Annotated[int, pydantic.Field(ge=0, le=4294967295)]  # uint32
    params: list[Param]  # rmf_building_map_msgs/Param
    edge_type: Annotated[int, pydantic.Field(ge=0, le=255)]  # uint8


# uint32 v1_idx
# uint32 v2_idx
# Param[] params
#
# # when edge_type is UNIDIRECTIONAL, it means v1 -> v2
# # when edge_type is BIDIRECTIONAL, it means v1 <-> v2
# uint8 edge_type
# uint8 EDGE_TYPE_BIDIRECTIONAL=0
# uint8 EDGE_TYPE_UNIDIRECTIONAL=1
