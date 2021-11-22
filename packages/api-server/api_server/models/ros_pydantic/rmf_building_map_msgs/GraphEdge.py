# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphEdge(pydantic.BaseModel):
    v1_idx: pydantic.conint(ge=0, le=4294967295) = 0  # uint32
    v2_idx: pydantic.conint(ge=0, le=4294967295) = 0  # uint32
    params: List[Param] = []  # rmf_building_map_msgs/Param
    edge_type: pydantic.conint(ge=0, le=255) = 0  # uint8

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "v1_idx",
                "v2_idx",
                "params",
                "edge_type",
            ],
        }


# uint32 v1_idx
# uint32 v2_idx
# Param[] params
#
# # when edge_type is UNIDIRECTIONAL, it means v1 -> v2
# # when edge_type is BIDIRECTIONAL, it means v1 <-> v2
# uint8 edge_type
# uint8 EDGE_TYPE_BIDIRECTIONAL=0
# uint8 EDGE_TYPE_UNIDIRECTIONAL=1
