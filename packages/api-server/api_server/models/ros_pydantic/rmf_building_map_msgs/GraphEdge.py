# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_building_map_msgs.Param import Param


class GraphEdge(pydantic.BaseModel):
    v1_idx: pydantic.conint(ge=0, le=4294967295)
    v2_idx: pydantic.conint(ge=0, le=4294967295)
    params: List[Param]
    edge_type: pydantic.conint(ge=0, le=255)

    class Config:
        orm_mode = True

    def __init__(
        self,
        v1_idx: int = 0,  # uint32
        v2_idx: int = 0,  # uint32
        params: List = None,  # rmf_building_map_msgs/Param
        edge_type: int = 0,  # uint8
        **kwargs,
    ):
        super().__init__(
            v1_idx=v1_idx,
            v2_idx=v2_idx,
            params=params or [],
            edge_type=edge_type,
            **kwargs,
        )


# uint32 v1_idx
# uint32 v2_idx
# Param[] params
#
# # when edge_type is UNIDIRECTIONAL, it means v1 -> v2
# # when edge_type is BIDIRECTIONAL, it means v1 <-> v2
# uint8 edge_type
# uint8 EDGE_TYPE_BIDIRECTIONAL=0
# uint8 EDGE_TYPE_UNIDIRECTIONAL=1
