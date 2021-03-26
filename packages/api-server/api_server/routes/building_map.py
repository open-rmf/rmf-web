from fastapi import APIRouter, HTTPException
from rosidl_runtime_py import message_to_ordereddict

from ..rmf_io import RmfGateway


def building_map_router(rmf: RmfGateway):
    router = APIRouter()

    @router.get("")
    async def _get_building_map():
        building_map = rmf.building_map.value
        if not building_map:
            raise HTTPException(503, "map unavailable")
        return message_to_ordereddict(building_map)

    return router
