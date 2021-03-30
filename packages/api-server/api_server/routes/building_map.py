from fastapi import APIRouter, HTTPException

from ..models import BuildingMap
from ..rmf_io import RmfGateway


def building_map_router(rmf: RmfGateway):
    router = APIRouter()

    @router.get("", response_model=BuildingMap)
    async def _get_building_map():
        building_map = rmf.building_map.value
        if not building_map:
            raise HTTPException(503, "map unavailable")
        return building_map

    return router
