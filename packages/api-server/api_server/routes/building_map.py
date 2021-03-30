import asyncio

from fastapi import APIRouter, HTTPException

from ..rmf_io import RmfGateway


def building_map_router(rmf: RmfGateway):
    router = APIRouter()

    @router.get("")
    async def _get_building_map():
        await asyncio.sleep(1)
        building_map = rmf.building_map.value
        if not building_map:
            raise HTTPException(503, "map unavailable")
        return building_map

    return router
