from fastapi import APIRouter, HTTPException

from ..dependencies import ros
from ..models import BuildingMap

router = APIRouter(tags=["building"])


@router.get("", response_model=BuildingMap)
async def _get_building_map():
    building_map = ros.rmf_gateway.building_map.value
    if not building_map:
        raise HTTPException(503, "map unavailable")
    return building_map
