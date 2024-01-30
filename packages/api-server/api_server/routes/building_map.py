from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import BuildingMap
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["Building"])


@router.get("", response_model=BuildingMap)
async def get_building_map(rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    """
    Available in socket.io
    """
    building_map = await rmf_repo.get_bulding_map()
    if building_map is None:
        raise HTTPException(status_code=404)
    return building_map


@router.sub("", response_model=BuildingMap)
def sub_building_map(_req: SubscriptionRequest):
    return rmf_events.building_map.pipe(rxops.filter(lambda x: x is not None))
