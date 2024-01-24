from typing import List

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Ingestor, IngestorHealth, IngestorState
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import rmf_events

router = FastIORouter(tags=["Ingestors"])


@router.get("", response_model=List[Ingestor])
async def get_ingestors(rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    return await rmf_repo.get_ingestors()


@router.get("/{guid}/state", response_model=IngestorState)
async def get_ingestor_state(
    guid: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    ingestor_state = await rmf_repo.get_ingestor_state(guid)
    if ingestor_state is None:
        raise HTTPException(status_code=404)
    return ingestor_state


@router.sub("/{guid}/state", response_model=IngestorState)
async def sub_ingestor_state(req: SubscriptionRequest, guid: str):
    user = sio_user(req)
    obs = rmf_events.ingestor_states.pipe(rxops.filter(lambda x: x.guid == guid))
    ingestor_state = await get_ingestor_state(guid, RmfRepository(user))
    if ingestor_state:
        return obs.pipe(rxops.start_with(ingestor_state))
    return obs


@router.get("/{guid}/health", response_model=IngestorHealth)
async def get_ingestor_health(
    guid: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    """
    Available in socket.io
    """
    ingestor_health = await rmf_repo.get_ingestor_health(guid)
    if ingestor_health is None:
        raise HTTPException(status_code=404)
    return ingestor_health


@router.sub("/{guid}/health", response_model=IngestorHealth)
async def sub_ingestor_health(req: SubscriptionRequest, guid: str):
    user = sio_user(req)
    obs = rmf_events.ingestor_health.pipe(rxops.filter(lambda x: x.id_ == guid))
    health = await get_ingestor_health(guid, RmfRepository(user))
    if health:
        return obs.pipe(rxops.start_with(health))
    return obs
