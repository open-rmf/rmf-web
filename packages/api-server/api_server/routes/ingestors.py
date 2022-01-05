from typing import List, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject

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
    ingestor_state = await get_ingestor_state(guid, RmfRepository(user))
    sub = ReplaySubject(1)
    if ingestor_state:
        sub.on_next(ingestor_state)
    rmf_events.dispenser_states.subscribe(sub)
    return sub


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
    health = await get_ingestor_health(guid, RmfRepository(user))
    await req.sio.emit(req.room, health, req.sid)
    return rmf_events.ingestor_health.pipe(
        rxops.filter(lambda x: cast(IngestorHealth, x).id_ == guid)
    )
