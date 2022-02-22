from typing import List, Tuple, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops
from rx.subject.replaysubject import ReplaySubject
from rx.subject.subject import Subject

from api_server.dependencies import sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import MotionObjectModel, VideoRoomModel
from api_server.repositories import RmfRepository, rmf_repo_dep
from api_server.rmf_io import fleet_events

router = FastIORouter(tags=["Teleoperation"])
join_video_room_obs = Subject()
leave_video_room_obs = Subject()
motion_obs = Subject()


@router.post("/{name}/video/join")
async def join_video_room(
    name: str, room: VideoRoomModel, rmf_repo: RmfRepository = Depends(rmf_repo_dep)
):
    join_video_room_obs.on_next({"id": name, "data": room.dict()})


@router.sub("/{name}/video/join")
def sub_join_video_room(req: SubscriptionRequest, name: str):
    return join_video_room_obs.pipe(rxops.filter(lambda x: x["id"] == name))


@router.post("/{name}/video/leave")
async def leave_video_room(name: str, rmf_repo: RmfRepository = Depends(rmf_repo_dep)):
    leave_video_room_obs.on_next({"id": name})


@router.sub("/{name}/video/leave")
def sub_leave_video_room(req: SubscriptionRequest, name: str):
    return leave_video_room_obs.pipe(rxops.filter(lambda x: x["id"] == name))


@router.post("/{name}/move")
async def move(
    name: str,
    motion: MotionObjectModel,
    rmf_repo: RmfRepository = Depends(rmf_repo_dep),
):
    motion_obs.on_next({"id": name, "data": motion.dict()})


@router.sub("/{name}/move")
def sub_move(req: SubscriptionRequest, name: str):
    return motion_obs.pipe(rxops.filter(lambda x: x["id"] == name))
