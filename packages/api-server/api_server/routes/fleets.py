from typing import List, Tuple, cast

from fastapi import Depends, HTTPException
from rx import operators as rxops

from api_server.authenticator import user_dep
from api_server.dependencies import between_query, sio_user
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.logger import logger
from api_server.models import (
    Commission,
    Error,
    FleetLog,
    FleetState,
    RobotCommissionRequest,
    RobotCommissionResponse,
    RobotCommissionResponseItem1,
    User,
)
from api_server.repositories import FleetRepository, fleet_repo_dep
from api_server.response import RawJSONResponse
from api_server.rmf_io import fleet_events, robots_service

router = FastIORouter(tags=["Fleets"])


@router.get("", response_model=List[FleetState])
async def get_fleets(
    repo: FleetRepository = Depends(fleet_repo_dep),
):
    return await repo.get_all_fleets()


@router.get("/{name}/state", response_model=FleetState)
async def get_fleet_state(name: str, repo: FleetRepository = Depends(fleet_repo_dep)):
    """
    Available in socket.io
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(status_code=404)
    return fleet_state


@router.sub("/{name}/state", response_model=FleetState)
async def sub_fleet_state(req: SubscriptionRequest, name: str):
    user = sio_user(req)
    repo = FleetRepository(user)
    obs = fleet_events.fleet_states.pipe(
        rxops.filter(lambda x: cast(FleetState, x).name == name)
    )
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state:
        return obs.pipe(rxops.start_with(fleet_state))
    return obs


@router.get("/{name}/log", response_model=FleetLog)
async def get_fleet_log(
    name: str,
    repo: FleetRepository = Depends(fleet_repo_dep),
    between: Tuple[int, int] = Depends(between_query),
):
    """
    Available in socket.io
    """
    fleet_log = await repo.get_fleet_log(name, between)
    if fleet_log is None:
        raise HTTPException(status_code=404)
    return fleet_log


@router.sub("/{name}/log", response_model=FleetLog)
async def sub_fleet_log(_req: SubscriptionRequest, name: str):
    return fleet_events.fleet_logs.pipe(
        rxops.filter(lambda x: cast(FleetLog, x).name == name)
    )


@router.post(
    "/{name}/decommission",
    response_model=RobotCommissionResponse,
    responses={
        400: {"model": RobotCommissionResponseItem1},
        404: {"model": RobotCommissionResponseItem1},
    },
)
async def decommission_robot(
    name: str,
    robot_name: str,
    reassign_tasks: bool,
    repo: FleetRepository = Depends(fleet_repo_dep),
    user: User = Depends(user_dep),
):
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        resp = RobotCommissionResponseItem1(
            success=False,
            errors=[Error(code=404, category=None, detail=f"Fleet {name} not found")],
        )
        return RawJSONResponse(resp.json(), 404)
    if fleet_state.robots is None or robot_name not in fleet_state.robots:
        resp = RobotCommissionResponseItem1(
            success=False,
            errors=[
                Error(
                    code=404,
                    category=None,
                    detail=f"Robot {robot_name} not found in fleet {name}",
                )
            ],
        )
        return RawJSONResponse(resp.json(), 404)

    commission = Commission(
        dispatch_tasks=False, direct_tasks=False, idle_behavior=False
    )
    request = RobotCommissionRequest(
        type="commission_request",
        robot=robot_name,
        fleet=name,
        commission=commission,
        reassign_tasks=reassign_tasks,
    )

    logger.info(f"Decommissioning {robot_name} of {name} called by {user.username}")
    if reassign_tasks:
        logger.info("Task re-assignment requested")
    else:
        logger.info("No Task re-assignment requested, tasks will be cancelled")
    resp = RobotCommissionResponse.parse_raw(
        await robots_service().call(request.json(exclude_none=True))
    )
    logger.info(resp)
    if not resp.__root__.success:
        logger.error(f"Failed to decommission {robot_name} of {name}")
        return RawJSONResponse(resp.json(), 400)
    return resp.__root__


@router.post(
    "/{name}/recommission",
    response_model=RobotCommissionResponse,
    responses={
        400: {"model": RobotCommissionResponseItem1},
        404: {"model": RobotCommissionResponseItem1},
    },
)
async def recommission_robot(
    name: str,
    robot_name: str,
    repo: FleetRepository = Depends(fleet_repo_dep),
    user: User = Depends(user_dep),
):
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        resp = RobotCommissionResponseItem1(
            success=False,
            errors=[Error(code=404, category=None, detail=f"Fleet {name} not found")],
        )
        return RawJSONResponse(resp.json(), 404)
    if fleet_state.robots is None or robot_name not in fleet_state.robots:
        resp = RobotCommissionResponseItem1(
            success=False,
            errors=[
                Error(
                    code=404,
                    category=None,
                    detail=f"Robot {robot_name} not found in fleet {name}",
                )
            ],
        )
        return RawJSONResponse(resp.json(), 404)

    commission = Commission(dispatch_tasks=True, direct_tasks=True, idle_behavior=True)
    request = RobotCommissionRequest(
        type="commission_request",
        robot=robot_name,
        fleet=name,
        commission=commission,
        reassign_tasks=None,
    )

    logger.info(f"Recommissioning {robot_name} of {name} called by {user.username}")
    resp = RobotCommissionResponse.parse_raw(
        await robots_service().call(request.json(exclude_none=True))
    )
    logger.info(resp)
    if not resp.__root__.success:
        logger.error(f"Failed to recommission {robot_name} of {name}")
        return RawJSONResponse(resp.json(), 400)
    return resp.__root__
