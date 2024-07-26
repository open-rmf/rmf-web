from datetime import datetime
from typing import Annotated

from fastapi import Depends, HTTPException
from reactivex import operators as rxops

from api_server.authenticator import user_dep
from api_server.dependencies import time_between_query
from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.gateway import RmfGateway, get_rmf_gateway
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import (
    Commission,
    FleetLog,
    FleetState,
    PendingDirectTasksPolicy,
    PendingDispatchTasksPolicy,
    RobotCommissionRequest,
    RobotCommissionResponse,
    User,
)
from api_server.repositories import FleetRepository
from api_server.rmf_io import get_tasks_service
from api_server.rmf_io.events import get_fleet_events
from api_server.rmf_io.rmf_service import RmfService

router = FastIORouter(tags=["Fleets"])


@router.get("", response_model=list[FleetState])
async def get_fleets(
    repo: Annotated[FleetRepository, Depends(FleetRepository)],
):
    return await repo.get_all_fleets()


@router.get("/{name}/state", response_model=FleetState)
async def get_fleet_state(
    name: str, repo: Annotated[FleetRepository, Depends(FleetRepository)]
):
    """
    Available in socket.io
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(status_code=404)
    return fleet_state


@router.sub("/{name}/state", response_model=FleetState)
async def sub_fleet_state(req: SubscriptionRequest, name: str):
    user = req.user
    repo = FleetRepository(user, req.logger)
    obs = get_fleet_events().fleet_states.pipe(rxops.filter(lambda x: x.name == name))
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state:
        return obs.pipe(rxops.start_with(fleet_state))
    return obs


@router.get("/{name}/log", response_model=FleetLog)
async def get_fleet_log(
    name: str,
    repo: Annotated[FleetRepository, Depends(FleetRepository)],
    between: Annotated[
        tuple[datetime, datetime] | None,
        Depends(time_between_query("between", default="-60000")),
    ],
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
    return get_fleet_events().fleet_logs.pipe(rxops.filter(lambda x: x.name == name))


@router.post("/{name}/decommission", response_model=RobotCommissionResponse)
async def decommission_robot(
    name: str,
    robot_name: str,
    reassign_tasks: bool,
    allow_idle_behavior: bool,
    repo: Annotated[FleetRepository, Depends(FleetRepository)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    tasks_service: Annotated[RmfService, Depends(get_tasks_service)],
):
    """
    Decommissions a robot, cancels all direct tasks, and preventing it from
    accepting any new tasks (both dispatch tasks and direct tasks), with the
    options to:

    - Reassign all queued dispatch tasks to other robots. If task reassignment
      is chosen, the response will contain the results of the reassignment as
      well, any failed reassignments will be cancelled instead.
    - Still allow idle behaviors (formerly known as finishing tasks). If
      allowed, the robot will still be issued idle behavior commands (e.g.
      return to charger, park somewhere) once it is decommissioned, as opposed
      to waiting for human intervention at the same position when it was
      decommissioned.

    This will not affect the ongoing task that the robot is currently
    performing.
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(404, f"Fleet {name} not found")
    if fleet_state.robots is None or robot_name not in fleet_state.robots:
        raise HTTPException(404, f"Robot {robot_name} not found in fleet {name}")

    commission = Commission(
        dispatch_tasks=False, direct_tasks=False, idle_behavior=allow_idle_behavior
    )
    dispatch_policy = (
        PendingDispatchTasksPolicy.reassign
        if reassign_tasks
        else PendingDispatchTasksPolicy.cancel
    )
    request = RobotCommissionRequest(
        type="robot_commission_request",
        robot=robot_name,
        fleet=name,
        commission=commission,
        pending_direct_tasks_policy=PendingDirectTasksPolicy.cancel,
        pending_dispatch_tasks_policy=dispatch_policy,
    )

    reassignment_log_str = "No Task re-assignment requested, tasks will be cancelled."
    if reassign_tasks:
        reassignment_log_str = "Task re-assignment requested."
    logger.info(f"Decommissioning {robot_name} of {name}. {reassignment_log_str}")
    resp = RobotCommissionResponse.model_validate_json(
        await tasks_service.call(request.model_dump_json(exclude_none=True))
    )
    logger.info(resp)
    if not resp.commission.root.success:
        logger.error(f"Failed to decommission {robot_name} of {name}")
        raise HTTPException(400, resp)
    return resp


@router.post(
    "/{name}/recommission",
    response_model=RobotCommissionResponse,
)
async def recommission_robot(
    name: str,
    robot_name: str,
    repo: Annotated[FleetRepository, Depends(FleetRepository)],
    user: Annotated[User, Depends(user_dep)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    tasks_service: Annotated[RmfService, Depends(get_tasks_service)],
):
    """
    Recommissions a robot, allowing it to accept new dispatch tasks and direct
    tasks, as well as resume idle behaviors (formerly known as finishing tasks).
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(404, f"Fleet {name} not found")
    if fleet_state.robots is None or robot_name not in fleet_state.robots:
        raise HTTPException(404, f"Robot {robot_name} not found in fleet {name}")

    commission = Commission(dispatch_tasks=True, direct_tasks=True, idle_behavior=True)
    request = RobotCommissionRequest(
        type="robot_commission_request",
        robot=robot_name,
        fleet=name,
        commission=commission,
        pending_direct_tasks_policy=PendingDirectTasksPolicy.complete,
        pending_dispatch_tasks_policy=PendingDispatchTasksPolicy.complete,
    )

    logger.info(f"Recommissioning {robot_name} of {name} called by {user.username}")
    resp = RobotCommissionResponse.model_validate_json(
        await tasks_service.call(request.model_dump_json(exclude_none=True))
    )
    logger.info(resp)
    if not resp.commission.root.success:
        logger.error(f"Failed to recommission {robot_name} of {name}")
        raise HTTPException(400, resp)
    return resp


@router.post("/{name}/unlock_mutex_group")
async def unlock_mutex_group(
    name: str,
    robot_name: str,
    mutex_group: str,
    repo: Annotated[FleetRepository, Depends(FleetRepository)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
    rmf_gateway: Annotated[RmfGateway, Depends(get_rmf_gateway)],
):
    """
    Request to manually unlock a mutex group that is currently being held by a
    specific robot of a specific fleet.
    """
    fleet_state = await repo.get_fleet_state(name)
    if fleet_state is None:
        raise HTTPException(404, f"Fleet {name} not found")
    if fleet_state.robots is None or robot_name not in fleet_state.robots:
        raise HTTPException(404, f"Robot {robot_name} not found in fleet {name}")
    mutex_groups = fleet_state.robots[robot_name].mutex_groups
    if (
        mutex_groups is None
        or mutex_groups.locked is None
        or mutex_group not in mutex_groups.locked
    ):
        raise HTTPException(
            400,
            f"Robot {robot_name} in fleet {name} does not have mutex group {mutex_group} locked",
        )

    logger.info(
        f"Manual release mutex group {mutex_group} for {robot_name} of fleet {name} requested"
    )
    rmf_gateway.manual_release_mutex_groups(
        mutex_groups=[mutex_group], fleet=name, robot=robot_name
    )
