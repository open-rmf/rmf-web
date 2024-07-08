import asyncio
from datetime import datetime
from typing import Annotated, cast
from zoneinfo import ZoneInfo

import pydantic
import schedule
import tortoise.timezone
import tortoise.transactions
from fastapi import Depends, HTTPException, Query
from pydantic import BaseModel
from tortoise.expressions import Q

from api_server.app_config import app_config
from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import (
    DispatchTaskRequest,
    Pagination,
    ScheduledTask,
    ScheduledTaskSchedule,
    TaskRequest,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.repositories import TaskRepository
from api_server.repositories.rmf import RmfRepository
from api_server.rmf_io.rmf_service import TasksService

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[ScheduledTaskSchedule]
    except_dates: list[datetime] = pydantic.Field(
        default=[],
        description="A list of dates which the schedule should be skipped, this is based on the server date. The time portion will be discarded, it is only used to convert the date to the server timezone.",
    )
    tz: str | None = None
    """Timezone of the schedules, if omitted, the server timezone will be used
    
    Note that this does not affect the `start_from` param, it will always use the
    timezone specified on the param.
    """


async def schedule_task(
    task: ttm.ScheduledTask,
    rmf_repo: RmfRepository,
    task_repo: TaskRepository,
    tasks_service: TasksService,
    logger: LoggerAdapter,
):
    await task.fetch_related("schedules")
    jobs: list[tuple[ttm.ScheduledTaskSchedule, schedule.Job]] = []
    for sche in task.schedules:
        try:
            jobs.append((sche, sche.to_job()))
        except schedule.ScheduleValueError:
            pass
    if len(jobs) == 0:
        # don't allow creating scheduled tasks that never runs
        raise HTTPException(422, "Task is never going to run")

    if not isinstance(task.task_request, dict):
        logger.error(f"task_request is not a dict: {type(task.task_request)}")
        raise HTTPException(500)
    req = DispatchTaskRequest(
        type="dispatch_task_request",
        request=TaskRequest(**task.task_request),
    )

    async def run():
        await post_dispatch_task(req, rmf_repo, task_repo, logger, tasks_service)
        task.last_ran = datetime.now()
        await task.save()

    def do():
        logger.info(f"starting task {task.pk}")
        datetime_to_iso = datetime.now().isoformat()
        if datetime_to_iso[:10] in task.except_dates:
            return
        asyncio.get_event_loop().create_task(run())

    for _, j in jobs:
        j.do(do)
    logger.info(f"scheduled task [{task.pk}]")


@router.post("", status_code=201, response_model=ScheduledTask)
async def post_scheduled_task(
    scheduled_task_request: PostScheduledTaskRequest,
    user: Annotated[User, Depends(user_dep)],
    rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)],
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
):
    """
    Create a scheduled task. Below are some examples of how the schedules are represented.
    For more examples, check the docs of the underlying library used [here](https://github.com/dbader/schedule/blob/6eb0b5346b1ce35ece5050e65789fa6e44368175/docs/examples.rst).

    | every | to | period | at | description |
    | - | - | - | - | - |
    | 10 | - | minutes | - | Every 10 minutes |
    | - | - | hour | - | Every hour |
    | - | - | day | 10:30 | Every day at 10:30am |
    | - | - | monday | - | Every monday |
    | - | - | wednesday | 13:15 | Every wednesday at 01:15pm |
    | - | - | minute | :17 | Every 17th sec of a mintue |
    | 5 | 10 | seconds | - | Every 5-10 seconds (randomly) |
    """
    try:
        async with tortoise.transactions.in_transaction():
            scheduled_task = await ttm.ScheduledTask.create(
                task_request=scheduled_task_request.task_request.model_dump_json(
                    exclude_none=True
                ),
                created_by=user.username,
                except_dates=[
                    ttm.ScheduledTask.format_except_date(x)
                    for x in scheduled_task_request.except_dates
                ],
            )
            schedules = [
                ttm.ScheduledTaskSchedule(
                    scheduled_task=scheduled_task, **x.model_dump()
                )
                for x in scheduled_task_request.schedules
            ]
            await ttm.ScheduledTaskSchedule.bulk_create(schedules)

            await scheduled_task.fetch_related("schedules")
            await schedule_task(
                scheduled_task, rmf_repo, task_repo, tasks_service, logger
            )

            return scheduled_task
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=list[ScheduledTask])
async def get_scheduled_tasks(
    start_before: Annotated[
        datetime,
        Query(
            description="Only return scheduled tasks that start before given timestamp"
        ),
    ],
    until_after: Annotated[
        datetime,
        Query(
            description="Only return scheduled tasks that stop after given timestamp"
        ),
    ],
    pagination: Annotated[Pagination, Depends(pagination_query)],
):
    q = (
        ttm.ScheduledTask.filter(
            Q(schedules__start_from__lte=start_before)
            | Q(schedules__start_from__isnull=True),
            Q(schedules__until__gte=until_after) | Q(schedules__until__isnull=True),
        )
        .prefetch_related("schedules")
        .distinct()
        .limit(pagination.limit)
        .offset(pagination.offset)
    )
    if pagination.order_by:
        q.order_by(*pagination.order_by)
    results = await q
    await ttm.ScheduledTask.fetch_for_list(results, "schedules")
    return [ScheduledTask.model_validate(x) for x in results]


@router.get("/{task_id}", response_model=ScheduledTask)
async def get_scheduled_task(task_id: int) -> ttm.ScheduledTask:
    task = await ttm.ScheduledTask.get_or_none(id=task_id).prefetch_related("schedules")
    if task is None:
        raise HTTPException(404)
    return task


def convert_timezone_from(to_convert: datetime, reference: datetime):
    tz = reference.tzinfo or ZoneInfo(app_config.timezone)
    return to_convert.astimezone(tz)


class AddExceptDateRequest(BaseModel):
    except_date: datetime


@router.post(
    "/{task_id}/except_date",
    description="Skip tasks on the excepted date",
    status_code=201,
)
async def add_except_date(
    task_id: int,
    data: AddExceptDateRequest,
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
):
    task = await get_scheduled_task(task_id)

    new_except_date = ttm.ScheduledTask.format_except_date(data.except_date)
    exists = next(
        (True for x in task.except_dates if x == new_except_date),
        False,
    )
    if not exists:
        cast(list[str], task.except_dates).append(new_except_date)
        await task.save()
        logger.info(f"added except date {new_except_date} to task {task_id}")


@router.post("/{task_id}/update", status_code=200, response_model=ScheduledTask)
async def update_schedule_task(
    task_id: int,
    scheduled_task_request: PostScheduledTaskRequest,
    rmf_repo: Annotated[RmfRepository, Depends(RmfRepository)],
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    tasks_service: Annotated[TasksService, Depends(TasksService.get_instance)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
):
    task = await get_scheduled_task(task_id)
    try:
        async with tortoise.transactions.in_transaction():
            task.task_request = scheduled_task_request.task_request.model_dump(
                round_trip=True
            )
            task.except_dates = [
                ttm.ScheduledTask.format_except_date(x)
                for x in scheduled_task_request.except_dates
            ]
            await task.save()

            for sche in task.schedules:
                schedule.clear(sche.get_id())
                await sche.delete()

            new_schedules = [
                ttm.ScheduledTaskSchedule(scheduled_task=task, **x.model_dump())
                for x in scheduled_task_request.schedules
            ]

            await ttm.ScheduledTaskSchedule.bulk_create(new_schedules)

            await schedule_task(task, rmf_repo, task_repo, tasks_service, logger)
            return task
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.delete("/{task_id}")
async def del_scheduled_tasks(task_id: int):
    async with tortoise.transactions.in_transaction():
        task = await get_scheduled_task(task_id)
        for sche in task.schedules:
            schedule.clear(sche.get_id())
        await task.delete()
