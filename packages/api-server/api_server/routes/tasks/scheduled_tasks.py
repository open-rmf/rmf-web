import asyncio
from datetime import date, datetime
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
from api_server.rmf_io.rmf_service import RmfService, get_tasks_service
from api_server.scheduler import get_scheduler

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[ScheduledTaskSchedule]
    start_from: datetime | None = None
    until: datetime | None = None
    except_dates: list[datetime] = pydantic.Field(
        default=[],
        description="A list of dates which the schedule should be skipped, this is based on the server date. The time portion will be discarded, it is only used to convert the date to the server timezone.",
    )


async def schedule_task(
    task: ttm.ScheduledTask,
    task_repo: TaskRepository,
    tasks_service: RmfService,
    scheduler: schedule.Scheduler,
    logger: LoggerAdapter,
):
    jobs = await task.to_jobs(scheduler)
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

    async def run(j: schedule.Job):
        # fetch the task from db again in case the schedule changed
        task_to_run = await ttm.ScheduledTask.get_or_none(id=task.pk)
        if task_to_run is None:
            logger.warning(
                f"failed to run scheduled task {task.pk}: scheduled task does not exist"
            )
            return

        # check if we should skip this run
        if (
            task_to_run.start_from is not None
            and task_to_run.start_from > datetime.now(tz=ZoneInfo(app_config.timezone))
        ):
            logger.info(f"skipping task {task.pk} because it is before start_from")
            return

        except_dates = [date.fromisoformat(x) for x in task_to_run.except_dates]
        # `next_run` contains the expected time it should run, we use that over `datetime.now`
        # in case of delays in the scheduler.
        if j.next_run and j.next_run.date() in except_dates:
            logger.info(f"skipping task {task.pk} because it is in the except_dates")
            return

        logger.info(f"starting task {task.pk}")
        await post_dispatch_task(req, task_repo, logger, tasks_service)
        task.last_ran = datetime.now(tz=ZoneInfo(app_config.timezone))
        await task.save()

    for j in jobs:
        j.do(lambda inner_j: asyncio.create_task(run(inner_j)), j)
    logger.info(f"scheduled task [{task.pk}]")


@router.post("", status_code=201, response_model=ScheduledTask)
async def post_scheduled_task(
    scheduled_task_request: PostScheduledTaskRequest,
    user: Annotated[User, Depends(user_dep)],
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    tasks_service: Annotated[RmfService, Depends(get_tasks_service)],
    scheduler: Annotated[schedule.Scheduler, Depends(get_scheduler)],
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
                start_from=scheduled_task_request.start_from,
                until=scheduled_task_request.until,
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
                scheduled_task, task_repo, tasks_service, scheduler, logger
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
            Q(start_from__lte=start_before) | Q(start_from__isnull=True),
            Q(until__gte=until_after) | Q(until__isnull=True),
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
    task_repo: Annotated[TaskRepository, Depends(TaskRepository)],
    tasks_service: Annotated[RmfService, Depends(get_tasks_service)],
    scheduler: Annotated[schedule.Scheduler, Depends(get_scheduler)],
    logger: Annotated[LoggerAdapter, Depends(get_logger)],
):
    task = await get_scheduled_task(task_id)
    try:
        async with tortoise.transactions.in_transaction():
            task.task_request = scheduled_task_request.task_request.model_dump(
                round_trip=True
            )
            task.start_from = scheduled_task_request.start_from
            task.until = scheduled_task_request.until
            task.except_dates = [
                ttm.ScheduledTask.format_except_date(x)
                for x in scheduled_task_request.except_dates
            ]
            await task.save()

            for sche in task.schedules:
                scheduler.clear(sche.get_id())
                await sche.delete()

            new_schedules = [
                ttm.ScheduledTaskSchedule(scheduled_task=task, **x.model_dump())
                for x in scheduled_task_request.schedules
            ]

            await ttm.ScheduledTaskSchedule.bulk_create(new_schedules)

            await schedule_task(task, task_repo, tasks_service, scheduler, logger)
            return task
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.delete("/{task_id}")
async def del_scheduled_tasks(
    task_id: int, scheduler: Annotated[schedule.Scheduler, Depends(get_scheduler)]
):
    async with tortoise.transactions.in_transaction():
        task = await get_scheduled_task(task_id)
        for sche in task.schedules:
            scheduler.clear(sche.get_id())
        await task.delete()
