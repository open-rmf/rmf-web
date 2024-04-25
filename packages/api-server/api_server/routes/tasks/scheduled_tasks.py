import asyncio
from datetime import datetime
from typing import Optional
from zoneinfo import ZoneInfo

import schedule
import tortoise.transactions
from fastapi import Depends, HTTPException, Query
from pydantic import BaseModel
from tortoise.expressions import Q

from api_server.app_config import app_config
from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter
from api_server.logging import LoggerAdapter, get_logger
from api_server.models import DispatchTaskRequest, Pagination, TaskRequest, User
from api_server.models import tortoise_models as ttm
from api_server.repositories import TaskRepository

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[ttm.ScheduledTaskSchedulePydantic]


async def schedule_task(
    task: ttm.ScheduledTask, task_repo: TaskRepository, logger: LoggerAdapter
):
    await task.fetch_related("schedules")
    jobs: list[tuple[ttm.ScheduledTaskSchedule, schedule.Job]] = []
    for sche in task.schedules:
        try:
            jobs.append((sche, sche.to_job(app_config.timezone)))
        except schedule.ScheduleValueError:
            pass
    if len(jobs) == 0:
        # don't allow creating scheduled tasks that never runs
        raise HTTPException(422, "Task is never going to run")

    req = DispatchTaskRequest(
        type="dispatch_task_request",
        request=TaskRequest(**task.task_request),
    )

    async def run():
        req.request.unix_millis_request_time = round(datetime.now().timestamp() * 1e3)
        await post_dispatch_task(req, task_repo, logger)
        task.last_ran = datetime.now()
        await task.save()

    def do(start_from: Optional[datetime]):
        logger.info(f"Checking if scheduled task [{task.pk}] needs to run")
        datetime_now = datetime.now()
        if start_from is not None and datetime_now < start_from:
            logger.info(
                f"Scheduled task [{task.pk}] is due to start from [{start_from}], skipping current execution"
            )
            return

        datetime_to_iso = datetime_now.isoformat()
        if datetime_to_iso[:10] in task.except_dates:
            logger.info(
                f"The current date [{datetime_to_iso}] is exempted for scheduled task [{task.pk}], skipping current execution"
            )
            return

        logger.info(f"Starting task {task.pk}")
        logger.info(f"{task.task_request}")
        asyncio.get_event_loop().create_task(run())
        logger.warning(f"Schedule has {len(schedule.get_jobs())} jobs left")

    for sched, j in jobs:
        start_from_datetime = None
        if sched.start_from is not None:
            start_from_datetime = datetime.fromtimestamp(sched.start_from.timestamp())
        j.do(do, start_from=start_from_datetime)
    logger.info(f"scheduled task [{task.pk}]")


def convert_date_server_timezone_iso_str(date: datetime, logger: LoggerAdapter) -> str:
    # Server time zone
    server_tz_info = ZoneInfo(app_config.timezone)
    logger.info(f"Server tz: {server_tz_info}")

    # If the event date has time zone information, we check if it is in the same
    # time zone as the server
    # FIXME This solution breaks if users change the time zone of the server
    # after creating schedules.
    event_date_utc_offset = date.utcoffset()
    server_time_utc_offset = datetime.now(tz=server_tz_info).utcoffset()
    if event_date_utc_offset is not None and server_time_utc_offset is not None:
        event_tz_utc_offset_seconds = round(event_date_utc_offset.total_seconds())
        server_tz_utc_offset_seconds = round(server_time_utc_offset.total_seconds())
        logger.info(
            f"Event tz utc offset: {event_tz_utc_offset_seconds}, server tz utc offset: {server_tz_utc_offset_seconds}"
        )

        # if the time zones are the same, we can extract the date directly
        if event_tz_utc_offset_seconds == server_tz_utc_offset_seconds:
            logger.info("Event and server are in the same timezone")
            date_str = date.isoformat()
        # otherwise, we convert the date before extracting the date
        else:
            logger.info("Event and server are not in the same timezone")
            event_date_local = date.astimezone(server_tz_info)
            date_str = event_date_local.isoformat()
    # Without any time zone information, we assume it is the same as the server
    else:
        logger.info("Event date does not contain any tz information, using server tz")
        date_str = date.replace(tzinfo=server_tz_info).isoformat()
    return date_str


@router.post("", status_code=201, response_model=ttm.ScheduledTaskPydantic)
async def post_scheduled_task(
    scheduled_task_request: PostScheduledTaskRequest,
    user: User = Depends(user_dep),
    task_repo: TaskRepository = Depends(TaskRepository),
    logger: LoggerAdapter = Depends(get_logger),
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
                task_request=scheduled_task_request.task_request.json(
                    exclude_none=True
                ),
                created_by=user.username,
            )
            schedules = [
                ttm.ScheduledTaskSchedule(scheduled_task=scheduled_task, **x.dict())
                for x in scheduled_task_request.schedules
            ]
            await ttm.ScheduledTaskSchedule.bulk_create(schedules)

            await schedule_task(scheduled_task, task_repo, logger)
        scheduled_task_model = await ttm.ScheduledTaskPydantic.from_tortoise_orm(
            scheduled_task
        )
        logger.info(f"New scheduled task:\n{scheduled_task_model}")
        return scheduled_task_model
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=ttm.ScheduledTaskPydanticList)
async def get_scheduled_tasks(
    start_before: datetime = Query(
        description="Only return scheduled tasks that start before given timestamp"
    ),
    until_after: datetime = Query(
        description="Only return scheduled tasks that stop after given timestamp"
    ),
    pagination: Pagination = Depends(pagination_query),
):
    q = (
        ttm.ScheduledTask.filter(
            Q(schedules__start_from__lte=start_before)
            | Q(schedules__start_from__isnull=True),
            Q(schedules__until__gte=until_after) | Q(schedules__until__isnull=True),
        )
        .distinct()
        .limit(pagination.limit)
        .offset(pagination.offset)
    )
    if pagination.order_by:
        q.order_by(*pagination.order_by.split(","))
    return await ttm.ScheduledTaskPydanticList.from_queryset(q)


@router.get("/{task_id}", response_model=ttm.ScheduledTaskPydantic)
async def get_scheduled_task(task_id: int) -> ttm.ScheduledTask:
    task = await ttm.ScheduledTask.get_or_none(id=task_id).prefetch_related("schedules")
    if task is None:
        raise HTTPException(404)
    return task


@router.put("/{task_id}/clear")
async def del_scheduled_tasks_event(
    task_id: int,
    event_date: datetime,
    task_repo: TaskRepository = Depends(TaskRepository),
    logger: LoggerAdapter = Depends(get_logger),
):
    logger.info(f"Deleting task with schedule id {task_id}, event date {event_date}")

    task = await get_scheduled_task(task_id)
    if task is None:
        logger.error(f"Task with scehdule id {task_id} not found")
        raise HTTPException(404)

    event_date_str = convert_date_server_timezone_iso_str(event_date, logger)

    logger.info(f"Deleting event with iso format date: {event_date_str}")
    task.except_dates.append(event_date_str[:10])
    await task.save()

    for sche in task.schedules:
        logger.info(f"Clearing schedule: {sche.get_id()}")
        schedule.clear(sche.get_id())

    pydantic_schedule_task = await ttm.ScheduledTaskPydantic.from_tortoise_orm(task)
    logger.info(f"Re-scheduling task: {pydantic_schedule_task}")
    try:
        await schedule_task(task, task_repo, logger)
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.post(
    "/{task_id}/update", status_code=201, response_model=ttm.ScheduledTaskPydantic
)
async def update_schedule_task(
    task_id: int,
    scheduled_task_request: PostScheduledTaskRequest,
    except_date: Optional[datetime] = None,
    task_repo: TaskRepository = Depends(TaskRepository),
    logger: LoggerAdapter = Depends(get_logger),
):
    try:
        logger.info(f"Updating scheduled task [{task_id}]")
        task = await get_scheduled_task(task_id)
        if task is None:
            raise HTTPException(404)
        # If "except_date" is provided, it means a single event is being updated.
        # In this case, we perform the following steps:
        #   1. Add the "except_date" to the list of exception dates for the task.
        #   2. Clear all existing schedules associated with the task.
        #   3. Create a new scheduled task with the requested data from the schedule form.

        async with tortoise.transactions.in_transaction():
            if except_date:
                event_date_str = convert_date_server_timezone_iso_str(
                    except_date, logger
                )
                logger.info(f"Updating event with iso format date: {event_date_str}")
                task.except_dates.append(event_date_str[:10])
                await task.save()

                for sche in task.schedules:
                    logger.info(f"Clearing schedule: {sche.get_id()}")
                    schedule.clear(sche.get_id())

                pydantic_schedule_task = (
                    await ttm.ScheduledTaskPydantic.from_tortoise_orm(task)
                )
                logger.info(f"Re-scheduling task: {pydantic_schedule_task}")
                await schedule_task(task, task_repo, logger)

                scheduled_task = await ttm.ScheduledTask.create(
                    task_request=scheduled_task_request.task_request.json(
                        exclude_none=True
                    ),
                    created_by=task.created_by,
                )
                schedules = [
                    ttm.ScheduledTaskSchedule(scheduled_task=scheduled_task, **x.dict())
                    for x in scheduled_task_request.schedules
                ]
                await ttm.ScheduledTaskSchedule.bulk_create(schedules)

                pydantic_schedule_single_event_task = (
                    await ttm.ScheduledTaskPydantic.from_tortoise_orm(scheduled_task)
                )
                logger.info(
                    f"Scheduling single event task: {pydantic_schedule_single_event_task}"
                )
                await schedule_task(scheduled_task, task_repo, logger)
            else:
                # If "except_date" is not provided, it means the entire series is being updated.
                # In this case, we perform the following steps:
                #   1. Update the task with the requested data from the schedule form and clear exception dates.
                #   2. Clear all existing schedules associated with the task.
                #   3. Delete all existing schedules associated with the task.
                #   4. Create new schedules based on the requested data.
                task.update_from_dict(
                    {
                        "task_request": scheduled_task_request.task_request.json(
                            exclude_none=True
                        ),
                        "except_dates": [],
                    }
                )

                for sche in task.schedules:
                    logger.info(f"Clearing schedule: {sche.get_id()}")
                    schedule.clear(sche.get_id())
                for sche in task.schedules:
                    await sche.delete()

                await task.save()
                schedules = [
                    ttm.ScheduledTaskSchedule(scheduled_task=task, **x.dict())
                    for x in scheduled_task_request.schedules
                ]

                await ttm.ScheduledTaskSchedule.bulk_create(schedules)

                pydantic_schedule_task = (
                    await ttm.ScheduledTaskPydantic.from_tortoise_orm(task)
                )
                logger.info(f"Re-scheduling task: {pydantic_schedule_task}")
                await schedule_task(task, task_repo, logger)
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.delete("/{task_id}")
async def del_scheduled_tasks(task_id: int):
    async with tortoise.transactions.in_transaction():
        task = await get_scheduled_task(task_id)
        for sche in task.schedules:
            schedule.clear(sche.get_id())
        await task.delete()
