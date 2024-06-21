import asyncio
from datetime import datetime
from typing import Optional

import schedule
import tortoise.transactions
from fastapi import Depends, HTTPException, Query
from pydantic import BaseModel
from tortoise.expressions import Q

from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter
from api_server.logger import logger
from api_server.models import (
    DispatchTaskRequest,
    Pagination,
    ScheduledTask,
    ScheduledTaskSchedule,
    TaskRequest,
    User,
)
from api_server.models import tortoise_models as ttm
from api_server.repositories import TaskRepository, task_repo_dep

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[ScheduledTaskSchedule]


async def schedule_task(task: ttm.ScheduledTask, task_repo: TaskRepository):
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
        await post_dispatch_task(req, task_repo)
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
    user: User = Depends(user_dep),
    task_repo: TaskRepository = Depends(task_repo_dep),
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
            )
            schedules = [
                ttm.ScheduledTaskSchedule(
                    scheduled_task=scheduled_task, **x.model_dump()
                )
                for x in scheduled_task_request.schedules
            ]
            await ttm.ScheduledTaskSchedule.bulk_create(schedules)

            await schedule_task(scheduled_task, task_repo)
        return ScheduledTask.model_validate(scheduled_task)
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=list[ScheduledTask])
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
        .prefetch_related("schedules")
        .distinct()
        .limit(pagination.limit)
        .offset(pagination.offset)
    )
    if pagination.order_by:
        q.order_by(*pagination.order_by)
    results = await q
    await ttm.ScheduledTask.fetch_for_list(results)
    return [ScheduledTask.model_validate(x) for x in results]


@router.get("/{task_id}", response_model=ScheduledTask)
async def get_scheduled_task(task_id: int) -> ttm.ScheduledTask:
    task = await ttm.ScheduledTask.get_or_none(id=task_id).prefetch_related("schedules")
    if task is None:
        raise HTTPException(404)
    return task


@router.put("/{task_id}/clear")
async def del_scheduled_tasks_event(
    task_id: int,
    event_date: datetime,
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    task = await get_scheduled_task(task_id)
    if task is None:
        raise HTTPException(404)

    event_date_str = event_date.isoformat()
    if not isinstance(task.except_dates, list):
        logger.error(f"task.except_dates is not a list: {type(task.except_dates)}")
        raise HTTPException(500)
    task.except_dates.append(event_date_str[:10])
    await task.save()

    for sche in task.schedules:
        schedule.clear(sche.get_id())

    await schedule_task(task, task_repo)


@router.post("/{task_id}/update", status_code=201, response_model=ScheduledTask)
async def update_schedule_task(
    task_id: int,
    scheduled_task_request: PostScheduledTaskRequest,
    except_date: Optional[datetime] = None,
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    try:
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
                event_date_str = except_date.isoformat()
                if not isinstance(task.except_dates, list):
                    logger.error(
                        f"task.except_dates is not a list: {type(task.except_dates)}"
                    )
                    raise HTTPException(500)
                task.except_dates.append(event_date_str[:10])
                await task.save()

                for sche in task.schedules:
                    schedule.clear(sche.get_id())

                await schedule_task(task, task_repo)

                scheduled_task = await ttm.ScheduledTask.create(
                    task_request=scheduled_task_request.task_request.model_dump_json(
                        exclude_none=True
                    ),
                    created_by=task.created_by,
                )
                schedules = [
                    ttm.ScheduledTaskSchedule(
                        scheduled_task=scheduled_task, **x.model_dump()
                    )
                    for x in scheduled_task_request.schedules
                ]
                await ttm.ScheduledTaskSchedule.bulk_create(schedules)

                await schedule_task(scheduled_task, task_repo)
            else:
                # If "except_date" is not provided, it means the entire series is being updated.
                # In this case, we perform the following steps:
                #   1. Update the task with the requested data from the schedule form and clear exception dates.
                #   2. Clear all existing schedules associated with the task.
                #   3. Delete all existing schedules associated with the task.
                #   4. Create new schedules based on the requested data.
                task.update_from_dict(
                    {
                        "task_request": scheduled_task_request.task_request.model_dump_json(
                            exclude_none=True
                        ),
                        "except_dates": [],
                    }
                )

                for sche in task.schedules:
                    schedule.clear(sche.get_id())
                for sche in task.schedules:
                    await sche.delete()

                await task.save()
                schedules = [
                    ttm.ScheduledTaskSchedule(scheduled_task=task, **x.model_dump())
                    for x in scheduled_task_request.schedules
                ]

                await ttm.ScheduledTaskSchedule.bulk_create(schedules)

                await schedule_task(task, task_repo)
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e)) from e


@router.delete("/{task_id}")
async def del_scheduled_tasks(task_id: int):
    async with tortoise.transactions.in_transaction():
        task = await get_scheduled_task(task_id)
        for sche in task.schedules:
            schedule.clear(sche.get_id())
        await task.delete()
