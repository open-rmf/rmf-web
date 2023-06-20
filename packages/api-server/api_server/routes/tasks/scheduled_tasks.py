import asyncio
from datetime import datetime

import schedule
import tortoise.transactions
from fastapi import Depends, HTTPException, Query
from pydantic import BaseModel
from tortoise.expressions import Q

from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter
from api_server.logger import logger
from api_server.models import DispatchTaskRequest, Pagination, TaskRequest, User
from api_server.models import tortoise_models as ttm
from api_server.repositories import TaskRepository, task_repo_dep

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[ttm.ScheduledTaskSchedulePydantic]


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
        asyncio.get_event_loop().create_task(run())

    for _, j in jobs:
        j.do(do)
    logger.info(f"scheduled task [{task.pk}]")


@router.post("", status_code=201, response_model=ttm.ScheduledTaskPydantic)
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

            await schedule_task(scheduled_task, task_repo)
        return await ttm.ScheduledTaskPydantic.from_tortoise_orm(scheduled_task)
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
async def get_scheduled_task(task_id: int):
    task = await ttm.ScheduledTask.get_or_none(id=task_id).prefetch_related("schedules")
    if task is None:
        raise HTTPException(404)
    return task


@router.delete("/{task_id}")
async def del_scheduled_tasks(task_id: int):
    async with tortoise.transactions.in_transaction():
        task = await get_scheduled_task(task_id)
        await task.delete()
        schedule.clear(f"task_{task_id}")
