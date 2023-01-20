import asyncio
from datetime import datetime

import schedule
import tortoise.transactions
from fastapi import Depends, HTTPException, Response
from pydantic import BaseModel

from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter
from api_server.models import DispatchTaskRequest, Pagination, TaskRequest
from api_server.models import tortoise_models as ttm
from api_server.repositories import TaskRepository, task_repo_dep

from .tasks import post_dispatch_task

router = FastIORouter(tags=["Tasks"])


class PostScheduledTaskRequest(BaseModel):
    task_request: TaskRequest
    schedules: list[
        ttm.ScheduledTaskSchedulePydantic
    ]  # a scheduled task can have multiple schedules


async def schedule_task(task: ttm.ScheduledTask, task_repo: TaskRepository):
    await task.fetch_related("schedules")
    jobs = [x.to_job() for x in task.schedules]
    for j in jobs:
        req = DispatchTaskRequest(
            type="dispatch_task_request",
            request=TaskRequest.parse_raw(task.task_request),
        )

        async def run():
            await post_dispatch_task(req, task_repo)
            task.last_ran = datetime.now()
            await task.save()

        j.do(lambda: asyncio.get_event_loop().create_task(run()))

    # Job have a operator overload that sorts based on the next run
    next_run = min(jobs).next_run
    if next_run != task.next_run:
        task.next_run = next_run
        await task.save()


@router.post("", status_code=201, response_class=Response)
async def post_scheduled_task(
    scheduled_task_request: PostScheduledTaskRequest,
    task_repo: TaskRepository = Depends(task_repo_dep),
):
    try:
        async with tortoise.transactions.in_transaction():
            scheduled_task = await ttm.ScheduledTask.create(
                task_request=scheduled_task_request.task_request.json(
                    exclude_none=True
                ),
            )
            schedules = [
                ttm.ScheduledTaskSchedule(scheduled_task=scheduled_task, **x.dict())
                for x in scheduled_task_request.schedules
            ]
            await ttm.ScheduledTaskSchedule.bulk_create(schedules)

            await schedule_task(scheduled_task, task_repo)
            if scheduled_task.next_run is None:
                # don't allow creating scheduled tasks that never runs
                raise HTTPException(422, "Task is never going to run")
    except schedule.ScheduleError as e:
        raise HTTPException(422, str(e))


@router.get("", response_model=list[ttm.ScheduledTaskPydantic])
async def get_scheduled_tasks(pagination: Pagination = Depends(pagination_query)):
    q = ttm.ScheduledTask.all().limit(pagination.limit).offset(pagination.offset)
    if pagination.order_by:
        q.order_by(*pagination.order_by.split(","))
    return await q
