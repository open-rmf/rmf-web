import uuid
from datetime import datetime
from typing import Dict, List

from fastapi import Depends, HTTPException
from tortoise.exceptions import IntegrityError

from api_server.authenticator import user_dep
from api_server.fast_io import FastIORouter
from api_server.models import TaskFavorite, User
from api_server.models import tortoise_models as ttm

router = FastIORouter(tags=["Tasks"])


@router.post("")
async def post_favorite_task(
    request: TaskFavorite,
    user: User = Depends(user_dep),
):
    try:
        await ttm.TaskFavorite.update_or_create(
            {
                "name": request.name,
                "unix_millis_earliest_start_time": datetime.fromtimestamp(
                    request.unix_millis_earliest_start_time / 1000
                ),
                "priority": request.priority if request.priority else None,
                "category": request.category,
                "description": request.description if request.description else None,
                "user": user.username,
                "labels": request.labels,
            },
            id=request.id if request.id != "" else uuid.uuid4(),
        )
    except IntegrityError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=List[TaskFavorite])
async def get_favorites_tasks(
    user: User = Depends(user_dep),
):
    favorites_tasks = await ttm.TaskFavorite.filter(user=user.username)
    return [
        TaskFavorite(
            id=favorite_task.id,
            name=favorite_task.name,
            unix_millis_earliest_start_time=int(
                favorite_task.unix_millis_earliest_start_time.strftime("%Y%m%d%H%M%S")
            ),
            priority=favorite_task.priority if favorite_task.priority else None,
            category=favorite_task.category,
            description=favorite_task.description
            if favorite_task.description
            else None,
            user=user.username,
            labels=favorite_task.labels,
        )
        for favorite_task in favorites_tasks
    ]


@router.delete("/{favorite_task_id}")
async def delete_favorite_task(
    favorite_task_id: str,
):
    favorite_task = await ttm.TaskFavorite.get_or_none(id=favorite_task_id)
    if favorite_task is None:
        raise HTTPException(
            404, f"Favorite task with id {favorite_task_id} does not exists"
        )
    await favorite_task.delete()
