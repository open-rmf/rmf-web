import uuid
from datetime import datetime
from typing import Dict, List

from fastapi import Depends, HTTPException
from pydantic import BaseModel
from tortoise.exceptions import IntegrityError

from api_server.authenticator import user_dep
from api_server.fast_io import FastIORouter
from api_server.logger import logger
from api_server.models import User
from api_server.models import tortoise_models as ttm

router = FastIORouter(tags=["Tasks"])


class TaskFavoritePydantic(BaseModel):
    id: str
    name: str
    unix_millis_earliest_start_time: int
    priority: Dict | None
    category: str
    description: Dict | None
    user: str


@router.post("", response_model=None)
async def post_favorite_task(
    request: TaskFavoritePydantic,
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
            },
            id=request.id if request.id != "" else uuid.uuid4(),
        )
    except IntegrityError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=List[TaskFavoritePydantic])
async def get_favorites_tasks(
    user: User = Depends(user_dep),
):
    favorites_tasks = await ttm.TaskFavorite.filter(user=user.username)
    result: list[TaskFavoritePydantic] = []
    for favorite_task in favorites_tasks:
        if not isinstance(favorite_task.priority, dict):
            logger.error(f"priority is not a dict: {type(favorite_task.priority)}")
            raise HTTPException(500)
        if not isinstance(favorite_task.description, dict):
            logger.error(
                f"description is not a dict: {type(favorite_task.description)}"
            )
            raise HTTPException(500)

        result.append(
            TaskFavoritePydantic(
                id=favorite_task.id,
                name=favorite_task.name,
                unix_millis_earliest_start_time=int(
                    favorite_task.unix_millis_earliest_start_time.strftime(
                        "%Y%m%d%H%M%S"
                    )
                ),
                priority=favorite_task.priority if favorite_task.priority else None,
                category=favorite_task.category,
                description=favorite_task.description
                if favorite_task.description
                else None,
                user=user.username,
            )
        )
    return result


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
