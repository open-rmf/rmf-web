import uuid
from typing import Annotated

from fastapi import Depends, HTTPException
from tortoise.exceptions import IntegrityError

from api_server.authenticator import user_dep
from api_server.fast_io import FastIORouter
from api_server.models import TaskFavorite, User
from api_server.models import tortoise_models as ttm

router = FastIORouter(tags=["Tasks"])


@router.post("")
async def post_favorite_task(
    favorite_task: TaskFavorite,
    user: Annotated[User, Depends(user_dep)],
):
    try:
        await ttm.TaskFavorite.update_or_create(
            {
                "name": favorite_task.name,
                "unix_millis_earliest_start_time": favorite_task.unix_millis_earliest_start_time,
                "priority": favorite_task.priority if favorite_task.priority else None,
                "category": favorite_task.category,
                "description": (
                    favorite_task.description if favorite_task.description else None
                ),
                "user": user.username,
                "task_definition_id": favorite_task.task_definition_id,
            },
            id=favorite_task.id if favorite_task.id != "" else uuid.uuid4(),
        )
    except IntegrityError as e:
        raise HTTPException(422, str(e)) from e


@router.get("", response_model=list[TaskFavorite])
async def get_favorites_tasks(
    user: Annotated[User, Depends(user_dep)],
):
    favorites_tasks = await ttm.TaskFavorite.filter(user=user.username)
    return [
        TaskFavorite.model_validate(favorite_task) for favorite_task in favorites_tasks
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
