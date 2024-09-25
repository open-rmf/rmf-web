from typing import Annotated

import fastapi
import pydantic
import tortoise
import tortoise.exceptions
from fastapi import Depends, HTTPException

import api_server.models as mdl
import api_server.models.tortoise_models as ttm
from api_server.dependencies import pagination_query
from api_server.fast_io import FastIORouter

router = FastIORouter(tags=["Missions"])


class CreateMission(pydantic.BaseModel):
    name: str
    ui_schema: str = pydantic.Field(
        ..., description="ui schema according to https://jsonforms.io"
    )
    task_template: str = pydantic.Field(..., description="jinja2 template")


@router.post("/create_mission", status_code=201)
async def create_mission(mission: CreateMission):
    try:
        db_mission = await ttm.Mission.create(
            name=mission.name,
            ui_schema=mission.ui_schema,
            task_template=mission.task_template,
        )
        return mdl.Mission.model_validate(db_mission)
    except tortoise.exceptions.IntegrityError as e:
        raise fastapi.HTTPException(409) from e


@router.get("", response_model=list[mdl.Mission])
async def get_all_missions(
    pagination: Annotated[mdl.Pagination, Depends(pagination_query)],
    name: str | None = None,
):
    q = (
        ttm.Mission.all()
        .limit(pagination.limit)
        .offset(pagination.offset)
        .order_by(*pagination.order_by)
    )
    if name is not None:
        q = q.filter(name=name)
    db_missions = await q
    return [mdl.Mission.model_validate(m) for m in db_missions]


@router.get("/{id}", response_model=mdl.Mission)
async def get_mission(id: int):
    try:
        db_mission = await ttm.Mission.get(id=id)
        return mdl.Mission.model_validate(db_mission)
    except tortoise.exceptions.DoesNotExist as e:
        raise HTTPException(404) from e


def to_db_mission(id: int, mission: CreateMission):
    return ttm.Mission(
        id=id,
        name=mission.name,
        ui_schema=mission.ui_schema,
        task_template=mission.task_template,
    )


@router.put("/{id}", response_model=None)
async def update_mission(id: int, mission: CreateMission):
    db_mission = to_db_mission(id, mission)
    try:
        await db_mission.save(force_update=True)
    except tortoise.exceptions.IntegrityError as e:
        raise HTTPException(404) from e


@router.delete("/{id}", response_model=None)
async def delete_mission(id: int):
    try:
        db_mission = await ttm.Mission.get(id=id)
        await db_mission.delete()
    except tortoise.exceptions.DoesNotExist as e:
        raise HTTPException(404) from e


# @router.post("/{id}/render_mission", response_model=dict)
# async def render_mission(id: int, inputs: dict):
#     try:
#         tmpl = (await ttm.Mission.get(id=id).values_list("task_template"))[0]
#         # TODO: render the task with jinja2 and return the result
#     except tortoise.exceptions.DoesNotExist:
#         raise HTTPException(404)
#     raise NotImplementedError()
