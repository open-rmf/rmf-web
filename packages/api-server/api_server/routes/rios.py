from typing import Annotated

from fastapi import Query, Response

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Rio
from api_server.models.tortoise_models import Rio as DbRio
from api_server.rmf_io import rio_events

router = FastIORouter(tags=["RIOs"])


@router.get("", response_model=list[Rio])
async def query_rios(
    id: Annotated[str | None, Query(description="comma separated list of ids")] = None,
    type: Annotated[
        str | None, Query(description="comma separated list of types")
    ] = None,
):
    filters = {}
    if id:
        filters["id__in"] = id.split(",")
    if type:
        filters["type__in"] = type.split(",")

    rios = await DbRio.filter(**filters)
    return [Rio.model_validate(x) for x in rios]


@router.sub("", response_model=Rio)
async def sub_rio(_req: SubscriptionRequest):
    return rio_events.rios


@router.put("", response_model=None)
async def put_rio(rio: Rio, resp: Response):
    rio_dict = rio.model_dump()
    del rio_dict["id"]
    _, created = await DbRio.update_or_create(rio_dict, id=rio.id)
    if created:
        resp.status_code = 201
    rio_events.rios.on_next(rio)
