from typing import Annotated

from fastapi import Depends, Query, Response

from api_server.fast_io import FastIORouter, SubscriptionRequest
from api_server.models import Rio
from api_server.models.tortoise_models import Rio as DbRio
from api_server.rmf_io import RioEvents, get_rio_events

router = FastIORouter(tags=["RIOs"])


@router.get("", response_model=list[Rio])
async def query_rios(
    id_: Annotated[
        str | None, Query(alias="id", description="comma separated list of ids")
    ] = None,
    type_: Annotated[
        str | None, Query(alias="type", description="comma separated list of types")
    ] = None,
):
    filters = {}
    if id_:
        filters["id__in"] = id_.split(",")
    if type_:
        filters["type__in"] = type_.split(",")

    rios = await DbRio.filter(**filters)
    return [Rio.model_validate(x) for x in rios]


@router.sub("", response_model=Rio)
async def sub_rio(_req: SubscriptionRequest):
    rio_events = get_rio_events()
    return rio_events.rios


@router.put("", response_model=None)
async def put_rio(
    rio: Rio,
    resp: Response,
    rio_events: Annotated[RioEvents, Depends(get_rio_events)],
):
    rio_dict = rio.model_dump()
    del rio_dict["id"]
    _, created = await DbRio.update_or_create(rio_dict, id=rio.id)
    if created:
        resp.status_code = 201
    rio_events.rios.on_next(rio)
