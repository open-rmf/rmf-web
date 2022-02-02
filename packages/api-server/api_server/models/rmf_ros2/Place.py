# generated by datamodel-codegen:
#   filename:  Place.json

from __future__ import annotations

from typing import Optional, Union

from pydantic import BaseModel, Field, conint


class Waypoint(BaseModel):
    __root__: Union[str, conint(ge=0)]


class PlaceDescriptionItem(BaseModel):
    waypoint: Waypoint
    orientation: Optional[float] = None


class PlaceDescription(BaseModel):
    __root__: Union[Waypoint, PlaceDescriptionItem] = Field(
        ...,
        description="Description of a place that the robot can go to",
        title="Place Description",
    )
