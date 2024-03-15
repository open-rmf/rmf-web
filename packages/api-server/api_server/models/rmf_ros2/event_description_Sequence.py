# generated by datamodel-codegen:
#   filename:  event_description_Sequence.json

from __future__ import annotations

from typing import Any, List, Optional, Union

from pydantic import BaseModel, Field, RootModel


class ActivityArrayItem(BaseModel):
    category: Optional[str] = Field(
        default=None, description="The category of the activity"
    )
    description: Optional[Any] = Field(
        default=None,
        description="A description of the activity. This must match a schema supported by a fleet for the activity category.",
    )


class ActivityArray(RootModel[List[ActivityArrayItem]]):
    root: List[ActivityArrayItem] = Field(..., ge=1)


class ActivitySequence1(BaseModel):
    activities: ActivityArray
    category: Optional[str] = Field(
        default=None, description="Customize the category display for this sequence"
    )
    detail: Optional[Any] = Field(
        default=None, description="Customize the detail display for this sequence"
    )


class ActivitySequence(RootModel[Union[ActivityArray, ActivitySequence1]]):
    root: Union[ActivityArray, ActivitySequence1] = Field(
        ..., description="A sequence of activities", title="Activity Sequence"
    )
