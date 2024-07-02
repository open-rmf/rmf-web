# generated by datamodel-codegen:
#   filename:  task_description__clean.json

from __future__ import annotations

from pydantic import Field, RootModel

from . import event_description__clean


class CleanTask(RootModel[event_description__clean.CleanEvent]):
    root: event_description__clean.CleanEvent = Field(
        ..., description="Clean a zone", title="Clean Task"
    )
