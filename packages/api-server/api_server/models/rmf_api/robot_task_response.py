# generated by datamodel-codegen:
#   filename:  robot_task_response.json

from __future__ import annotations

from pydantic import Field, RootModel

from . import dispatch_task_response


class RobotTaskResponse(RootModel[dispatch_task_response.TaskDispatchResponse]):
    root: dispatch_task_response.TaskDispatchResponse = Field(
        ..., description="Response to a robot task request", title="Robot Task Response"
    )
