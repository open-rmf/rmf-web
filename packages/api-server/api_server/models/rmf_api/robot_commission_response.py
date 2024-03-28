# generated by datamodel-codegen:
#   filename:  robot_commission_response.json

from __future__ import annotations

from typing import List, Optional, Union

from pydantic import BaseModel, Field
from typing_extensions import Literal

from . import error


class ResultItem(BaseModel):
    success: Literal[True]


class ResultItem1(BaseModel):
    success: Literal[False]
    errors: Optional[List[error.Error]] = Field(
        None, description="Any error messages explaining why the request failed"
    )


class Result(BaseModel):
    __root__: Union[ResultItem, ResultItem1]


class RobotCommissionResponse(BaseModel):
    commission: Result
    pending_dispatch_tasks_policy: Optional[Result] = None
    pending_direct_tasks_policy: Optional[Result] = None
