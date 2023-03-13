# generated by datamodel-codegen:
#   filename:  task_state.json

from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional, Union

from pydantic import BaseModel, Field, conint

from . import error


class AssignedTo(BaseModel):
    group: str
    name: str


class Cancellation(BaseModel):
    unix_millis_request_time: int = Field(
        ..., description="The time that the cancellation request arrived"
    )
    labels: List[str] = Field(..., description="Labels to describe the cancel request")


class Killed(BaseModel):
    unix_millis_request_time: int = Field(
        ..., description="The time that the cancellation request arrived"
    )
    labels: List[str] = Field(..., description="Labels to describe the kill request")


class Booking(BaseModel):
    id: str = Field(..., description="The unique identifier for this task")
    unix_millis_earliest_start_time: Optional[int] = None
    priority: Optional[Union[Dict[str, Any], str]] = Field(
        None, description="Priority information about this task"
    )
    labels: Optional[List[str]] = Field(
        None, description="Information about how and why this task was booked"
    )


class Category(BaseModel):
    __root__: str = Field(..., description="The category of this task or phase")


class Detail(BaseModel):
    __root__: Union[Dict[str, Any], List, str] = Field(
        ..., description="Detailed information about a task, phase, or event"
    )


class Status1(Enum):
    queued = "queued"
    selected = "selected"
    dispatched = "dispatched"
    failed_to_assign = "failed_to_assign"
    canceled_in_flight = "canceled_in_flight"


class Assignment(BaseModel):
    fleet_name: Optional[str] = None
    expected_robot_name: Optional[str] = None


class EstimateMillis(BaseModel):
    __root__: conint(ge=0) = Field(
        ...,
        description="An estimate, in milliseconds, of how long the subject will take to complete",
    )


class Id(BaseModel):
    __root__: conint(ge=0)


class ResumedBy(BaseModel):
    unix_millis_request_time: Optional[int] = Field(
        None, description="The time that the resume request arrived"
    )
    labels: List[str] = Field(..., description="Labels to describe the resume request")


class Interruption(BaseModel):
    unix_millis_request_time: int = Field(
        ..., description="The time that the interruption request arrived"
    )
    labels: List[str] = Field(
        ..., description="Labels to describe the purpose of the interruption"
    )
    resumed_by: Optional[ResumedBy] = Field(
        None,
        description="Information about the resume request that ended this interruption. This field will be missing if the interruption is still active.",
    )


class Status(Enum):
    uninitialized = "uninitialized"
    blocked = "blocked"
    error = "error"
    failed = "failed"
    queued = "queued"
    standby = "standby"
    underway = "underway"
    delayed = "delayed"
    skipped = "skipped"
    canceled = "canceled"
    killed = "killed"
    completed = "completed"


class EventState(BaseModel):
    id: Id
    status: Optional[Status] = None
    name: Optional[str] = Field(None, description="The brief name of the event")
    detail: Optional[Detail] = Field(
        None, description="Detailed information about the event"
    )
    deps: Optional[List[conint(ge=0)]] = Field(
        None,
        description="This event may depend on other events. This array contains the IDs of those other event dependencies.",
    )


class Undo(BaseModel):
    unix_millis_request_time: int = Field(
        ..., description="The time that the undo skip request arrived"
    )
    labels: List[str] = Field(
        ..., description="Labels to describe the undo skip request"
    )


class SkipPhaseRequest(BaseModel):
    unix_millis_request_time: int = Field(
        ..., description="The time that the skip request arrived"
    )
    labels: List[str] = Field(
        ..., description="Labels to describe the purpose of the skip request"
    )
    undo: Optional[Undo] = Field(
        None,
        description="Information about an undo skip request that applied to this request",
    )


class Dispatch(BaseModel):
    status: Status1
    assignment: Optional[Assignment] = None
    errors: Optional[List[error.Error]] = None


class Phase(BaseModel):
    id: Id
    category: Optional[Category] = None
    detail: Optional[Detail] = None
    unix_millis_start_time: Optional[int] = None
    unix_millis_finish_time: Optional[int] = None
    original_estimate_millis: Optional[EstimateMillis] = None
    estimate_millis: Optional[EstimateMillis] = None
    final_event_id: Optional[Id] = None
    events: Optional[Dict[str, EventState]] = Field(
        None,
        description="A dictionary of events for this phase. The keys (property names) are the event IDs, which are integers.",
    )
    skip_requests: Optional[Dict[str, SkipPhaseRequest]] = Field(
        None, description="Information about any skip requests that have been received"
    )


class TaskState(BaseModel):
    booking: Booking
    category: Optional[Category] = None
    detail: Optional[Detail] = None
    unix_millis_start_time: Optional[int] = None
    unix_millis_finish_time: Optional[int] = None
    original_estimate_millis: Optional[EstimateMillis] = None
    estimate_millis: Optional[EstimateMillis] = None
    assigned_to: Optional[AssignedTo] = Field(
        None, description="Which agent (robot) is the task assigned to"
    )
    status: Optional[Status] = None
    unix_millis_request_time: Optional[int] = None
    dispatch: Optional[Dispatch] = None
    phases: Optional[Dict[str, Phase]] = Field(
        None,
        description="A dictionary of the states of the phases of the task. The keys (property names) are phase IDs, which are integers.",
    )
    completed: Optional[List[Id]] = Field(
        None, description="An array of the IDs of completed phases of this task"
    )
    active: Optional[Id] = Field(
        None, description="The ID of the active phase for this task"
    )
    pending: Optional[List[Id]] = Field(
        None, description="An array of the pending phases of this task"
    )
    interruptions: Optional[Dict[str, Interruption]] = Field(
        None,
        description="A dictionary of interruptions that have been applied to this task. The keys (property names) are the unique token of the interruption request.",
    )
    cancellation: Optional[Cancellation] = Field(
        None,
        description="If the task was cancelled, this will describe information about the request.",
    )
    killed: Optional[Killed] = Field(
        None,
        description="If the task was killed, this will describe information about the request.",
    )
