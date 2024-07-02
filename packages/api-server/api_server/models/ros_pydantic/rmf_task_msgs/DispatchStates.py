# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.DispatchState import DispatchState


class DispatchStates(pydantic.BaseModel):
    active: List[DispatchState] = []  # rmf_task_msgs/DispatchState
    finished: List[DispatchState] = []  # rmf_task_msgs/DispatchState

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "active",
                "finished",
            ],
        }


# # States of tasks that are currently in the process of being dispatched
# DispatchState[] active
#
# # States of tasks that have recently finished being dispatched. This may mean
# # the task was assigned or it may mean it failed to be dispatched or was
# # canceled before the dispatch took place.
# DispatchState[] finished
