# This is a generated file, do not edit

from typing import List

import pydantic


class CancelTask_Request(pydantic.BaseModel):
    requester: str = ""  # string
    task_id: str = ""  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "requester",
                "task_id",
            ],
        }


# # Cancel Task | "Delete" service call
#
# # Identifier for who is requesting the service
# string requester
#
# # generated task ID by dispatcher node
# string task_id
#
