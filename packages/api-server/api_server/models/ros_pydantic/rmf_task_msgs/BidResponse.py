# This is a generated file, do not edit

from typing import List

import pydantic

from ..rmf_task_msgs.BidProposal import BidProposal


class BidResponse(pydantic.BaseModel):
    task_id: str = ""  # string
    has_proposal: bool = False  # bool
    proposal: BidProposal = BidProposal()  # rmf_task_msgs/BidProposal
    errors: List[str] = []  # string

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "task_id",
                "has_proposal",
                "proposal",
                "errors",
            ],
        }


# # ID of the task that is being bid on
# string task_id
#
# # True if this response contains a proposal
# bool has_proposal
#
# # The proposal of this response, if has_proposal is true
# BidProposal proposal
#
# # Any errors related to this bid
# string[] errors
