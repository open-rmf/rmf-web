# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..rmf_task_msgs.BidProposal import BidProposal


class BidResponse(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str  # string
    has_proposal: bool  # bool
    proposal: BidProposal  # rmf_task_msgs/BidProposal
    errors: list[str]  # string


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
