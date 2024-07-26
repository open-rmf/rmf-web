# This is a generated file, do not edit

from typing import Annotated

import pydantic

from .BidProposal import BidProposal as rmf_task_msgs_msg_BidProposal


class BidResponse(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    task_id: str
    has_proposal: bool
    proposal: rmf_task_msgs_msg_BidProposal
    errors: list[str]
