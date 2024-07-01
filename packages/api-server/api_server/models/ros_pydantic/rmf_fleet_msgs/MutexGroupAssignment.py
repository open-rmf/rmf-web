# This is a generated file, do not edit

from typing import List

import pydantic

from ..builtin_interfaces.Time import Time


class MutexGroupAssignment(pydantic.BaseModel):
    group: str = ""  # string
    claimant: pydantic.conint(ge=0, le=18446744073709551615) = 0  # uint64
    claim_time: Time = Time()  # builtin_interfaces/Time

    class Config:
        orm_mode = True
        schema_extra = {
            "required": [
                "group",
                "claimant",
                "claim_time",
            ],
        }


# # This message maps a mutex group name to the name of an agent that is currently
# # holding the claim to that group.
#
# # Name of the mutex group that is being described.
# string group
#
# # Traffic Participant ID of the agent that has currently claimed the group.
# # If the group is unclaimed, this will be the max uint64 value.
# uint64 claimant
#
# # Time stamp of when the claim request began.
# builtin_interfaces/Time claim_time
