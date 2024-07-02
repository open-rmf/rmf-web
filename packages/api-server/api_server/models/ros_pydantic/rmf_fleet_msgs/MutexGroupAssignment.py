# This is a generated file, do not edit

from typing import Annotated

import pydantic

from ..builtin_interfaces.Time import Time


class MutexGroupAssignment(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    group: str  # string
    claimant: Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]  # uint64
    claim_time: Time  # builtin_interfaces/Time


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
