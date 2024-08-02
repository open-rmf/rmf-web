# generated by datamodel-codegen:
#   filename:  task_description__delivery.json

from __future__ import annotations

from pydantic import BaseModel

from . import event_description__dropoff, event_description__pickup


class DeliveryTaskDescription(BaseModel):
    pickup: event_description__pickup.PickUpEventDescription
    dropoff: event_description__dropoff.DropOffEventDescription