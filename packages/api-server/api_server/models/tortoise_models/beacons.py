from tortoise.contrib.pydantic.creator import pydantic_model_creator
from tortoise.fields import BooleanField, CharField
from tortoise.models import Model


class BeaconState(Model):
    id = CharField(255, pk=True)
    online = BooleanField(index=True)
    category = CharField(255, null=True, index=True)
    activated = BooleanField(index=True)
    level = CharField(255, null=True, index=True)


BeaconStatePydantic = pydantic_model_creator(BeaconState)
