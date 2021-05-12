from enum import Enum

from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class AuthEvents(models.Model):
    id = fields.IntField(pk=True)
    # user = fields.ForeignKey(User, null=True)
    username = fields.TextField(null=True)
    user_keycloak_id = fields.TextField(null=True)
    event_type = fields.TextField()
    realm_id = fields.TextField(null=True)
    client_id = fields.TextField(null=True)
    ip_address = fields.TextField(null=True)
    payload = fields.JSONField()
    created = fields.DatetimeField(auto_now_add=True)

    def __str__(self):
        return str(self.event_type)


AuthEvents_Pydantic = pydantic_model_creator(AuthEvents, name="AuthEvents")
