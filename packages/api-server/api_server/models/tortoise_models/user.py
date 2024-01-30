from tortoise.fields.data import BooleanField, CharField
from tortoise.fields.relational import ManyToManyField, ManyToManyRelation
from tortoise.models import Model

from .authorization import Role


class User(Model):
    username: str = CharField(255, pk=True)  # type: ignore
    is_admin: bool = BooleanField()  # type: ignore
    roles: ManyToManyRelation[Role] = ManyToManyField("models.Role")
