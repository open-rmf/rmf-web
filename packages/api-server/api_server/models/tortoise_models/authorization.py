from tortoise.fields.data import CharField
from tortoise.fields.relational import (
    ForeignKeyField,
    ForeignKeyRelation,
    ReverseRelation,
)
from tortoise.models import Model


class Role(Model):
    name: str = CharField(255, pk=True)  # type: ignore
    permissions: ReverseRelation["ResourcePermission"]


class ResourcePermission(Model):
    # "obj" in casbin speak
    # This has no foreign key because resources can be given any arbitrary group, sometimes even dynamically.
    authz_grp: str = CharField(255, index=True)  # type: ignore
    # "sub" in casbin speak
    role: ForeignKeyRelation[Role] = ForeignKeyField("models.Role")
    action: str = CharField(255)  # type: ignore


class ProtectedResource:
    authz_grp = CharField(255, null=True, index=True)
