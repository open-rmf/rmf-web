from tortoise.fields import ForeignKeyRelation, ReverseRelation, TextField
from tortoise.models import Model


class ResourcePermission:
    obj: ForeignKeyRelation["Types.ProtectedResourceModel"]
    sub = TextField()
    act = TextField()


class ProtectedResource:
    owner = TextField(null=True)
    permissions: ReverseRelation["Types.ResourcePermissionModel"]


# Nest the type definitions in a container class to prevent tortoise from generating schemas for them.
class Types:
    class ResourcePermissionModel(Model, ResourcePermission):
        pass

    class ProtectedResourceModel(Model, ProtectedResource):
        pass
