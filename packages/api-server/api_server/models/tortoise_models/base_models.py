from tortoise.models import Model

from .authorization import ProtectedResource


# Nest the type definitions in a container class to prevent tortoise from generating schemas for them.
class BaseModels:
    class ProtectedResourceModel(Model, ProtectedResource):
        pass
