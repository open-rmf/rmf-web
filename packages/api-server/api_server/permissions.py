from enum import Enum
from typing import Sequence, TypeVar

from tortoise.queryset import QuerySet
from tortoise.transactions import in_transaction

from .models import User
from .models.tortoise_models import ProtectedResourceModel

ProtectedResourceT = TypeVar("ProtectedResourceT", bound=ProtectedResourceModel)


class Permission(Enum):
    Read = "read"
    Write = "write"


class Enforcer:
    @staticmethod
    def _is_superadmin(user: User):
        return "_rmf_superadmin" in user.roles

    @staticmethod
    async def is_authorized(
        r: ProtectedResourceModel, user: User, permission: Permission
    ):
        if r.owner == user.username:
            return True

        perm = await r.permissions.remote_model.filter(
            role__in=user.roles, permission=permission
        ).first()
        return perm is not None

    @staticmethod
    def can_submit_task(user: User):
        return Enforcer._is_superadmin(user) or "_rmf_task_submit" in user.roles

    @staticmethod
    def query(
        user: User, r: ProtectedResourceModel
    ) -> QuerySet[ProtectedResourceModel]:
        return r.filter(
            permissions__role__in=user.roles, permissions__permission=Permission.Read
        )

    @staticmethod
    async def save(
        r: ProtectedResourceModel, owner: User, permissions: Sequence[Permission] = None
    ):
        """
        Saves a protected resource along with it's permissions into the database.
        All roles that the owner has will be given the same set of permissions.
        """
        permissions = permissions or [Permission.Read]
        async with in_transaction():
            r.owner = owner.username
            await r.save()
            for role in owner.roles:
                for perm in permissions:
                    await r.permissions.remote_model(
                        resource=r, role=role, permission=perm
                    ).save()
