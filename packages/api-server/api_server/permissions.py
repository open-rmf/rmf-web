from typing import List, TypeVar, cast

from tortoise.expressions import Subquery
from tortoise.models import Model as TortoiseModel
from tortoise.queryset import QuerySet

from .models import User
from .models import tortoise_models as ttm

ResourceT = TypeVar("ResourceT", bound=TortoiseModel)


class RmfAction:
    TaskRead = "task_read"
    TaskSubmit = "task_submit"
    TaskCancel = "task_cancel"


class Enforcer:
    @staticmethod
    async def is_authorized(user: User, authz_grp: str, action: str):
        """
        Checks if an user has permission to perform an action.
        """
        if user.is_admin:
            return True
        perm = await ttm.ResourcePermission.filter(
            authz_grp=authz_grp, role__name__in=user.roles, action=action
        ).first()
        return perm is not None

    @staticmethod
    def query(
        user: User,
        resource: QuerySet[ResourceT],
        action: str,
    ) -> QuerySet[ResourceT]:
        """
        Augments a query with the read permissions of an user, this returns a query
        that only returns results that the user can see. The admin roles can always see
        all resources.

        @param resource: A query set of a model which implements `ProtectedResource`.
        """
        if user.is_admin:
            return resource
        permissions = ttm.ResourcePermission.filter(
            action=action, role__name__in=user.roles
        ).values("authz_grp")
        return resource.filter(authz_grp__in=Subquery(permissions))

    @staticmethod
    async def get_effective_permissions(user: User, authz_grp: str) -> List[str]:
        return cast(
            List[str],
            await ttm.ResourcePermission.filter(
                authz_grp=authz_grp, role__name__in=user.roles
            ).values_list("action", flat=True),
        )
