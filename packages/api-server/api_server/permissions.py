from typing import Sequence, TypeVar

from tortoise.query_utils import Q
from tortoise.queryset import QuerySet
from tortoise.transactions import in_transaction

from .models import User
from .models.tortoise_models import ProtectedResource, Types

ProtectedResourceT = TypeVar("ProtectedResourceT", bound=Types.ProtectedResourceModel)


Action = str


class BasicAction:
    Read = "read"
    Write = "write"


class RmfRole:
    Admin = "_rmf_superadmin"
    TaskSubmit = "_rmf_task_submit"
    TaskCancel = "_rmf_task_cancel"


class Enforcer:
    @staticmethod
    async def is_authorized(
        r: Types.ProtectedResourceModel, user: User, action: Action
    ):
        """
        Checks if an user has permission to perform an action, the owner of a resource
        and the users with the admin role can always permission any actions on it.
        """
        if r.owner == user.username or RmfRole.Admin in user.roles:
            return True

        subjects = [user.username]
        subjects.extend(user.groups)
        perm = await r.permissions.remote_model.filter(
            sub__in=subjects, act=action
        ).first()
        return perm is not None

    @staticmethod
    def _has_role(user: User, role: RmfRole) -> bool:
        return any((r in [RmfRole.Admin, role] for r in user.roles))

    @staticmethod
    def can_submit_task(user: User) -> bool:
        return Enforcer._has_role(user, RmfRole.TaskSubmit)

    @staticmethod
    def can_cancel_task(user: User):
        return Enforcer._has_role(user, RmfRole.TaskCancel)

    @staticmethod
    def query(
        user: User,
        r: QuerySet[ProtectedResource],
    ) -> QuerySet[Types.ProtectedResourceModel]:
        """
        Augments a query with the read permissions of an user, this returns a query
        that only returns results that the user can see. The admin roles can always see
        all resources.
        """
        if RmfRole.Admin in user.roles:
            return r.all()
        subjects = [user.username]
        subjects.extend(user.groups)
        return r.filter(
            Q(
                permissions__sub__in=user.groups,
                permissions__act=BasicAction.Read,
            )
            | Q(owner=user.username)
        )

    @staticmethod
    async def save_permissions(
        r: Types.ProtectedResourceModel,
        subjects: Sequence[str],
        actions: Sequence[Action],
    ):
        """
        Helper function to save the permissions for a group of subjects. All subjects
        will be given the same permissions.
        """
        async with in_transaction():
            for sub in subjects:
                for act in actions:
                    await r.permissions.remote_model.update_or_create(
                        obj=r, sub=sub, act=act
                    )
