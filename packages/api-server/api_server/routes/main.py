from typing import List

import api_server.models.tortoise_models as ttm
from api_server.base_app import BaseApp
from api_server.models import Permission, User
from fastapi import APIRouter, Depends


def main_router(app: BaseApp):
    router = APIRouter()
    user_dep = app.auth_dep

    @router.get("/user", response_model=User)
    async def get_user(user: User = Depends(user_dep)):
        """
        Get the currently logged in user
        """
        return user

    @router.get("/permissions", response_model=List[Permission])
    async def get_effective_permissions(user: User = Depends(user_dep)):
        """
        Get the effective permissions of the current user
        """
        perms = (
            await ttm.ResourcePermission.filter(role__name__in=user.roles)
            .distinct()
            .values("authz_grp", "action")
        )
        return [
            Permission.construct(authz_grp=p["authz_grp"], action=p["action"])
            for p in perms
        ]

    return router
