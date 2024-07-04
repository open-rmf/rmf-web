from typing import Annotated

from fastapi import APIRouter, Depends

import api_server.ros_time
from api_server.authenticator import user_dep
from api_server.models import Permission, User
from api_server.models.tortoise_models import ResourcePermission

router = APIRouter()


@router.get("/user", response_model=User)
async def get_user(user: Annotated[User, Depends(user_dep)]):
    """
    Get the currently logged in user
    """
    return user


@router.get("/permissions", response_model=list[Permission])
async def get_effective_permissions(user: Annotated[User, Depends(user_dep)]):
    """
    Get the effective permissions of the current user
    """
    perms = (
        await ResourcePermission.filter(role__name__in=user.roles)
        .distinct()
        .values("authz_grp", "action")
    )
    return [Permission(authz_grp=p["authz_grp"], action=p["action"]) for p in perms]


@router.get("/time", response_model=int)
async def get_time():
    """
    Get the current rmf time in unix milliseconds
    """
    return api_server.ros_time.now()
