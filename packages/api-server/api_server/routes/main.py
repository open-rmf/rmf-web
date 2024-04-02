from typing import List

from fastapi import APIRouter, Depends

from api_server import clock
from api_server.authenticator import user_dep
from api_server.models import Permission, User
from api_server.models.tortoise_models import ResourcePermission

router = APIRouter()


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
        await ResourcePermission.filter(role__name__in=user.roles)
        .distinct()
        .values("authz_grp", "action")
    )
    return [
        Permission.model_construct(authz_grp=p["authz_grp"], action=p["action"])
        for p in perms
    ]


@router.get("/time", response_model=int)
async def get_time():
    """
    Get the current rmf time in unix milliseconds
    """
    return clock.now()
