from typing import Callable

from fastapi import APIRouter, Depends

from ..models import User


def main_router(user_dep: Callable[..., User]):
    router = APIRouter()

    @router.get("/user", response_model=User)
    async def get_user(user: User = Depends(user_dep)):
        """
        Get the currently logged in user
        """
        return user

    return router
