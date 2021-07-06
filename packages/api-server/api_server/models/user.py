from typing import List

from pydantic import BaseModel

from . import tortoise_models as ttm


class User(BaseModel):
    username: str
    is_admin: bool = False
    roles: List[str] = list()

    @staticmethod
    async def load_from_db(username: str) -> "User":
        ttm_user = await ttm.User.get(username=username).prefetch_related("roles")
        return User(
            username=username,
            is_admin=ttm_user.is_admin,
            roles=[r.name for r in ttm_user.roles],
        )
