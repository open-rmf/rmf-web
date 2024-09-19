from fastapi import HTTPException

from . import tortoise_models as ttm
from .base import PydanticModel


class User(PydanticModel):
    # FIXME(koonpeng): We should use the "userId" as the identifier. Some idP may allow
    # duplicated usernames.
    # userId: str
    username: str
    is_admin: bool = False
    roles: list[str] = []

    @staticmethod
    async def load_or_create_from_db(username: str) -> "User":
        """
        Loads an user from db, creates the user if it does not exist.
        NOTE: This should only be called after verifying the username comes from
        a trusted source (e.g. after verifying the jwt).
        """
        user = await User.load_from_db(username)
        if user is None:
            ttm_user, _ = await ttm.User.get_or_create(
                {"is_admin": False}, username=username
            )
            return await User.from_db(ttm_user)
        return user

    @staticmethod
    async def load_from_db(username: str) -> "User | None":
        if username.startswith("_"):
            raise ValueError("username cannot start with '_'")
        ttm_user = await ttm.User.get_or_none(username=username)
        if ttm_user is None:
            return None
        return await User.from_db(ttm_user)

    @staticmethod
    async def from_db(db_user: ttm.User) -> "User":
        """
        Convert a db user model to a pydantic user model.
        """
        await db_user.fetch_related("roles")
        return User(
            username=db_user.username,
            is_admin=db_user.is_admin,
            roles=[r.name for r in db_user.roles],
        )

    @staticmethod
    def get_system_user() -> "User":
        """Return a dummy user to be used for system operations"""
        return _system_user

    async def update_admin(self, is_admin: bool):
        ttm_user = await ttm.User.get_or_none(username=self.username)
        if ttm_user is None:
            raise HTTPException(status_code=404)

        ttm_user.update_from_dict({"is_admin": is_admin})
        await ttm_user.save()
        self.is_admin = is_admin


_system_user = User(username="__system__", is_admin=True)
