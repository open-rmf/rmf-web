from pydantic import BaseModel, ConfigDict

from . import tortoise_models as ttm


class User(BaseModel):
    model_config = ConfigDict(
        json_schema_extra={"required": ["username", "is_admin", "roles"]}
    )
    username: str
    is_admin: bool = False
    roles: list[str] = []

    @staticmethod
    async def load_or_create_from_db(username: str) -> "User":
        """
        Loads an user from db, creates the user if it does not exist.
        NOTE: This should only be called after verifying the username comes from a trusted source (e.g. after verifying the jwt).
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
