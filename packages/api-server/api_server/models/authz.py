from pydantic import BaseModel


class Permission(BaseModel):
    id: int
    authz_grp: str
    action: str
