from typing import List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query
from pydantic import BaseModel
from tortoise.exceptions import IntegrityError
from tortoise.transactions import in_transaction

import api_server.models.tortoise_models as ttm
from api_server.authenticator import user_dep
from api_server.dependencies import pagination_query
from api_server.models import Pagination, Permission, User
from api_server.repositories.rmf import RmfRepository, rmf_repo_dep


class PostUsers(BaseModel):
    username: str
    is_admin: bool = False


class PostRoles(BaseModel):
    name: str


class PostMakeAdmin(BaseModel):
    admin: bool


async def _get_db_role(role_name: str) -> ttm.Role:
    db_role = await ttm.Role.get_or_none(name=role_name)
    if db_role is None:
        raise HTTPException(404)
    return db_role


async def _get_db_user(username: str) -> ttm.User:
    user = await ttm.User.get_or_none(username=username)
    if user is None:
        raise HTTPException(404)
    return user


def admin_dep(user: User = Depends(user_dep)):
    if not user.is_admin:
        raise HTTPException(403)


router = APIRouter(tags=["Admin"], dependencies=[Depends(admin_dep)])


@router.get("/users", response_model=list[str])
async def get_users(
    rmf_repo: RmfRepository = Depends(rmf_repo_dep),
    pagination: Pagination = Depends(pagination_query),
    username: Optional[str] = Query(
        None, description="filters username that starts with the value"
    ),
    is_admin: Optional[bool] = Query(None),
):
    """
    Search users
    """
    return await rmf_repo.query_users(pagination, username=username, is_admin=is_admin)


@router.post("/users")
async def create_user(body: PostUsers):
    """
    Create a user
    """
    await ttm.User.create(username=body.username, is_admin=body.is_admin)


@router.get("/users/{username}", response_model=User)
async def get_user(username: str):
    """
    Get a user
    """
    # checks if the user exist in the database
    await _get_db_user(username)
    return await User.load_or_create_from_db(username)


@router.delete("/users/{username}")
async def delete_user(username: str):
    """
    Delete a user

    This only performs a soft delete, while the user is deleted from the app database,
    it still exists in the idp so they can still log in, the user will then be re-created
    with the default permissions.
    """
    user = await _get_db_user(username)
    await user.delete()


@router.post("/users/{username}/make_admin")
async def make_admin(username: str, body: PostMakeAdmin):
    """
    Make or remove admin privilege from a user
    """
    user = await _get_db_user(username)
    user.is_admin = body.admin
    await user.save()


@router.post("/users/{username}/roles")
async def add_user_role(username: str, body: PostRoles):
    """
    Add role to a user
    """
    user = await _get_db_user(username)
    await user.fetch_related("roles")
    role = await _get_db_role(body.name)
    await user.roles.add(role)


@router.put("/users/{username}/roles")
async def set_user_roles(username: str, body: List[PostRoles]):
    """
    Set the roles of a user
    """
    user = await _get_db_user(username)
    async with in_transaction():
        role_names = [r.name for r in body]
        roles = await ttm.Role.filter(name__in=role_names)
        if len(roles) != len(role_names):
            raise HTTPException(422, "one or more roles does not exist")
        await user.roles.clear()
        await user.roles.add(*roles)


@router.delete("/users/{username}/roles/{role}")
async def delete_user_role(username: str, role: str):
    """
    Remove role from a user
    """
    user = await _get_db_user(username)
    await user.fetch_related("roles")
    db_role = await _get_db_role(role)
    await user.roles.remove(db_role)


@router.get("/roles", response_model=List[str])
async def get_roles():
    """
    Get all roles
    """
    return await ttm.Role.all().values_list("name", flat=True)


@router.post("/roles")
async def create_role(body: PostRoles):
    """
    Create a new role
    """
    try:
        await ttm.Role.create(name=body.name)
    except IntegrityError as e:
        raise HTTPException(422, str(e)) from e


@router.delete("/roles/{role}")
async def delete_role(role: str):
    """
    Delete a role
    """
    db_role = await _get_db_role(role)
    await db_role.delete()


@router.get("/roles/{role}/permissions", response_model=List[Permission])
async def get_role_permissions(role: str):
    """
    Get all permissions of a role
    """
    db_role = await _get_db_role(role)
    permissions = await ttm.ResourcePermission.filter(role=db_role).prefetch_related(
        "role"
    )
    return [Permission(authz_grp=p.authz_grp, action=p.action) for p in permissions]


@router.post("/roles/{role}/permissions")
async def add_role_permission(role: str, body: Permission):
    """
    Add a permission to a role
    """
    db_role = await _get_db_role(role)
    await ttm.ResourcePermission.update_or_create(
        authz_grp=body.authz_grp,
        role=db_role,
        action=body.action,
    )


@router.post("/roles/{role}/permissions/remove")
async def remove_role_permission(role: str, permission: Permission):
    """
    Delete a permission from a role
    """
    db_role = await _get_db_role(role)
    perm = await ttm.ResourcePermission.get_or_none(
        role=db_role, authz_grp=permission.authz_grp, action=permission.action
    )
    if perm:
        await perm.delete()
