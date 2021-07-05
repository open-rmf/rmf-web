import unittest

from tortoise import Tortoise
from tortoise.fields import CharField, ForeignKeyField, ForeignKeyRelation
from tortoise.models import Model as TortoiseModel

from .models import User
from .models.tortoise_models import ProtectedResource, ResourcePermission
from .permissions import BasicAction, Enforcer, RmfRole


class Greeting(TortoiseModel, ProtectedResource):
    message = CharField(255, pk=True)


class GreetingPermission(TortoiseModel, ResourcePermission):
    obj: ForeignKeyRelation[Greeting] = ForeignKeyField(
        "models.Greeting", "permissions"
    )


class TestPermissions(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["api_server.test_permissions"]},
        )
        await Tortoise.generate_schemas()

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_user_outside_group_cannot_see_resource(self):
        owner = User(username="test_owner", groups=["greeter"])
        greeting = Greeting(message="hello", owner=owner.username)
        await greeting.save()
        await Enforcer.save_permissions(greeting, owner.groups, [BasicAction.Read])

        q = Enforcer.query(owner, Greeting).count()
        self.assertEqual(1, await q)

        # unrelated user cannot see the resource
        user = User(username="test_user")
        q = Enforcer.query(user, Greeting).count()
        self.assertEqual(0, await q)

        # users with the same groups can see the resource
        user2 = User(username="test_user2", groups=["greeter"])
        q = Enforcer.query(user2, Greeting).count()
        self.assertEqual(1, await q)

    async def test_user_authorization(self):
        owner = User(username="test_owner", groups=["greeter"])
        greeting = Greeting(message="hello", owner=owner.username)
        await greeting.save()
        await Enforcer.save_permissions(
            greeting, owner.groups, [BasicAction.Read, BasicAction.Write]
        )

        user = User(username="test_user", groups=["greeter"])
        authorized = await Enforcer.is_authorized(greeting, user, BasicAction.Write)
        self.assertTrue(authorized)

        user2 = User(username="test_user2")
        authorized = await Enforcer.is_authorized(greeting, user2, BasicAction.Write)
        self.assertFalse(authorized)

    async def test_owner_has_full_access(self):
        owner = User(username="test_owner", groups=["greeter"])
        greeting = Greeting(message="hello", owner=owner.username)
        await greeting.save()
        await Enforcer.save_permissions(greeting, owner.groups, [BasicAction.Read])
        authorized = await Enforcer.is_authorized(greeting, owner, BasicAction.Write)
        self.assertTrue(authorized)

    async def test_user_without_groups_can_see_own_resources(self):
        owner = User(username="test_owner")
        greeting = Greeting(message="hello", owner=owner.username)
        await greeting.save()
        await Enforcer.save_permissions(greeting, owner.groups, [BasicAction.Read])
        count = await Enforcer.query(owner, Greeting).count()
        self.assertEqual(1, count)

    async def test_superadmin_can_see_resource_with_no_owner(self):
        greeting = Greeting(message="hello")
        await greeting.save()
        user = User(username="test_superadmin", roles=[RmfRole.Admin])
        count = await Enforcer.query(user, Greeting).count()
        self.assertEqual(1, count)
