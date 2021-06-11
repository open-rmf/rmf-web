import unittest

from tortoise import Tortoise
from tortoise.fields import CharField, ForeignKeyField, ForeignKeyRelation
from tortoise.models import Model as TortoiseModel

from .models import User
from .models.tortoise_models import ProtectedResource, ResourcePermission
from .permissions import Enforcer, Permission


class GreetingPermission(TortoiseModel, ResourcePermission):
    resource: ForeignKeyRelation["Greeting"] = ForeignKeyField(
        "models.Greeting", "permissions"
    )


class Greeting(TortoiseModel, ProtectedResource):
    message = CharField(255, pk=True)


class TestPermissions(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        await Tortoise.init(
            db_url="sqlite://:memory:",
            modules={"models": ["api_server.test_permissions"]},
        )
        await Tortoise.generate_schemas()
        # print(len(Tortoise.describe_models()))

    async def asyncTearDown(self):
        await Tortoise.close_connections()

    async def test_user_without_role_cannot_see_resource(self):
        owner = User(username="test_owner", roles=["greeter"])
        greeting = Greeting(message="hello")
        await Enforcer.save(greeting, owner)

        q = Enforcer.query(owner, Greeting).count()
        self.assertEqual(1, await q)

        # unrelated user cannot see the resource
        user = User(username="test_user", roles=[])
        q = Enforcer.query(user, Greeting).count()
        self.assertEqual(0, await q)

        # users with the same roles can see the resource
        user2 = User(username="test_user2", roles=["greeter"])
        q = Enforcer.query(user2, Greeting).count()
        self.assertEqual(1, await q)

    async def test_user_authorization(self):
        owner = User(username="test_owner", roles=["greeter"])
        greeting = Greeting(message="hello")
        await Enforcer.save(greeting, owner, [Permission.Read, Permission.Write])

        user = User(username="test_user", roles=["greeter"])
        authorized = await Enforcer.is_authorized(greeting, user, Permission.Write)
        self.assertTrue(authorized)

        user2 = User(username="test_user2", roles=[])
        authorized = await Enforcer.is_authorized(greeting, user2, Permission.Write)
        self.assertFalse(authorized)

    async def test_owner_has_full_access(self):
        owner = User(username="test_owner", roles=["greeter"])
        greeting = Greeting(message="hello")
        await Enforcer.save(greeting, owner, [Permission.Read])
        authorized = await Enforcer.is_authorized(greeting, owner, Permission.Write)
        self.assertTrue(authorized)
