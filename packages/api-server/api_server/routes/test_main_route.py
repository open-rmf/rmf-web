from api_server.test.test_fixtures import AppFixture


class TestMainRoute(AppFixture):
    async def test_effective_permissions(self):
        user = await self.create_user()
        role1 = await self.create_role()
        await self.add_permission(role1, "test_action", "test_group")
        role2 = await self.create_role()
        await self.add_permission(role2, "test_action_2", "test_group")
        role3 = await self.create_role()
        await self.add_permission(role3, "test_action_2", "test_group")
        await self.assign_role(user, role1)
        await self.assign_role(user, role2)
        await self.assign_role(user, role3)

        self.set_user(user)
        resp = await self.client.get("/permissions")
        self.assertEqual(200, resp.status_code)
        perms = resp.json()
        self.assertEqual(2, len(perms))

        compiled = [f"{p['authz_grp']}__{p['action']}" for p in perms]
        self.assertIn("test_group__test_action", compiled)
        self.assertIn("test_group__test_action_2", compiled)
