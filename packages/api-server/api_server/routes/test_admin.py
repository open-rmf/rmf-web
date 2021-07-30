from api_server.test.test_fixtures import AppFixture


class TestAdminRoute(AppFixture):
    async def test_query_users(self):
        # query usernames
        user = await self.create_user()
        resp = await self.client.get(f"/admin/users?username={user}")
        self.assertEqual(200, resp.status_code)
        users = resp.json()
        self.assertIn(user, users)

        # query admins
        resp = await self.client.get("/admin/users?is_admin=true")
        self.assertEqual(200, resp.status_code)
        users = resp.json()
        self.assertIn("admin", users)

    async def test_crud_user(self):
        username = await self.create_user()

        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(username, user["username"])

        resp = await self.client.post(
            f"/admin/users/{username}/make_admin", json={"admin": True}
        )
        self.assertEqual(200, resp.status_code)
        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(True, user["is_admin"])

        resp = await self.client.delete(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)

        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(404, resp.status_code)

    async def test_crud_roles(self):
        role = await self.create_role()

        resp = await self.client.get("/admin/roles")
        self.assertEqual(200, resp.status_code)
        roles = resp.json()
        self.assertIn(role, roles)

        resp = await self.client.delete(f"/admin/roles/{role}")
        self.assertEqual(200, resp.status_code)

        resp = await self.client.get("/admin/roles")
        self.assertEqual(200, resp.status_code)
        self.assertNotIn(role, resp.json())

    async def test_crud_role_permissions(self):
        role = await self.create_role()
        await self.add_permission(role, "test_action", "test_group")

        resp = await self.client.get(
            f"/admin/roles/{role}/permissions",
        )
        self.assertEqual(200, resp.status_code)
        permissions = resp.json()
        self.assertEqual(1, len(permissions))
        self.assertEqual("test_group", permissions[0]["authz_grp"])
        self.assertEqual("test_action", permissions[0]["action"])

        resp = await self.client.post(
            f"/admin/roles/{role}/permissions/remove",
            json={
                "authz_grp": permissions[0]["authz_grp"],
                "action": permissions[0]["action"],
            },
        )
        self.assertEqual(200, resp.status_code)

        resp = await self.client.get(
            f"/admin/roles/{role}/permissions",
        )
        self.assertEqual(200, resp.status_code)
        permissions = resp.json()
        self.assertEqual(0, len(permissions))

    async def test_crud_user_roles(self):
        username = await self.create_user()
        role = await self.create_role()
        await self.assign_role(username, role)

        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(1, len(user["roles"]))
        self.assertEqual(role, user["roles"][0])

        resp = await self.client.delete(f"/admin/users/{username}/roles/{role}")
        self.assertEqual(200, resp.status_code)

        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(0, len(user["roles"]))

    async def test_put_user_roles(self):
        username = await self.create_user()
        role = await self.create_role()
        role2 = await self.create_role()
        role3 = await self.create_role()
        await self.assign_role(username, role3)

        # error when one of the is invalid
        resp = await self.client.put(
            f"/admin/users/{username}/roles",
            json=[{"name": role}, {"name": "non_existing_role"}],
        )
        self.assertEqual(422, resp.status_code)

        resp = await self.client.put(
            f"/admin/users/{username}/roles",
            json=[{"name": role}, {"name": role2}],
        )
        self.assertEqual(200, resp.status_code)

        resp = await self.client.get(f"/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(2, len(user["roles"]))
        self.assertIn(role, user["roles"])
        self.assertIn(role2, user["roles"])
        self.assertNotIn(role3, user["roles"])
