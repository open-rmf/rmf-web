from uuid import uuid4

from ..test.test_fixtures import RouteFixture


class TestAdminRoute(RouteFixture):
    def create_user(self):
        username = f"user_{uuid4().hex}"
        resp = self.session.post(
            f"{self.base_url}/admin/users", json={"username": username}
        )
        self.assertEqual(200, resp.status_code)
        return username

    def create_role(self):
        role_name = f"role_{uuid4().hex}"
        resp = self.session.post(
            f"{self.base_url}/admin/roles", json={"name": role_name}
        )
        self.assertEqual(200, resp.status_code)
        return role_name

    def test_crud_user(self):
        username = self.create_user()

        resp = self.session.get(f"{self.base_url}/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(username, user["username"])

        resp = self.session.delete(f"{self.base_url}/admin/users/{username}")
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(f"{self.base_url}/admin/users/{username}")
        self.assertEqual(404, resp.status_code)

    def test_crud_roles(self):
        role = self.create_role()

        resp = self.session.get(f"{self.base_url}/admin/roles")
        self.assertEqual(200, resp.status_code)
        roles = resp.json()
        self.assertIn(role, roles)

        resp = self.session.delete(f"{self.base_url}/admin/roles/{role}")
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(f"{self.base_url}/admin/roles")
        self.assertEqual(200, resp.status_code)
        self.assertNotIn(role, resp.json())

    def test_crud_role_permissions(self):
        role = self.create_role()

        resp = self.session.post(
            f"{self.base_url}/admin/roles/{role}/permissions",
            json={"action": "test_action", "authz_grp": "test_group"},
        )
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(
            f"{self.base_url}/admin/roles/{role}/permissions",
        )
        self.assertEqual(200, resp.status_code)
        permissions = resp.json()
        self.assertEqual(1, len(permissions))
        self.assertEqual("test_group", permissions[0]["authz_grp"])
        self.assertEqual("test_action", permissions[0]["action"])

        resp = self.session.delete(
            f"{self.base_url}/admin/roles/{role}/permissions",
            json={"action": "test_action", "authz_grp": "test_group"},
        )
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(
            f"{self.base_url}/admin/roles/{role}/permissions",
        )
        self.assertEqual(200, resp.status_code)
        permissions = resp.json()
        self.assertEqual(0, len(permissions))

    def test_crud_user_roles(self):
        username = self.create_user()
        role = self.create_role()

        resp = self.session.post(
            f"{self.base_url}/admin/users/{username}/roles", json={"name": role}
        )
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(f"{self.base_url}/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(1, len(user["roles"]))
        self.assertEqual(role, user["roles"][0])

        resp = self.session.delete(
            f"{self.base_url}/admin/users/{username}/roles", json={"name": role}
        )
        self.assertEqual(200, resp.status_code)

        resp = self.session.get(f"{self.base_url}/admin/users/{username}")
        self.assertEqual(200, resp.status_code)
        user = resp.json()
        self.assertEqual(0, len(user["roles"]))
