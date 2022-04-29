from api_server.test import AppFixture


class TestMainRoute(AppFixture):
    def test_effective_permissions(self):
        user = self.create_user()
        role1 = self.create_role()
        self.add_permission(role1, "test_action", "test_group")
        role2 = self.create_role()
        self.add_permission(role2, "test_action_2", "test_group")
        role3 = self.create_role()
        self.add_permission(role3, "test_action_2", "test_group")
        self.assign_role(user, role1)
        self.assign_role(user, role2)
        self.assign_role(user, role3)

        self.client.set_user(user)
        resp = self.client.get("/permissions")
        self.assertEqual(200, resp.status_code)
        perms = resp.json()
        self.assertEqual(2, len(perms))

        compiled = [f"{p['authz_grp']}__{p['action']}" for p in perms]
        self.assertIn("test_group__test_action", compiled)
        self.assertIn("test_group__test_action_2", compiled)
