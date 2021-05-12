import unittest

from models.dispenser_state import DispenserState

from .auth_event_parser import auth_event_parser


class TestCaseAuthEvent(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.data = '[0m[0m20:41:54,721 INFO  [org.keycloak.events] (default task-2) JSON_EVENT::{"type":"LOGIN_ERROR","realmId":"579ce396-83c7-4094-964d-7ea07553089f","clientId":"reporting","ipAddress":"192.168.49.1","error":"user_not_found","auth_method":"openid-connect","auth_type":"code","redirect_uri":"https://example.com/reporting","code_id":"f813403c-2732-4062-9911-cf65b89a2278","username":"test"}'

    async def test_parse_and_get_values(self):
        parsed_values = await auth_event_parser(self.data)
        self.assertEqual(parsed_values["username"], "test")
        self.assertEqual(parsed_values["user_keycloak_id"], None)
        self.assertEqual(parsed_values["event_type"], "LOGIN_ERROR")
        self.assertEqual(
            parsed_values["realm_id"], "579ce396-83c7-4094-964d-7ea07553089f"
        )
        self.assertEqual(parsed_values["client_id"], "reporting")
        self.assertEqual(parsed_values["ip_address"], "192.168.49.1")
