import asyncio
from typing import cast
from uuid import uuid4

from rmf_door_msgs.msg import DoorMode as RmfDoorMode

from api_server.models import DoorState
from api_server.test import AppFixture, make_building_map, make_door_state


class TestDoorsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.building_map = make_building_map()
        asyncio.run(cls.building_map.save())

        cls.door_states = [make_door_state(f"test_{uuid4()}")]

        for x in cls.door_states:
            asyncio.run(x.save())

    def test_get_doors(self):
        resp = self.client.get("/doors")
        self.assertEqual(200, resp.status_code)
        doors = resp.json()
        self.assertEqual(1, len(doors))
        self.assertEqual("test_door", doors[0]["name"])

    def test_get_door_state(self):
        resp = self.client.get(f"/doors/{self.door_states[0].door_name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.door_states[0].door_name, state["door_name"])

    def test_sub_door_state(self):
        msg = next(self.subscribe_sio(f"/doors/{self.door_states[0].door_name}/state"))
        self.assertEqual(self.door_states[0].door_name, cast(DoorState, msg).door_name)

    def test_post_door_request(self):
        resp = self.client.post(
            "/doors/test_door/request", json={"mode": RmfDoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
