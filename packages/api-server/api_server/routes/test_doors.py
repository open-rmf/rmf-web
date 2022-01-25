from uuid import uuid4

from rmf_door_msgs.msg import DoorMode as RmfDoorMode

from api_server.test import AppFixture, make_building_map, make_door_state
from api_server.test.test_fixtures import try_until


class TestDoorsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.building_map = make_building_map()
        cls.door_states = [make_door_state(f"test_{uuid4()}")]

        async def prepare_db():
            await cls.building_map.save()
            for x in cls.door_states:
                await x.save()

        cls.run_in_app_loop(prepare_db())

    def test_get_doors(self):
        resp = self.session.get("/doors")
        self.assertEqual(200, resp.status_code)
        doors = resp.json()
        self.assertEqual(1, len(doors))
        self.assertEqual("test_door", doors[0]["name"])

    def test_get_door_state(self):
        resp = self.session.get(f"/doors/{self.door_states[0].door_name}/state")
        self.assertEqual(200, resp.status_code)
        state = resp.json()
        self.assertEqual(self.door_states[0].door_name, state["door_name"])

    def test_sub_door_state(self):
        fut = self.subscribe_sio(f"/doors/{self.door_states[0].door_name}/state")
        try_until(fut.done, lambda x: x)
        result = fut.result(0)
        self.assertEqual(self.door_states[0].door_name, result["door_name"])

    def test_post_door_request(self):
        resp = self.session.post(
            "/doors/test_door/request", json={"mode": RmfDoorMode.MODE_OPEN}
        )
        self.assertEqual(resp.status_code, 200)
