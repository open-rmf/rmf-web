import pydantic

from api_server.models import Mission
from api_server.routes.tasks.missions import CreateMission
from api_server.test.test_fixtures import AppFixture


@AppFixture.reset_app_before_test
class TestMissions(AppFixture):
    def test_crud(self):
        create_mission = CreateMission(name="test", ui_schema="{}", task_template="")
        resp = self.client.post(
            "/missions/create_mission",
            content=create_mission.model_dump_json(),
        )
        mission = Mission.model_validate_json(resp.content)
        self.assertEqual(201, resp.status_code)

        # test cannot missions with the same name
        resp = self.client.post(
            "/missions/create_mission", content=create_mission.model_dump_json()
        )
        self.assertEqual(409, resp.status_code)

        # test getting all missions
        resp = self.client.get("/missions")
        self.assertEqual(200, resp.status_code)
        all_missions = pydantic.TypeAdapter(list[Mission]).validate_json(resp.content)
        self.assertEqual(1, len(all_missions))
        self.assertEqual(mission.model_dump_json(), all_missions[0].model_dump_json())

        # test query mission by name
        resp = self.client.get("/missions?name=test")
        self.assertEqual(200, resp.status_code)
        all_missions = pydantic.TypeAdapter(list[Mission]).validate_json(resp.content)
        self.assertEqual(1, len(all_missions))
        self.assertEqual(mission.model_dump_json(), all_missions[0].model_dump_json())

        # test querying non existing name
        resp = self.client.get("/missions?name=nonexisting")
        self.assertEqual(200, resp.status_code)
        all_missions = pydantic.TypeAdapter(list[Mission]).validate_json(resp.content)
        self.assertEqual(0, len(all_missions))

        # test getting one mission
        resp = self.client.get(f"/missions/{mission.id}")
        self.assertEqual(200, resp.status_code)
        rt_mission = Mission.model_validate_json(resp.content)
        self.assertEqual(mission.model_dump_json(), rt_mission.model_dump_json())

        # test getting non existing mission
        resp = self.client.get(f"/missions/{mission.id + 1}")
        self.assertEqual(404, resp.status_code)

        # test update mission
        update_mission = create_mission
        update_mission.name = "updated"
        resp = self.client.put(
            f"/missions/{mission.id}", content=update_mission.model_dump_json()
        )
        self.assertEqual(200, resp.status_code)
        resp = self.client.get(f"/missions/{mission.id}")
        self.assertEqual(200, resp.status_code)
        updated = Mission.model_validate_json(resp.content)
        self.assertEqual(update_mission.name, updated.name)

        # test update non existing mission
        resp = self.client.put(
            f"/missions/{mission.id + 1}", content=update_mission.model_dump_json()
        )
        self.assertEqual(404, resp.status_code)

        # test delete mission
        resp = self.client.delete(f"/missions/{mission.id}")
        self.assertEqual(200, resp.status_code)

        # test delete non existing missing
        resp = self.client.delete(f"/missions/{mission.id}")
        self.assertEqual(404, resp.status_code)
