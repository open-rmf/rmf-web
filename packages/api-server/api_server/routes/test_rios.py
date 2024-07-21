import pydantic

from api_server.models import Rio
from api_server.models.tortoise_models import Rio as DbRio
from api_server.rmf_io import get_rio_events
from api_server.test import AppFixture


@AppFixture.reset_app_before_test
class TestRiosRoute(AppFixture):
    def test_get_rios(self):
        self.portal.call(
            DbRio(id="test_rio", type="test_type", data={"battery": 1}).save
        )
        self.portal.call(
            DbRio(id="test_rio2", type="test_type", data={"battery": 0.5}).save
        )
        self.portal.call(
            DbRio(id="test_rio3", type="test_type3", data={"battery": 0}).save
        )

        test_cases = [
            ("id=test_rio,test_rio2", 2),
            ("id=test_rio,test_rio4", 1),
            ("type=test_type,test_type3", 3),
            ("type=test_type,test_rio", 2),
            ("id=test_rio,test_rio3&type=test_type3", 1),
        ]

        for tc in test_cases:
            resp = self.client.get(f"/rios?{tc[0]}")
            self.assertEqual(200, resp.status_code, tc)
            rios = pydantic.TypeAdapter(list[Rio]).validate_json(resp.content)
            self.assertEqual(tc[1], len(rios))

    def test_sub_rios(self):
        with self.subscribe_sio("/rios") as sub:
            get_rio_events().rios.on_next(
                Rio(id="test_rio", type="test_type", data={"battery": 1})
            )
            rio = Rio(**next(sub))
            self.assertEqual("test_rio", rio.id)

    def test_put_rios(self):
        resp = self.client.put(
            "/rios",
            content=Rio(
                id="test_rio", type="test_type", data={"battery": 1}
            ).model_dump_json(),
        )
        self.assertEqual(201, resp.status_code)

        rios = self.portal.call(DbRio.all)
        self.assertEqual(1, len(rios))

        resp = self.client.put(
            "/rios",
            content=Rio(
                id="test_rio", type="test_type", data={"battery": 0.5}
            ).model_dump_json(),
        )
        # should return 200 if an existing resource is updated
        self.assertEqual(200, resp.status_code)
        rios = self.portal.call(DbRio.all)
        self.assertEqual(1, len(rios))
        if not isinstance(rios[0].data, dict):
            self.fail("data should be a dict")
        self.assertEqual(0.5, rios[0].data["battery"])
