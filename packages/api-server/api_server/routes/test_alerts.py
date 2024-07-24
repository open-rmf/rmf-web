from urllib.parse import urlencode
from uuid import uuid4

from api_server import models as mdl
from api_server.test import AppFixture, make_alert_request


class TestAlertsRoute(AppFixture):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()

    def test_create_new_alert(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # repeated creation with same ID will fail
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(409, resp.status_code, resp.json())

    def respond_to_alert(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # respond to alert that does not exist
        params = {"response": "resume"}
        resp = self.client.post(
            f"/alerts/request/wrong_alert/respond?{urlencode(params)}"
        )
        self.assertEqual(422, resp.status_code, resp.json())

        # response that is unavailable
        params = {"response": "wrong"}
        resp = self.client.post(f"/alerts/{alert_id}/respond?{urlencode(params)}")
        self.assertEqual(422, resp.status_code, resp.json())

        # respond correctly
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(id, resp.json()["id"], resp.json())
        self.assertEqual(response, resp.json()["response"], resp.json())

    def test_get_alert(self):
        alert_id = str(uuid4())

        # alert does not exist
        resp = self.client.get(f"/alerts/request/{alert_id}")
        self.assertEqual(404, resp.status_code, resp.json())

        # create alert
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # alert exists now
        resp = self.client.get(f"/alerts/request/{alert_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

    def test_get_alert_response(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # respond
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert_id, resp.json()["id"], resp.json())
        self.assertEqual(response, resp.json()["response"], resp.json())

        # response exists
        resp = self.client.get(f"/alerts/request/{alert_id}/response")
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(alert_id, resp.json()["id"], resp.json())
        self.assertEqual(response, resp.json()["response"], resp.json())

    def test_sub_alert(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])

        # check subscribed alert
        with self.subscribe_sio("/alerts/requests") as sub:
            resp = self.client.post("/alerts/request", content=alert.model_dump_json())
            self.assertEqual(201, resp.status_code, resp.json())
            self.assertEqual(alert.model_dump(), resp.json(), resp.json())

            subbed_alert = mdl.AlertRequest(**next(sub))
            self.assertEqual(subbed_alert, alert, subbed_alert)

    def test_sub_alert_response(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # respond
        response = "resume"
        params = {"response": response}
        with self.subscribe_sio("/alerts/responses") as sub:
            resp = self.client.post(
                f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
            )
            self.assertEqual(201, resp.status_code, resp.json())
            self.assertEqual(alert_id, resp.json()["id"], resp.json())
            self.assertEqual(response, resp.json()["response"], resp.json())

            # check subscribed alert response
            subbed_alert_response = mdl.AlertResponse(**next(sub))
            self.assertEqual(
                subbed_alert_response.model_dump(), resp.json(), subbed_alert_response
            )

    def test_get_alerts_of_task(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        alert.task_id = "test_task_id"
        resp = self.client.post("/alerts/request", content=alert.model_dump_json())
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert.model_dump(), resp.json(), resp.json())

        # check for non-existent alert for a wrong task ID
        resp = self.client.get("/alerts/requests/task/wrong_task_id")
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(0, len(resp.json()), resp.json())

        # check for correct task ID
        resp = self.client.get(f"/alerts/requests/task/{alert.task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(1, len(resp.json()), resp.json())
        self.assertEqual(resp.json()[0], alert.model_dump(), resp.json())

        # respond to alert
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(alert_id, resp.json()["id"], resp.json())
        self.assertEqual(response, resp.json()["response"], resp.json())

        # check for alert of correct task ID again (will only return
        # unresponded by default)
        resp = self.client.get(f"/alerts/requests/task/{alert.task_id}")
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(0, len(resp.json()), resp.json())

        # check for alert of correct task ID again with unresponded False
        params = {"unresponded": False}
        resp = self.client.get(
            f"/alerts/requests/task/{alert.task_id}?{urlencode(params)}"
        )
        self.assertEqual(200, resp.status_code, resp.json())
        self.assertEqual(1, len(resp.json()), resp.json())
        self.assertEqual(resp.json()[0], alert.model_dump(), resp.json())

    def test_get_unresponded_alert_ids(self):
        first_id = str(uuid4())
        first_alert = make_alert_request(
            alert_id=first_id, responses=["resume", "cancel"]
        )
        resp = self.client.post(
            "/alerts/request", content=first_alert.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(first_alert.model_dump(), resp.json(), resp.json())

        second_id = str(uuid4())
        second_alert = make_alert_request(
            alert_id=second_id, responses=["resume", "cancel"]
        )
        resp = self.client.post(
            "/alerts/request", content=second_alert.model_dump_json()
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(second_alert.model_dump(), resp.json(), resp.json())

        # both alerts unresponded
        resp = self.client.get("/alerts/unresponded_requests")
        self.assertEqual(200, resp.status_code, resp.json())
        unresponded_num = len(resp.json())
        self.assertTrue(unresponded_num > 0, resp.json())
        returned_alerts = resp.json()
        returned_alert_ids = [a["id"] for a in returned_alerts]
        self.assertTrue(first_id in returned_alert_ids)
        self.assertTrue(second_id in returned_alert_ids)

        # respond to first
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{first_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.json())
        self.assertEqual(first_id, resp.json()["id"], resp.json())
        self.assertEqual(response, resp.json()["response"], resp.json())

        # first is no longer returned
        resp = self.client.get("/alerts/unresponded_requests")
        self.assertEqual(200, resp.status_code, resp.json())
        new_unresponded_num = len(resp.json())
        self.assertTrue(new_unresponded_num > 0, resp.json())
        self.assertTrue(unresponded_num - new_unresponded_num == 1, resp.json())
        returned_alerts = resp.json()
        returned_alert_ids = [a["id"] for a in returned_alerts]
        self.assertTrue(first_id not in returned_alert_ids)
        self.assertTrue(second_id in returned_alert_ids)
