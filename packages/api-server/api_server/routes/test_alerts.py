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
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # repeated creation with same ID will fail
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(409, resp.status_code, resp.content)

    def respond_to_alert(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # respond to alert that does not exist
        params = {"response": "resume"}
        resp = self.client.post(
            f"/alerts/request/wrong_alert/respond?{urlencode(params)}"
        )
        self.assertEqual(422, resp.status_code, resp.content)

        # response that is unavailable
        params = {"response": "wrong"}
        resp = self.client.post(f"/alerts/{alert_id}/respond?{urlencode(params)}")
        self.assertEqual(422, resp.status_code, resp.content)

        # respond correctly
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(id, resp.json()["id"], resp.content)
        self.assertEqual(response, resp.json()["response"], resp.content)

    def test_get_alert(self):
        alert_id = str(uuid4())

        # alert does not exist
        resp = self.client.get(f"/alerts/request/{alert_id}")
        self.assertEqual(404, resp.status_code, resp.content)

        # create alert
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # alert exists now
        resp = self.client.get(f"/alerts/request/{alert_id}")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

    def test_get_alert_response(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # respond
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert_id, resp.json()["id"], resp.content)
        self.assertEqual(response, resp.json()["response"], resp.content)

        # response exists
        resp = self.client.get(f"/alerts/request/{alert_id}/response")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(alert_id, resp.json()["id"], resp.content)
        self.assertEqual(response, resp.json()["response"], resp.content)

    def test_sub_alert(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])

        # check subscribed alert
        with self.subscribe_sio("/alerts/requests") as sub:
            resp = self.client.post(
                "/alerts/request", data=alert.json(exclude_none=True)
            )
            self.assertEqual(201, resp.status_code, resp.content)
            self.assertEqual(alert, resp.json(), resp.content)

            subbed_alert = mdl.AlertRequest(**next(sub))
            self.assertEqual(subbed_alert, alert, subbed_alert)

    def test_sub_alert_response(self):
        gen = self.subscribe_sio("/alerts/responses")

        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # respond
        response = "resume"
        params = {"response": response}
        with self.subscribe_sio("/alerts/responses") as sub:
            resp = self.client.post(
                f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
            )
            self.assertEqual(201, resp.status_code, resp.content)
            self.assertEqual(alert_id, resp.json()["id"], resp.content)
            self.assertEqual(response, resp.json()["response"], resp.content)

            # check subscribed alert response
            subbed_alert_response = mdl.AlertResponse(**next(sub))
            self.assertEqual(subbed_alert_response, resp.json(), subbed_alert_response)

    def test_get_alerts_of_task(self):
        alert_id = str(uuid4())
        alert = make_alert_request(alert_id=alert_id, responses=["resume", "cancel"])
        alert.task_id = "test_task_id"
        resp = self.client.post("/alerts/request", data=alert.json(exclude_none=True))
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert, resp.json(), resp.content)

        # check for non-existent alert for a wrong task ID
        resp = self.client.get("/alerts/requests/task/wrong_task_id")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(0, len(resp.json()), resp.content)

        # check for correct task ID
        resp = self.client.get(f"/alerts/requests/task/{alert.task_id}")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(1, len(resp.json()), resp.content)
        self.assertEqual(resp.json()[0], alert, resp.content)

        # respond to alert
        response = "resume"
        params = {"response": response}
        resp = self.client.post(
            f"/alerts/request/{alert_id}/respond?{urlencode(params)}"
        )
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(alert_id, resp.json()["id"], resp.content)
        self.assertEqual(response, resp.json()["response"], resp.content)

        # check for alert of correct task ID again (will only return
        # unresponded by default)
        resp = self.client.get(f"/alerts/requests/task/{alert.task_id}")
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(0, len(resp.json()), resp.content)

        # check for alert of correct task ID again with unresponded False
        params = {"unresponded": False}
        resp = self.client.get(
            f"/alerts/requests/task/{alert.task_id}?{urlencode(params)}"
        )
        self.assertEqual(200, resp.status_code, resp.content)
        self.assertEqual(1, len(resp.json()), resp.content)
        self.assertEqual(resp.json()[0], alert, resp.content)

    def test_get_unresponded_alert_ids(self):
        first_id = str(uuid4())
        first_alert = make_alert_request(
            alert_id=first_id, responses=["resume", "cancel"]
        )
        resp = self.client.post(
            "/alerts/request", data=first_alert.json(exclude_none=True)
        )
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(first_alert, resp.json(), resp.content)

        second_id = str(uuid4())
        second_alert = make_alert_request(
            alert_id=second_id, responses=["resume", "cancel"]
        )
        resp = self.client.post(
            "/alerts/request", data=second_alert.json(exclude_none=True)
        )
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(second_alert, resp.json(), resp.content)

        # both alerts unresponded
        resp = self.client.get("/alerts/unresponded_requests")
        self.assertEqual(200, resp.status_code, resp.content)
        unresponded_num = len(resp.json())
        self.assertTrue(unresponded_num > 0, resp.content)
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
        self.assertEqual(201, resp.status_code, resp.content)
        self.assertEqual(first_id, resp.json()["id"], resp.content)
        self.assertEqual(response, resp.json()["response"], resp.content)

        # first is no longer returned
        resp = self.client.get("/alerts/unresponded_requests")
        self.assertEqual(200, resp.status_code, resp.content)
        new_unresponded_num = len(resp.json())
        self.assertTrue(new_unresponded_num > 0, resp.content)
        self.assertTrue(unresponded_num - new_unresponded_num == 1, resp.content)
        returned_alerts = resp.json()
        returned_alert_ids = [a["id"] for a in returned_alerts]
        self.assertTrue(first_id not in returned_alert_ids)
        self.assertTrue(second_id in returned_alert_ids)
