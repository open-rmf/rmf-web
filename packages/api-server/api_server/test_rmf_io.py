# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order

import asyncio
import os
import unittest
import urllib.request
from typing import Callable, Optional

import jwt
import rclpy
import socketio
from api_server.rmf_io.topics import topics
from rmf_building_map_msgs.msg import BuildingMap
from rmf_dispenser_msgs.msg import DispenserState
from rmf_door_msgs.msg import DoorState
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorState
from rmf_lift_msgs.msg import LiftState
from rmf_task_msgs.msg import TaskSummary

from .test.ros_fixture import RosFixture
from .test.server import launch_server, terminate_server
from .test.test_data import (
    make_building_map,
    make_dispenser_state,
    make_door_state,
    make_fleet_state,
    make_ingestor_state,
    make_lift_state,
    make_task_summary,
)


class TestRmfIO(RosFixture):
    @classmethod
    def setUpClass(cls):
        cls.server_proc = launch_server()
        super().setUpClass()

    @classmethod
    def tearDownClass(cls):
        super().tearDownClass()
        terminate_server(cls.server_proc)

    async def make_client(self):
        client = socketio.AsyncClient()
        fut = asyncio.Future()
        client.on("connect", lambda: fut.set_result(None))
        retry_count = 0
        while True:
            try:
                await client.connect("http://localhost:8000")
                break
            except socketio.exceptions.ConnectionError:
                if retry_count < 3:
                    retry_count += 1
                    await asyncio.sleep(1)
                else:
                    raise
        await asyncio.wait_for(fut, 5)
        return client

    async def client_subscribe(self, client: socketio.AsyncClient, topic: str):
        await client.emit("subscribe", topic)
        fut = asyncio.Future()
        client.on("subscribe", fut.set_result)
        return await fut

    async def check_endpoint(self, topic: str, do_ros_publish: Callable[[], None]):
        """
        Checks that an endpoint can receive events and that newly connected clients receive the
        current state of all subjects
        """
        fut = asyncio.Future()
        self.client.on(topic, fut.set_result)
        await self.client_subscribe(self.client, topic)
        do_ros_publish()
        await asyncio.wait_for(fut, 1)

        new_client = await self.make_client()
        fut = asyncio.Future()
        new_client.on(topic, fut.set_result)
        await self.client_subscribe(new_client, topic)
        result = await asyncio.wait_for(fut, 1)
        await new_client.disconnect()

        return result

    async def subscribe_one(self, topic: str):
        fut = asyncio.Future()
        self.client.on(topic, fut.set_result)
        await self.client_subscribe(self.client, topic)
        return await asyncio.wait_for(fut, 1)

    async def asyncSetUp(self):
        self.client = await self.make_client()

    async def asyncTearDown(self):
        await self.client.disconnect()

    async def test_door_states(self):
        def ros_pub():
            pub = self.node.create_publisher(DoorState, "door_states", 10)
            pub.publish(make_door_state("test_door"))

        result = await self.check_endpoint(topics.door_states, ros_pub)
        self.assertEqual(result["door_name"], "test_door")

        health = await self.subscribe_one(topics.door_health)
        self.assertEqual(health["id"], "test_door")

    async def test_lift_states(self):
        def ros_pub():
            pub = self.node.create_publisher(LiftState, "lift_states", 10)
            pub.publish(make_lift_state("test_lift"))

        result = await self.check_endpoint(topics.lift_states, ros_pub)
        self.assertEqual(result["lift_name"], "test_lift")

        health = await self.subscribe_one(topics.lift_health)
        self.assertEqual(health["id"], "test_lift")

    async def test_dispenser_states(self):
        def ros_pub():
            pub = self.node.create_publisher(DispenserState, "dispenser_states", 10)
            pub.publish(make_dispenser_state("test_dispenser"))

        result = await self.check_endpoint(topics.dispenser_states, ros_pub)
        self.assertEqual(result["guid"], "test_dispenser")

        health = await self.subscribe_one(topics.dispenser_health)
        self.assertEqual(health["id"], "test_dispenser")

    async def test_ingestor_states(self):
        def ros_pub():
            pub = self.node.create_publisher(IngestorState, "ingestor_states", 10)
            pub.publish(make_ingestor_state("test_ingestor"))

        result = await self.check_endpoint(topics.ingestor_states, ros_pub)
        self.assertEqual(result["guid"], "test_ingestor")

        health = await self.subscribe_one(topics.ingestor_health)
        self.assertEqual(health["id"], "test_ingestor")

    async def test_fleet_states(self):
        def ros_pub():
            pub = self.node.create_publisher(FleetState, "fleet_states", 10)
            pub.publish(make_fleet_state("test_fleet"))

        result = await self.check_endpoint(topics.fleet_states, ros_pub)
        self.assertEqual(result["name"], "test_fleet")

        health = await self.subscribe_one(topics.robot_health)
        self.assertEqual(health["id"], "test_fleet/test_robot")

    async def test_task_summary(self):
        def ros_pub():
            pub = self.node.create_publisher(TaskSummary, "task_summaries", 10)
            pub.publish(make_task_summary())

        result = await self.check_endpoint(topics.task_summaries, ros_pub)
        self.assertEqual(result["task_id"], "test_task")

    async def test_building_map(self):
        """
        test building map image is changed to an url and that it is accessible
        """

        def ros_pub():
            pub = self.node.create_publisher(
                BuildingMap,
                "map",
                rclpy.qos.QoSProfile(
                    history=rclpy.qos.HistoryPolicy.KEEP_ALL,
                    depth=1,
                    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            pub.publish(make_building_map())

        result = await self.check_endpoint(topics.building_map, ros_pub)
        self.assertEqual(result["name"], "test_name")
        image_data = result["levels"][0]["images"][0]["data"]
        self.assertEqual(type(image_data), str)

        resp = urllib.request.urlopen(f"http://localhost:8000{image_data}")
        self.assertEqual(resp.status, 200)


class TestRmfIO_JWTAuth(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        cls.server_proc = launch_server(
            f"{os.path.dirname(__file__)}/test/config_auth.py"
        )
        super().setUpClass()

    @classmethod
    def tearDownClass(cls):
        super().tearDownClass()
        if cls.server_proc:
            terminate_server(cls.server_proc)

    async def asyncSetUp(self):
        self.client = socketio.AsyncClient()

    async def asyncTearDown(self):
        await self.client.disconnect()

    async def try_connect(self, token: Optional[str] = None) -> bool:
        args = [
            "node",
            f"{os.path.dirname(__file__)}/test/connect.js",
            "http://localhost:8000",
        ]
        if token:
            args.append(token)

        proc = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.DEVNULL,
        )
        await proc.wait()

        if proc.returncode == 0:
            return True
        return False

    async def test_fail_with_no_token(self):
        self.assertFalse(await self.try_connect())

    async def test_fail_with_invalid_token(self):
        self.assertFalse(await self.try_connect("invalid"))

    async def test_success_with_valid_token(self):
        with open(f"{os.path.dirname(__file__)}/test/test.key", "br") as f:
            private_key = f.read()
        token = jwt.encode(
            {"some": "payload", "aud": "rmf-server"}, private_key, algorithm="RS256"
        )

        success = await self.try_connect(token)

        self.assertTrue(success)
