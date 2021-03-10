import asyncio
import unittest
from typing import Any, Callable
from unittest.mock import MagicMock

import aiohttp.web
import socketio
from rx import Observable

from ..models import DispenserHealth, DoorHealth, HealthStatus, LiftHealth, RobotHealth
from ..repositories.static_files import StaticFilesRepository
from .gateway import RmfGateway
from .rmf_io import RmfIO
from .test_data import (
    make_building_map,
    make_dispenser_state,
    make_door_state,
    make_fleet_state,
    make_lift_state,
)
from .topics import topics


class TestRmfIO(unittest.IsolatedAsyncioTestCase):
    async def make_client(self):
        client = socketio.AsyncClient()
        self.clients.append(client)
        await client.connect(f"http://localhost:{self.server_port}")
        fut = asyncio.Future()
        client.on("connect", lambda: fut.set_result(None))
        await fut
        return client

    async def client_subscribe(self, client: socketio.AsyncClient, topic: str):
        await client.emit("subscribe", topic)
        fut = asyncio.Future()
        client.on("subscribe", fut.set_result)
        return await fut

    async def asyncSetUp(self):
        self.clients = []
        self.rmf_gateway = RmfGateway()
        self.sio = socketio.AsyncServer(async_mode="aiohttp")
        self.static_files = MagicMock(StaticFilesRepository)
        self.static_files.add_file.return_value = "/test_url"
        self.rmf_io = RmfIO(self.sio, self.rmf_gateway, self.static_files)
        self.app = aiohttp.web.Application()
        self.sio.attach(self.app)
        self.runner = aiohttp.web.AppRunner(self.app)
        await self.runner.setup()
        self.site = aiohttp.web.TCPSite(self.runner, "127.0.0.1", 0)
        await self.site.start()
        self.server_port = self.runner.addresses[0][1]

    async def asyncTearDown(self):
        asyncio.gather(*[client.disconnect() for client in self.clients])
        self.clients.clear()
        await self.runner.cleanup()

    async def check_endpoint(
        self,
        topic: str,
        factory: Callable[[str], Any],
        id_mapper: Callable[[Any], str],
        source: Observable,
    ):
        """
        Checks that an endpoint can receive events and that newly connected clients receive the
        current state of all subjects
        """
        done = asyncio.Future()
        test_ids = ["test_id", "test_id_2"]
        test_values = [factory(x) for x in test_ids]
        received = {}
        client = await self.make_client()
        await self.client_subscribe(client, topic)

        def on_receive(data):
            received[id_mapper(data)] = data
            if len(received) == len(test_ids) and all(
                (x in test_ids for x in received)
            ):
                done.set_result(True)

        client.on(topic, on_receive)

        for x in test_values:
            source.on_next(x)
        await asyncio.wait_for(done, 1)
        received = {}

        # test that a new client receives all the current states on subscribe
        done = asyncio.Future()
        client = await self.make_client()
        client.on(topic, on_receive)
        await self.client_subscribe(client, topic)
        await asyncio.wait_for(done, 1)

    async def test_door_states(self):
        await self.check_endpoint(
            topics.door_states,
            make_door_state,
            lambda x: x["door_name"],
            self.rmf_gateway.door_states,
        )

    async def test_door_health(self):
        def factory(id_: str):
            return DoorHealth(id_=id_, health_status=HealthStatus.HEALTHY)

        await self.check_endpoint(
            topics.door_health,
            factory,
            lambda x: x["id"],
            self.rmf_gateway.door_health,
        )

    async def test_lift_states(self):
        def factory(id_: str):
            state = make_lift_state()
            state.lift_name = id_
            return state

        await self.check_endpoint(
            topics.lift_states,
            factory,
            lambda x: x["lift_name"],
            self.rmf_gateway.lift_states,
        )

    async def test_lift_health(self):
        def factory(id_: str):
            return LiftHealth(id_=id_, health_status=HealthStatus.HEALTHY)

        await self.check_endpoint(
            topics.lift_health,
            factory,
            lambda x: x["id"],
            self.rmf_gateway.lift_health,
        )

    async def test_dispenser_states(self):
        def factory(id_: str):
            state = make_dispenser_state()
            state.guid = id_
            return state

        await self.check_endpoint(
            topics.dispenser_states,
            factory,
            lambda x: x["guid"],
            self.rmf_gateway.dispenser_states,
        )

    async def test_dispenser_health(self):
        def factory(id_: str):
            return DispenserHealth(id_=id_, health_status=HealthStatus.HEALTHY)

        await self.check_endpoint(
            topics.dispenser_health,
            factory,
            lambda x: x["id"],
            self.rmf_gateway.dispenser_health,
        )

    async def test_fleet_states(self):
        def factory(id_: str):
            state = make_fleet_state()
            state.name = id_
            return state

        await self.check_endpoint(
            topics.fleet_states,
            factory,
            lambda x: x["name"],
            self.rmf_gateway.fleet_states,
        )

    async def test_robot_health(self):
        def factory(id_: str):
            return RobotHealth(id_=id_, health_status=HealthStatus.HEALTHY)

        await self.check_endpoint(
            topics.robot_health,
            factory,
            lambda x: x["id"],
            self.rmf_gateway.robot_health,
        )

    async def test_building_map(self):
        """
        test building map image is saved to static files and data is replaced
        with the url.
        """
        client = await self.make_client()
        await self.client_subscribe(client, topics.building_map)
        done = asyncio.Future()

        def on_building_map(building_map: dict):
            image = building_map["levels"][0]["images"][0]
            image_data = image["data"]
            self.assertEqual(image_data, "/test_url")
            done.set_result(True)

        client.on(topics.building_map, on_building_map)

        building_map = make_building_map()
        self.rmf_gateway.building_map.on_next(building_map)
        await asyncio.wait_for(done, 5)
        self.assertEqual(
            self.static_files.add_file.call_args[0][1],
            "test_name/L1-test_image.thbyxgrllndgeciymb3a47hf2re5p7no.png",
        )
