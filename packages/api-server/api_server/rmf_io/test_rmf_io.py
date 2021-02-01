import unittest
import asyncio
import aiohttp.web
import socketio

from rosidl_runtime_py.convert import message_to_ordereddict
from builtin_interfaces.msg import Time
from rmf_door_msgs.msg import DoorState, DoorMode

from .rmf_io import RmfIO
from .topics import topics


def make_door_state(name: str):
    door_state = DoorState()
    door_state.door_name = name
    door_state.current_mode = DoorMode(value=DoorMode.MODE_CLOSED)
    door_state.door_time = Time(sec=0, nanosec=0)
    return message_to_ordereddict(door_state)


class TestRmfIO(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.sio = socketio.AsyncServer(async_mode='aiohttp')
        self.rmf_io = RmfIO(self.sio)
        self.app = aiohttp.web.Application()
        self.sio.attach(self.app)
        self.runner = aiohttp.web.AppRunner(self.app)
        await self.runner.setup()
        self.site = aiohttp.web.TCPSite(self.runner, 'localhost', 8080)
        await self.site.start()

        self.client = client = socketio.AsyncClient()
        await client.connect('http://localhost:8080')
        fut = asyncio.Future()
        self.client.on('connect', lambda: fut.set_result(None))
        await fut

    async def asyncTearDown(self):
        await self.client.disconnect()
        await self.runner.cleanup()

    async def test_door_states(self):
        '''
        test receiving door states events
        '''
        door_state_fut = asyncio.Future()
        self.client.on(topics.door_states, door_state_fut.set_result)

        await self.client.emit('subscribe', topics.door_states)
        sub_result = asyncio.Future()
        self.client.on('subscribe', sub_result.set_result)
        await sub_result

        self.rmf_io.on_door_state(make_door_state('test_door'))
        await door_state_fut
        door_state = door_state_fut.result()
        self.assertEqual(door_state['door_name'], 'test_door')

    async def test_door_states_late_sub(self):
        '''
        clients should receive all current door states on subscribe
        '''
        done = asyncio.Future()
        test_names = ['test_door', 'test_door_2']
        test_states = [make_door_state(name) for name in test_names]
        all_states = {}

        @self.client.on(topics.door_states)
        def on_door_states(door_state):
            all_states[door_state['door_name']] = door_state
            if len(all_states) == len(test_states) and \
                    all([x in test_names for x in all_states.keys()]):
                done.set_result(True)

        [self.rmf_io.on_door_state(x) for x in test_states]
        await self.client.emit('subscribe', topics.door_states)
        await asyncio.wait_for(done, 1)
