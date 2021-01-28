import asyncio
from logging import Logger
from typing import Union, Dict

from rx.subject import Subject

import socketio

from .topics import topics


class RmfIO():
    def __init__(self, loop: asyncio.AbstractEventLoop, logger: Logger):
        self.logger = logger
        self.loop = loop
        self.room_records: Dict[str, dict] = {}
        self.sio = socketio.AsyncServer(async_mode='asgi')
        self.asgi_app = socketio.WSGIApp(self.sio)

        self.sio.on('subscribe', self._on_subscribe)

        self._door_states: Dict[str, dict] = {}
        self.door_states = Subject()
        self.room_records[topics.door_states] = self._door_states

    async def on_door_state(self, state: dict):
        self._door_states[state['door_name']] = state
        await self.sio.emit(topics.door_states, state, to=topics.door_states)
        self.door_states.on_next(state)
        self.logger.debug(f'emitted message to room "{topics.door_states}"')

    async def _on_subscribe(self, sid, topic):
        self.logger.info(f'client: {sid}, room: {topic}, got new subscription')
        records = self.room_records[topic]
        if records:
            coros = [self.sio.emit(topic, rec, sid) for rec in records]
            await asyncio.gather(coros)
            self.logger.info(f'client: {sid}, room: {topic}, emitted existing records to new subscriber')
        else:
            self.logger.info(f'client: {sid}, room: {topic}, skipped emitting initial records (no existing records)')
        self.sio.enter_room(sid, topic)
        self.logger.info(f'added "{sid}" to room "{topic}"')
