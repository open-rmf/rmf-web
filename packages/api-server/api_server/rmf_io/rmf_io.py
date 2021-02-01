import asyncio
import logging
from typing import Optional

from rx.subject import Subject

import socketio

from .topics import topics
from .authenticator import Authenticator, StubAuthenticator


class RmfIO():
    def __init__(
        self,
        sio: socketio.AsyncServer,
        *,
        logger: logging.Logger = None,
        loop: asyncio.AbstractEventLoop = None,
        authenticator: Authenticator = StubAuthenticator(),
    ):
        self.sio = sio
        self.logger = logger or sio.logger
        self.loop = loop or asyncio.get_event_loop()
        self.authenticator = authenticator
        self.room_records: Dict[str, dict] = {}

        self.sio.on('connect', self._on_connect)
        self.sio.on('disconnect', self._on_disconnect)
        self.sio.on('subscribe', self._on_subscribe)

        self._door_states: Dict[str, dict] = {}
        self.door_states = Subject()
        self.room_records[topics.door_states] = self._door_states

    def on_door_state(self, state: dict):
        self._door_states[state['door_name']] = state

        async def emit_task():
            await self.sio.emit(topics.door_states, state, to=topics.door_states)
            self.logger.debug(
                f'emitted message to room "{topics.door_states}"')
        self.loop.create_task(emit_task())
        self.door_states.on_next(state)

    async def _on_subscribe(self, sid, topic):
        self.logger.info(f'[{sid}] got new subscription for "{topic}"')
        records: dict = self.room_records[topic]
        if records:
            coros = [self.sio.emit(topic, rec, sid)
                     for rec in records.values()]
            await asyncio.gather(*coros)
            self.logger.info(
                f'[{sid}] emitted existing records to new subscriber')
        else:
            self.logger.info(
                f'[{sid}] skipped emitting initial records (no existing records)')
        self.sio.enter_room(sid, topic)
        self.logger.info(f'[{sid}] added to room "{topic}"')

        # TODO: support unsubscribe
        await self.sio.emit('subscribe', 'ok', to=sid)

    def _on_connect(self, sid: str, environ: dict, auth: Optional[dict] = None):
        try:
            self.authenticator.authenticate(environ, auth)
            self.logger.info(
                f'[{sid}] new connection from "{environ["REMOTE_ADDR"]}:{environ["REMOTE_PORT"]}"')
            return True
        except e:
            self.logger.error(f'authentication failed: {e}')
            return False

    def _on_disconnect(self, sid):
        self.logger.info(f'[{sid}] disconnected')
