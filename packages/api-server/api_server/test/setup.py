from asyncio.events import AbstractEventLoop

from api_server.app import app

from .server import BackgroundServer

loop: AbstractEventLoop
server = BackgroundServer(app)


def setup():
    server.start()


def teardown():
    server.stop()
