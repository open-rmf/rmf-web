import asyncio
import threading

from fastapi import FastAPI

from api_server.__main__ import ServerManager


class TestServer:
    def __init__(self):
        self.server_manager = ServerManager()
        self._ready = threading.Semaphore(0)

        for srv in self.server_manager.servers:
            # uvloop does not work in background thread for some reason
            srv.config.loop = "none"

            def on_startup():
                self._ready.release()

            app: FastAPI = srv.config.app
            app.add_event_handler("startup", on_startup)

        self.loop: asyncio.AbstractEventLoop

    def run_in_background(self):
        def run():
            asyncio.set_event_loop(asyncio.new_event_loop())
            self.loop = asyncio.get_event_loop()
            self.server_manager.run()

        self.thread = threading.Thread(target=run)
        self.thread.start()

        for _ in range(len(self.server_manager.servers)):
            self._ready.acquire()  # pylint: disable=consider-using-with

    def shutdown(self):
        self.server_manager.shutdown()
        self.thread.join()


test_server = TestServer()
