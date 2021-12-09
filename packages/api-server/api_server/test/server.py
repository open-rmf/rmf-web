import asyncio
import os
import threading
from concurrent.futures import Future

import uvicorn
from fastapi import FastAPI


class BackgroundServer:
    def __init__(self, app: FastAPI, port_offset: int = 0):
        self.app = app
        self.port = int(os.environ.get("RMF_SERVER_TEST_PORT", "8000")) + port_offset
        self.loop: asyncio.AbstractEventLoop
        uvicorn_config = uvicorn.Config(
            self.app,
            "127.0.0.1",
            port=int(self.port),
            loop="asyncio",
            log_level="error",
        )
        self.server = uvicorn.Server(uvicorn_config)
        self.server_thread = threading.Thread(target=self._run_server)
        self.base_url = f"http://127.0.0.1:{self.port}"

        self._ready = Future()

        @self.app.on_event("startup")
        def on_startup():
            self.loop = asyncio.get_event_loop()
            self._ready.set_result(True)

    def start(self):
        self.server_thread.start()
        self._ready.result(1)

    def stop(self):
        if self.server_thread:
            self.server.should_exit = True
            self.server_thread.join()

    def _run_server(self):
        self.server.run()
        loop = asyncio.get_event_loop()
        tasks = asyncio.all_tasks(loop=loop)

        async def cleanup():
            if len(tasks) == 0:
                return
            for task in tasks:
                task.cancel()
            await asyncio.wait(tasks)

        loop.run_until_complete(cleanup())
