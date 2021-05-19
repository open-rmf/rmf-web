import asyncio
import os
import threading

import uvicorn


class BackgroundServer:
    def __init__(self, app):
        self.port = os.environ.get("RMF_SERVER_TEST_PORT", "8000")
        uvicorn_config = uvicorn.Config(
            app, "127.0.0.1", port=int(self.port), loop="asyncio", log_level="critical"
        )
        self.server = uvicorn.Server(uvicorn_config)
        self.server_thread = threading.Thread(target=self._run_server)
        self.base_url = f"http://127.0.0.1:{self.port}"

    def start(self):
        self.server_thread.start()

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
