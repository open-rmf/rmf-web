import asyncio
import signal
from typing import List

import uvicorn
from tortoise import Tortoise

from .app import app
from .app_config import app_config
from .rmf_gateway_app import app as rmf_gateway_app
from .ros import ros_shutdown, ros_spin


class CustomServer(uvicorn.Server):
    """
    Server without signal handlers.

    This is so that we can use uvloop while still supporting ctrl+c exit.
    uvicorn does not install signal handlers on non main thread, but uvloop does not work on
    non main thread for some reason.
    """

    def install_signal_handlers(self) -> None:
        pass


class TortoiseContext:
    async def __aenter__(self):
        await Tortoise.init(
            db_url=app_config.db_url,
            modules={"models": ["api_server.models.tortoise_models"]},
        )
        # FIXME: do this outside the app as recommended by the docs
        await Tortoise.generate_schemas()

    async def __aexit__(self, *args):
        await Tortoise.close_connections()


class RosContext:
    def __enter__(self):
        ros_spin()

    def __exit__(self, *args):
        ros_shutdown()


class AppContext:
    def __init__(self):
        self._tortoise = TortoiseContext()
        self._ros = RosContext()

    async def __aenter__(self):
        self._ros.__enter__()
        await self._tortoise.__aenter__()

    async def __aexit__(self, *args):
        await self._tortoise.__aexit__(*args)
        self._ros.__exit__(*args)


class ServerManager:
    def __init__(self):
        self.servers: List[CustomServer] = []

        self.servers.append(
            CustomServer(
                uvicorn.Config(
                    app=app,
                    host=app_config.host,
                    port=app_config.base_port,
                    root_path=app_config.public_url.path,
                    log_level=app_config.log_level.lower(),
                )
            )
        )

        self.servers.append(
            CustomServer(
                uvicorn.Config(
                    app=rmf_gateway_app,
                    host=app_config.host,
                    port=app_config.base_port + 1,
                    root_path=app_config.public_url.path,
                    log_level=app_config.log_level.lower(),
                )
            )
        )

    def run(self):
        for srv in self.servers:
            srv.config.setup_event_loop()

        async def serve():
            async with AppContext():
                await asyncio.gather(*[srv.serve() for srv in self.servers])

        asyncio.get_event_loop().run_until_complete(serve())

    def shutdown(self):
        for srv in self.servers:
            if srv.should_exit:
                srv.force_exit = True
            else:
                srv.should_exit = True


def main():
    server_manager = ServerManager()

    def stop_servers(_signum, _frame):
        server_manager.shutdown()

    signal.signal(signal.SIGINT, stop_servers)
    signal.signal(signal.SIGTERM, stop_servers)

    server_manager.run()


if __name__ == "__main__":
    main()
