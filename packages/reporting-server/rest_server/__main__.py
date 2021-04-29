import asyncio

from uvicorn import Config, Server

from .app import get_app
from .app_config import SystemMode, app_config


class MyServer(Server):
    # pylint: disable=invalid-overridden-method
    async def run(self, sockets=None):
        self.config.setup_event_loop()
        return await self.serve(sockets=sockets)


async def run():
    apps = []
    config1 = Config(
        get_app(SystemMode.FLUENTD),
        host=app_config.host,
        port=app_config.port_fluentd,
        root_path=app_config.public_url.path,
        log_level=app_config.log_level.lower(),
    )
    config2 = Config(
        get_app(SystemMode.REPORT),
        host=app_config.host,
        port=app_config.port,
        root_path=app_config.public_url.path,
        log_level=app_config.log_level.lower(),
    )

    server1 = MyServer(config=config1)
    server2 = MyServer(config=config2)

    apps.append(server1.run())
    apps.append(server2.run())

    return await asyncio.gather(*apps)


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())


if __name__ == "__main__":
    main()
