import signal
from threading import Thread

import uvicorn

from .app import App
from .rmf_gateway_app import app as rmf_gateway_app

app = App()


def main():
    public_server = uvicorn.Server(
        uvicorn.Config(
            app=app,
            host=app.app_config.host,
            port=app.app_config.base_port,
            root_path=app.app_config.public_url.path,
            log_level=app.app_config.log_level.lower(),
            loop="asyncio",
        )
    )

    rmf_gateway_server = uvicorn.Server(
        uvicorn.Config(
            app=rmf_gateway_app,
            host=app.app_config.host,
            port=app.app_config.base_port + 1,
            root_path=app.app_config.public_url.path,
            log_level=app.app_config.log_level.lower(),
            loop="asyncio",
        )
    )

    def stop_servers(signum, frame):
        public_server.should_exit = True
        rmf_gateway_server.should_exit = True

    signal.signal(signal.SIGINT, stop_servers)
    signal.signal(signal.SIGTERM, stop_servers)

    Thread(target=lambda: public_server.run()).start()
    Thread(target=lambda: rmf_gateway_server.run()).start()


if __name__ == "__main__":
    main()
