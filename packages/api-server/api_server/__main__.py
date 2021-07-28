import uvicorn

from api_server.app import app


def main():
    uvicorn.run(
        app,
        host=app.app_config.host,
        port=app.app_config.port,
        root_path=app.app_config.public_url.path,
        log_level=app.app_config.log_level.lower(),
    )


if __name__ == "__main__":
    main()
