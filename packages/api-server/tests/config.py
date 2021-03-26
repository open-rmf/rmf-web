# pylint: disable=line-too-long
config = {
    "host": "127.0.0.1",  # ip or hostname to bind the socket to
    "port": 8000,
    "db_url": "sqlite://:memory:",  # TODO: explain how to use different db
    "root_path": "",  # base path to mount the app on, must not end with '/'
    "socket_io_path": "socket.io",  # the path to handle socket.io connections
    "static_path": "/static",  # base path that static files should be served from, this is NOT relative to the root path
    "static_directory": "static",  # the directory where static files should be stored.
    "log_level": "CRITICAL",  # https://docs.python.org/3.8/library/logging.html#levels
    "jwt_public_key": None,  # path to a PEM encoded RSA public key which is used to verify JWT tokens, if the path is relative, it is based on the working dir. If `None`, authentication will be disabled.
}
