# pylint: disable=line-too-long
config = {
    "host": "127.0.0.1",  # ip or hostname to bind the socket to
    "port": 8000,
    "db_url": "sqlite://:memory:",  # TODO: explain how to use different db
    "root_path": "",  # base path to mount the app on, must not end with '/'
    "socket_io_path": "socket.io",  # the path to handle socket.io connections
    "static_directory": "static",  # The directory where static files should be stored.
    # If running behind a proxy, this should be the url of the proxy, else it should be
    # the same url that the server is bounded to.
    "proxy_url": "http://localhost:8000",
    "log_level": "WARNING",  # https://docs.python.org/3.8/library/logging.html#levels
    # path to a PEM encoded RSA public key which is used to verify JWT tokens, if the path is relative, it is based on the working dir.
    # Used to authenticate socketio requests.
    "jwt_public_key": None,
    # url to the oidc endpoint, used to authenticate rest requests, it should point to the well known endpoint, e.g.
    # http://localhost:8080/auth/realms/rmf-web/.well-known/openid-configuration
    "oidc_url": None,
    # client id registered with the authentication provider, this will be used to verify the
    # "aud" claim.
    "client_id": "rmf-server",
}
