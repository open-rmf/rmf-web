# pylint: disable=line-too-long
config = {
    "host": "127.0.0.1",  # ip or hostname to bind the socket to
    "port": 8000,
    "db_url": "sqlite://:memory:",
    # url that rmf-server is being served on.
    # When being a proxy, this must be the url that rmf-server is mounted on.
    # E.g. https://example.com/rmf/api/v1
    "public_url": "http://localhost:8000",
    "static_directory": "static",  # The directory where static files should be stored.
    "log_level": "WARNING",  # https://docs.python.org/3.8/library/logging.html#levels
    # path to a PEM encoded RSA public key which is used to verify JWT tokens, if the path is relative, it is based on the working dir.
    "jwt_public_key": None,
    # url to the oidc endpoint, used to authenticate rest requests, it should point to the well known endpoint, e.g.
    # http://localhost:8080/auth/realms/rmf-web/.well-known/openid-configuration.
    # NOTE: This is ONLY used for documentation purposes, the "jwt_public_key" will be the
    # only key used to verify a token.
    "oidc_url": None,
    # client id registered with the authentication provider, this will be used to verify the
    # "aud" claim.
    "client_id": "rmf-server",
}
