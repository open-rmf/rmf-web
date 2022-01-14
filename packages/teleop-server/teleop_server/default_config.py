# pylint: disable=line-too-long
config = {
    "host": "127.0.0.1",  # ip or hostname to bind the socket to
    # ports used are:
    # base_port: public REST and socketio endpoints
    # base_port + 1: internal websocket endpoint for communication with rmf
    "base_port": 8000,
    # url that teleop-server is being served on.
    "public_url": "http://localhost:8000",
    "log_level": "WARNING",
    # path to a PEM encoded RSA public key which is used to verify JWT tokens, if the path is relative, it is based on the working dir.
    "jwt_public_key": None,
    # url to the oidc endpoint, used to authenticate rest requests, it should point to the well known endpoint, e.g.
    # http://localhost:8080/auth/realms/rmf-web/.well-known/openid-configuration.
    # NOTE: This is ONLY used for documentation purposes, the "jwt_public_key" will be the
    # only key used to verify a token.
    "oidc_url": None,
    # Audience the access token is meant for. Can also be an array.
    # Used to verify the "aud" claim.
    "aud": "localhost",
    # url or string that identifies the entity that issued the jwt token
    # Used to verify the "iss" claim
    # If iss is set to None, it means that authentication should be disabled
    "iss": None,
}
