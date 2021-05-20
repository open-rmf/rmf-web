import jwt


class AuthenticationError(Exception):
    pass


class JwtAuthenticator:
    def __init__(self, pem_file: str, aud: str, iss: str):
        """
        Authenticates with a JWT token, the client must send an auth params with
        a "token" key.
        :param pem_file: path to a pem encoded certificate used to verify a token.
        """
        self.aud = aud
        self.iss = iss
        with open(pem_file, "br") as f:
            self._public_key = f.read()

    def verify_token(self, token: str):
        try:
            jwt.decode(
                token,
                self._public_key,
                algorithms=["RS256"],
                audience=self.aud,
                issuer=self.iss,
            )
        except jwt.InvalidTokenError as e:
            raise AuthenticationError(str(e)) from e
