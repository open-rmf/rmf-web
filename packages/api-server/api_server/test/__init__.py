from api_server.authenticator import JwtAuthenticator
from api_server.models import User

from .mocks import *
from .test_client import TestClient
from .test_data import *
from .test_fixtures import *

test_user = User(username="test_user", is_admin=True)
