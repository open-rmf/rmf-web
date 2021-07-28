from abc import ABC
from logging import Logger
from typing import Callable

from api_server.authenticator import JwtAuthenticator
from api_server.gateway import RmfGateway
from api_server.models import User
from api_server.repositories import RmfRepository, StaticFilesRepository
from api_server.rmf_io import RmfBookKeeper, RmfEvents


class BaseApp(ABC):
    def __init__(self):
        self.authenticator: JwtAuthenticator
        self.auth_dep: Callable[[], User]
        self.logger: Logger
        self.rmf_events: RmfEvents
        self.rmf_repo: RmfRepository
        self.static_files_repo: StaticFilesRepository
        self.rmf_gateway: RmfGateway
        self.rmf_bookkeeper: RmfBookKeeper
