from abc import ABC, abstractmethod
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
        self.auth_dep: Callable[..., User]
        self.rmf_repo: Callable[..., RmfRepository]
        self.logger: Logger
        self.static_files_repo: StaticFilesRepository

    @abstractmethod
    def rmf_events(self) -> RmfEvents:
        pass

    @abstractmethod
    def rmf_gateway(self) -> RmfGateway:
        pass

    @abstractmethod
    def rmf_bookkeeper(self) -> RmfBookKeeper:
        pass
