import logging
import os

from api_server.app_config import app_config
from api_server.fast_io import singleton_dep


class CachedFilesRepository:
    def __init__(
        self,
        base_url: str,
        directory: str,
    ):
        """
        :param base_url: base url that cached files are served from. When running behind a proxy,
        this should be the url of the proxy.
        :param directory: location to write the files to.
        This should be the same directory that the cached files server is serving from.
        """
        self.base_url = base_url
        self.directory = directory

    async def __aenter__(self):
        os.makedirs(self.directory, exist_ok=True)

    def add_file(self, data: bytes, path: str) -> str:
        """
        Parameters:
            data:
            path: relative path to save the file to
        Returns:
            the url path of the new file
        """
        filepath = f"{self.directory}/{path}"
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "bw") as f:
            f.write(data)
        logging.info(f'saved new file "{filepath}"')
        urlpath = f"{self.base_url}/{path}"
        return urlpath


@singleton_dep
def get_cached_file_repo():
    os.makedirs(app_config.cache_directory, exist_ok=True)
    return CachedFilesRepository(
        f"{app_config.public_url.geturl()}/cache", app_config.cache_directory
    )
