from typing import Sequence

from pydantic import BaseModel


class Labels(BaseModel):
    """
    Labels for a resource.
    """

    __root__: dict[str, str]

    @staticmethod
    def _parse_label(s: str) -> tuple[str, str]:
        sep = s.find("=")
        if sep == -1:
            return s, ""
        return s[:sep], s[sep + 1 :]

    @staticmethod
    def from_strings(labels: Sequence[str]) -> "Labels":
        return Labels(__root__=dict(Labels._parse_label(s) for s in labels))

    def to_strings(self) -> list[str]:
        return [f"{k}={v}" for k, v in self.__root__.items()]
