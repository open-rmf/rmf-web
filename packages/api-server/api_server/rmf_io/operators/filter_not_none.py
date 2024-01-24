from typing import Callable, TypeVar

from reactivex import Observable
from reactivex.operators import filter

T = TypeVar("T")


def filter_not_none(src: Observable[T | None]) -> Observable[T]:
    return src.pipe(filter(lambda x: x is not None))  # type: ignore
