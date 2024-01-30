from typing import TypeVar

from reactivex import Observable
from reactivex import operators as ops

T = TypeVar("T")


def filter_not_none(src: Observable[T | None]) -> Observable[T]:
    return src.pipe(ops.filter(lambda x: x is not None))  # type: ignore
