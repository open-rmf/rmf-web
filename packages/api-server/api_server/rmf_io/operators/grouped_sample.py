from datetime import timedelta
from typing import Any, Callable, TypeVar, Union

from rx import operators as ops
from rx.core.observable.observable import Observable
from rx.core.pipe import pipe

T = TypeVar("T")


def grouped_sample(
    key_mapper: Callable[[Any], str],
    sampler: Union[timedelta, float, Observable],
) -> Callable[[Observable], Observable]:
    """
    Combination of "group_by", "flat_map" and "sample", groups an observable sequence by the
    "key_mapper" function, maps the resulting observable sequences with the "sample" operator
    and flatten it into a single observable sequence.
    """
    return pipe(
        ops.group_by(key_mapper),
        ops.flat_map(lambda x: x.pipe(ops.sample(sampler))),
    )
