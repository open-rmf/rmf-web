from typing import Union

import rx
from rx import operators as ops


def grouped_sample(
    key_mapper: ops.Mapper,
    sampler: Union[ops.timedelta, float, rx.Observable],
):
    """
    Combination of "group_by", "flat_map" and "sample", groups an observable sequence by the
    "key_mapper" function, maps the resulting observable sequences with the "sample" operator
    and flatten it into a single observable sequence.
    """
    return rx.pipe(
        ops.group_by(key_mapper),
        ops.flat_map(lambda x: x.pipe(ops.sample(sampler))),
    )
