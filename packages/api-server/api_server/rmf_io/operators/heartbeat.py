from typing import Any, Callable, cast

from rx.core.observable.observable import Observable
from rx.core.pipe import pipe
from rx.operators import buffer_with_time_or_count, distinct_until_changed
from rx.operators import map as rx_map


def heartbeat(liveliness) -> Callable[[Observable], Observable]:
    """
    Projects a source observable sequence to a boolean. The resulting value is True when there
    is an observable emitted within the liveliness factor and False otherwise.
    Args:
        liveliness: The amount of time (in seconds) between each event for the observable to be
        considered alive. Note that an observable may stay alive for up to 2x liveliness between
        each event as this operator checks for items emitted between this window.
    """
    return pipe(
        buffer_with_time_or_count(liveliness, 1),
        rx_map(cast(Any, lambda items: len(items) > 0)),
        distinct_until_changed(),
    )
