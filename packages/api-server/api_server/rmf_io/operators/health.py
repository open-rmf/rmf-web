from typing import Sequence

import rx
from rx import operators as op
from rx.core.operators.timestamp import Timestamp

from ...models import HealthStatus


def most_critical():
    """
    Maps an observable sequence of a sequence of timestamp of BasicHealthModel to an
    observable sequence of BasicHealthModel with the most critical health status. If there
    are multiple BasicHealthModel with the same criticality, the most recent item is
    chosen.
    """

    def criticality(health_status: HealthStatus):
        """
        Converts a health status into an int such that less healthy > more healthy.
        """
        if health_status == HealthStatus.HEALTHY:
            return 0
        if health_status == HealthStatus.UNHEALTHY:
            return 1
        if health_status == HealthStatus.DEAD:
            return 2
        raise Exception("unknown health status")

    def get_most_critical(health_statuses: Sequence[Timestamp]):
        """
        :param health_status: Sequence[Timestamp[HealthStatus]]
        """
        most_crit = health_statuses[0]
        for health in health_statuses:
            cur = criticality(most_crit.value.health_status)
            other = criticality(health.value.health_status)
            if other > cur:
                most_crit = health
            elif other == cur:
                if health.timestamp > most_crit.timestamp:
                    most_crit = health
        return most_crit.value

    return rx.pipe(op.map(get_most_critical))
