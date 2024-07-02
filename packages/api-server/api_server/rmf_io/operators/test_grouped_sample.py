import unittest

from reactivex import Subject
from reactivex.scheduler.historicalscheduler import HistoricalScheduler

from .grouped_sample import grouped_sample


class TestGroupedSample(unittest.TestCase):
    def test_grouped_sample(self):
        subject = Subject()

        result = []
        scheduler = HistoricalScheduler()
        subject.pipe(grouped_sample(lambda x: x[0], 10)).subscribe(
            result.append, scheduler=scheduler
        )

        subject.on_next(("key", "first"))
        subject.on_next(("key_2", "first_2"))
        scheduler.advance_by(5)
        subject.on_next(("key", "second"))
        scheduler.advance_by(5)

        # check that only the latest of each key is returned
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][1], "second")
        self.assertEqual(result[1][1], "first_2")
