from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.cron import CronTrigger
from apscheduler.triggers.date import DateTrigger
from pytz import utc


class TimerJob:
    _scheduler = None

    def __init__(self):
        self._scheduler = AsyncIOScheduler()
        self._scheduler.configure(timezone=utc)  # utc

    def init_timer(self):
        self._scheduler.start()

    def close_timer(self):
        self._scheduler.shutdown(wait=True)

    def new_job(self, job_id: str, func: object, args: tuple, cron: str):
        return self._scheduler.add_job(
            id=job_id, func=func, args=args, trigger=DateTrigger(cron)
        )

    def delete_job(self, job_id: str):
        return self._scheduler.remove_job(job_id=job_id)

    def stop_job(self, job_id: str):
        return self._scheduler.pause_job(job_id=job_id)

    def replay_job(self, job_id: str):
        return self._scheduler.resume_job(job_id=job_id)

    def modify_job(self, job_id: str, func: object, args: tuple, cron: str):
        return self._scheduler.modify_job(
            job_id=job_id, func=func, args=args, trigger=CronTrigger.from_crontab(cron)
        )

    def get_job(self, job_id: str):
        return self._scheduler.get_job(job_id=job_id)

    def get_all(self):
        self._scheduler.get_jobs()


scheduler = TimerJob()
