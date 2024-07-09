import schedule

from api_server.fast_io import singleton_dep


@singleton_dep
def get_scheduler():
    return schedule.Scheduler()
