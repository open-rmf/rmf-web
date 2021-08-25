# import schedule
from datetime import datetime, timedelta

from fastapi.applications import FastAPI

from task_scheduler.models.tortoise_models.scheduled_task import ScheduledTask

# def setup_scheduler(app: FastAPI):
#     scheduler.start()
# scheduler.add_job(get_scheduled_tasks, 'interval', seconds=5)


async def get_scheduled_tasks():
    """Get all scheduled tasks from the database."""
    tasks = await ScheduledTask.filter(
        task_datetime__lt=datetime.utcnow() + timedelta(minutes=10)
    )
    print(tasks)
    for task in tasks:
        time = task.task_datetime.time()
        print(f"{time} {task.task_name}")

        def job():
            print(f"{time} {task.task_name}")

        # schedule.every().day.at().do(job)


# def job_that_executes_once():
#     # Do some work that only needs to happen once...
#     return schedule.CancelJob


# schedule.every().day.at('22:30').do(job_that_executes_once)
