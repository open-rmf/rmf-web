from tortoise import models

# type (minutes, hours, Daily, monthly, weekly, fixed, yearly) - Enum

# frequency (Can be 1-7[days of week], 1-30(or 28)[days of month], 1-365[days of year] or null(for daily, fixed) - ArrayField(of ints) - [1, 7] OR [23] OR [235]OR null


class ScheduledTask(models.Model):
    id = models.IntField(pk=True)
    description = models.TextField()
    task_type = models.TextField()
    interval = models.IntField()
    interval_type = models.TextField()
    time_of_day = models.TimeField()
    created_at = models.DatetimeField(auto_now_add=True)
    start_date = models.DateTimeField()
    end_date = models.DateTimeField()

    # id SERIAL UNIQUE,                      -- unique identifier for the job
    # name varchar(64) NOT NULL,             -- human readable name for the job
    # description text,                      -- details about the job
    # schedule varchar(64) NOT NULL,         -- valid CRON expression for the job schedule
    # handler varchar(64) NOT NULL,          -- string representing handler for the job
    # args text NOT NULL,                    -- arguments for the job handler
    # enabled boolean NOT NULL DEFAULT TRUE, -- whether the job should be run
    # created_at timestamp NOT NULL,         -- when was the job created
    # updated_at timestamp NOT NULL,         -- when was the job updated
    # start_date timestamp,                  -- job should not run until this time
    # end_date timestamp,                    -- job should not run after this time
    # last_triggered_at timestamp,           -- when was the job last triggered
    # meta json                              -- additional metadata for the job
