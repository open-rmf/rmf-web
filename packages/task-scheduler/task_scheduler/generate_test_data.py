# conflicts with isort because of local non-relative import
# pylint: disable=wrong-import-order


import logging
import sys
from datetime import datetime, timedelta

from dependencies import logger
from models.tortoise_models import TaskRule
from tortoise import Tortoise, run_async

from task_scheduler.app_config import default_config
from task_scheduler.models.tortoise_models.helpers.task_rule_definition import (
    FrequencyEnum,
)
from task_scheduler.models.tortoise_models.task_rule import TaskTypeEnum

logger = logging.getLogger("clean_script")
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)


logger.info("started data creation")


async def create_test_data():
    now = datetime.utcnow()
    future = now + timedelta(days=2)

    await TaskRule.create(
        name="test65",
        task_type=TaskTypeEnum.LOOP,
        frequency=1,
        frequency_type=FrequencyEnum.ONCE,
        first_day_to_apply_rule=now,
        start_datetime=now,
    )

    await TaskRule.create(
        name="test_minutely",
        task_type=TaskTypeEnum.LOOP,
        frequency=60,
        frequency_type=FrequencyEnum.MINUTELY,
        first_day_to_apply_rule=now,
        start_datetime=now,
        end_datetime=future,
    )


async def run():
    await Tortoise.init(
        db_url=default_config.db_url,
        modules={"models": ["task_scheduler.models.tortoise_models"]},
        # generate_schemas=True,
    )
    # Tortoise.init_models(["task_scheduler.models.tortoise_models"], "models")

    await Tortoise.generate_schemas()
    await create_test_data()


def main():
    run_async(run())


if __name__ == "__main__":
    main()
