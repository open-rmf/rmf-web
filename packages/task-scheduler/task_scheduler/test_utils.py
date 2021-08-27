from tortoise import Tortoise


async def start_test_database():
    await Tortoise.init(
        db_url="sqlite://:memory:",
        modules={"models": ["task_scheduler.models.tortoise_models"]},
    )
    await Tortoise.generate_schemas()
