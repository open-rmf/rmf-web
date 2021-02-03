from tortoise import Tortoise


class SQLRepository():
    def __init__(self, db_url: str):
        self.db_url = db_url

    async def init(self):
        await Tortoise.init(self.db_url, modules={'models': ['..models']})
        await Tortoise.generate_schemas()
