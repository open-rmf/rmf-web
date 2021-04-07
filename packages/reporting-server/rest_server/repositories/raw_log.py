from models.raw_log import RawLog


async def create_raw_log(logs: list):
    for log in logs:
        print(log)
        await RawLog.create(payload={log: log})
    return [{"username": "Rick"}, {"username": "Morty"}]
