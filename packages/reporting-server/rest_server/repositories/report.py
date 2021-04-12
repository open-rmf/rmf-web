from models.raw_log import RawLog, RawLog_Pydantic


async def get_all_raw_logs():
    return await RawLog_Pydantic.from_queryset(RawLog.all())


async def get_doors_state():
    return "work in progress"
