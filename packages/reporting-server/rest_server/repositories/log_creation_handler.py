from models.raw_log import RawLog
from parsers.log_type_parser import get_log_type


# Function that receives all the logs and store them on the database
async def create_raw_log(logs: list):
    for log in logs:
        print(log)
        log_level = get_log_type(log["log"])
        await RawLog.create(level=log_level, payload=log, message=log["log"])
    return "The logs were saved correctly"


# We want to grab specific data from this list of strings, so we need to preprocess
# this information
# async def create_clean_log(logs: list):
#     for log in logs:
#         print(log)
#         log_type = get_log_type(log)
#         # it should get the model instance to add the data
#         model, value = log_model_dispacher(log)
#         await model.create(*value)
#     return "Logs were saved correctly"
