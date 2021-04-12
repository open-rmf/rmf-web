from .doors_state import doors_state_parser

"""
This function dispatchs to the correct handler dependending on the text content.
"""


async def log_model_dispacher(fullstring: str):
    if "door states" in fullstring.lower():
        return doors_state_parser(fullstring)
    if "door health" in fullstring.lower():
        pass
    if "lift states" in fullstring.lower():
        pass
    if "lift health" in fullstring.lower():
        pass
    if "dispenser states" in fullstring.lower():
        pass
    if "dispenser health" in fullstring.lower():
        pass
    if "fleet states" in fullstring.lower():
        pass
    if "robot health" in fullstring.lower():
        pass
    if "tasks" in fullstring.lower():
        pass
