from tortoise.contrib.pydantic.creator import pydantic_model_creator

from . import tortoise_models as ttm
from .health import BaseBasicHealthModel


class RobotHealth(pydantic_model_creator(ttm.RobotHealth), BaseBasicHealthModel):
    pass
