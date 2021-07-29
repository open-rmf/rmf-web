from tortoise.contrib.pydantic import pydantic_model_creator

from .tortoise import ScheduledTask, TaskRule

TaskRule_Pydantic = pydantic_model_creator(TaskRule, name="TaskRule")

ScheduledTask_Pydantic = pydantic_model_creator(ScheduledTask, name="ScheduledTask")
