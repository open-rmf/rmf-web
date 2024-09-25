from tortoise.fields import CharField, TextField
from tortoise.models import Model


class Mission(Model):
    name = CharField(255, index=True, unique=True)
    ui_schema = TextField(index=False)
    task_template = TextField(index=False)
