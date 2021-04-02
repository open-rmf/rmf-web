from tortoise import Model, fields


class RawLog(Model):
    id = fields.UUIDField(pk=True)
    # JSONField https://tortoise-orm.readthedocs.io/en/latest/fields.html#tortoise.fields.data.IntField.field_type
    payload = fields.TextField()
    created = fields.DatetimeField(auto_now_add=True)

    def __str__(self):
        return self.payload
