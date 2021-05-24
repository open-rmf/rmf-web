from tortoise import fields


class HealthStatusMixin:
    health_status = fields.CharField(max_length=255, null=True)
    health_message = fields.TextField(null=True)

    def to_dict(self):
        return {
            "health_status": self.health_status.value,  # false positive, pylint: disable=no-member
            "health_message": self.health_message,
        }
