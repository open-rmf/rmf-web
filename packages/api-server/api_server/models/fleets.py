from . import tortoise_models as ttm
from .health import BasicHealth


class RobotHealth(BasicHealth):
    @classmethod
    async def from_tortoise_orm(cls, obj: ttm.RobotHealth) -> "RobotHealth":
        return RobotHealth(
            id_=obj.id_,
            health_status=obj.health_status,
            health_message=obj.health_message,
        )
