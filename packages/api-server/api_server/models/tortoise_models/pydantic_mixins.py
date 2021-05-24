from pydantic import BaseModel
from tortoise.contrib.pydantic import pydantic_model_creator


def with_pydantic(Base):
    class WithPydantic(Base):
        PydanticModel = pydantic_model_creator(Base)

        def get_pydantic(self) -> BaseModel:
            return self.PydanticModel.from_orm(self)

    return WithPydantic
