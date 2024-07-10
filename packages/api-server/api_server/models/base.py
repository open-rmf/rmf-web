from pydantic import BaseModel, ConfigDict


class PydanticModel(BaseModel):
    model_config = ConfigDict(
        from_attributes=True, json_schema_serialization_defaults_required=True
    )
