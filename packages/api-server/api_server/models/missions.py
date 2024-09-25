import pydantic


class Mission(pydantic.BaseModel):
    model_config = pydantic.ConfigDict(from_attributes=True)

    id: int
    name: str
    ui_schema: str = pydantic.Field(
        ..., description="ui schema according to https://jsonforms.io"
    )
    task_template: str = pydantic.Field(..., description="jinja2 template")
