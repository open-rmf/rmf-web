from tortoise import Tortoise

# need to init models before calling `pydantic_model_creator` https://tortoise-orm.readthedocs.io/en/latest/examples/pydantic.html?highlight=serializer#relations-early-init
Tortoise.init_models(["api_server.models.tortoise_models"], "models")
