from .app_config import app_config

TORTOISE_ORM = {
    "connections": {"default": app_config.db_url},
    "apps": {
        "models": {
            "models": ["models.tortoise_models", "aerich.models"],
            "default_connection": "default",
        },
    },
}
