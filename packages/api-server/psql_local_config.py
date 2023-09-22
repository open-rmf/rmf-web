from sqlite_local_config import config

here = dirname(__file__)
run_dir = f"{here}/run"

config.update(
    {
        "db_url": "postgres://postgres:postgres@127.0.0.1:5432",
        "cache_directory": f"{run_dir}/cache",  # The directory where cached files should be stored.
    }
)
