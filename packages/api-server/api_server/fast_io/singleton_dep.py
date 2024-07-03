import contextlib
from typing import Any, Self


class SingletonDep(contextlib.AbstractAsyncContextManager):
    """A helper base class to define stateful FastAPI dependencies

    FastAPI relies heavily on global objects for dependencies that must perist between
    requests. A simple implementation would be init the dependency at app startup and
    use save it in a global variable, but the issue is that it can cause issues in tests
    where the app will startup and shutdown multiple times.

    SingletonDep helps manage the lifecycle of such dependencies, subclass should
    implement cleanup logic in the __aexit__ method, during app startup, `set_instance`
    should be called which returns an async context.
    """

    _instance: Any | None = None

    @classmethod
    def get_instance(cls) -> Self:
        if cls._instance is None:
            raise AssertionError(f"{cls.__name__} is not set, is the context entered?")
        return cls._instance

    @classmethod
    @contextlib.asynccontextmanager
    async def set_instance(cls, instance: Self):
        try:
            async with instance:
                cls._instance = instance
                yield
        finally:
            pass
            cls._instance = None
