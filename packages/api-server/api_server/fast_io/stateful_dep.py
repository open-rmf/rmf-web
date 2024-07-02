import contextlib
from typing import Any, Self


class StatefulDep(contextlib.AbstractAsyncContextManager):
    """A helper base class to define stateful FastAPI dependencies

    FastAPI relies heavily on global objects for dependencies that must perist between
    requests. A simple implementation would be init the dependency at app startup and
    use save it in a global variable, but the issue is that it can cause issues in tests
    where the app will startup and shutdown multiple times.

    StatefulDep helps manage the lifecycle of such dependencies, subclass should
    implement cleanup logic in the __aexit__ method, during app startup, `set_instance`
    should be called which returns an async context.
    """

    _instance: Any | None = None

    @classmethod
    def get_instance(cls) -> Self:
        if cls._instance is None:
            raise RuntimeError("StatefulDep is not set")
        return cls._instance

    @contextlib.asynccontextmanager
    @classmethod
    async def set_instance(cls, instance: Self):
        try:
            async with instance:
                cls._instance = instance
                yield
        finally:
            pass
            cls._instance = None
