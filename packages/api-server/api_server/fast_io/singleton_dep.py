import contextlib
from asyncio import iscoroutine
from typing import Any, Callable, Coroutine, Generic, TypeVar

T = TypeVar("T")


class SingletonDep(Generic[T], contextlib.AbstractAsyncContextManager):
    def __init__(
        self,
        factory: (
            Callable[[], T]
            | Callable[[], Coroutine[Any, Any, T]]
            | Callable[[], contextlib.AbstractContextManager[T]]
            | Callable[[], contextlib.AbstractAsyncContextManager[T]]
        ),
    ):
        self.factory = factory
        self.instance: T | None = None
        self.context: (
            contextlib.AbstractContextManager[T]
            | contextlib.AbstractAsyncContextManager[T]
            | None
        ) = None

    async def __aenter__(self):
        result = self.factory()
        if isinstance(result, contextlib.AbstractContextManager):
            self.context = result
            self.instance = self.context.__enter__()
        elif isinstance(result, contextlib.AbstractAsyncContextManager):
            self.context = result
            self.instance = await self.context.__aenter__()
        elif iscoroutine(result):
            self.instance = await result
        else:
            self.instance = result

    async def __aexit__(self, exc_type, exc_value, traceback):
        if isinstance(self.context, contextlib.AbstractContextManager):
            self.context.__exit__(None, None, None)
        elif isinstance(self.context, contextlib.AbstractAsyncContextManager):
            await self.context.__aexit__(None, None, None)
        self.instance = None

    def __call__(self) -> T:
        if self.instance is None:
            raise AssertionError(
                f"{self.factory.__name__} is not set, is the context entered?"
            )
        return self.instance


def singleton_dep(
    f: (
        Callable[[], T]
        | Callable[[], Coroutine[Any, Any, T]]
        | Callable[[], contextlib.AbstractContextManager[T]]
        | Callable[[], contextlib.AbstractAsyncContextManager[T]]
    )
) -> SingletonDep[T]:
    """A function decorator define stateful FastAPI dependencies

    FastAPI relies heavily on global objects for dependencies that must perist between
    requests. A simple implementation would be init the dependency at app startup and
    use save it in a global variable, but the issue is that it can cause issues in tests
    where the app will startup and shutdown multiple times.

    singleton_dep helps manage the lifecycle of such dependencies, it should be used
    to decorate a function that return an instance of the dependency, singleton_dep
    will convert that to an async context manager, which when entered, will call
    the underlying function and cache the result.
    """

    return SingletonDep(f)
