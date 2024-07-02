import asyncio
from typing import Any, Coroutine

from typing_extensions import TypeGuard


def is_coroutine(value: object) -> TypeGuard[Coroutine[Any, Any, Any]]:
    return asyncio.iscoroutine(value)
