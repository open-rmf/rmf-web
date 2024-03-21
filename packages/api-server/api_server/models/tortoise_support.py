from typing import Annotated, TypeVar

from pydantic import BeforeValidator
from tortoise.fields.relational import ReverseRelation

T = TypeVar("T")


def reverse_relation_to_list(v):
    if isinstance(v, ReverseRelation):
        return v
    return list(v)


TortoiseReverseRelation = Annotated[list[T], BeforeValidator(reverse_relation_to_list)]
"""A helper type to validate pydantic models with tortoise models that has a reverse relation"""
