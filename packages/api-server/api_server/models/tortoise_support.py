from typing import Annotated, Generic, TypeVar

from pydantic import BeforeValidator
from tortoise.fields.relational import ReverseRelation

T = TypeVar("T")


def reverse_relation_to_list(v: ReverseRelation):
    if not type(v) is ReverseRelation:
        return v
    return list(v)


TortoiseReverseRelation = Annotated[list[T], BeforeValidator(reverse_relation_to_list)]
"""A helper type to validate pydantic models with tortoise models that has a reverse relation"""
