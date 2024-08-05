import os
from os import makedirs
from os.path import dirname
from os.path import join as joinp
from typing import Sequence

from jinja2 import Environment, FileSystemLoader
from ros_translator.library import Message, Namespace, RosLibrary
from rosidl_parser.definition import (
    AbstractSequence,
    AbstractString,
    AbstractType,
    Array,
    BasicType,
    BoundedSequence,
    NamespacedType,
    UnboundedSequence,
)

template_loader = FileSystemLoader(searchpath=dirname(__file__))
template_env = Environment(loader=template_loader)
template_env.trim_blocks = True
template_env.keep_trailing_newline = True

PRIMITIVE_TYPES = {
    "boolean": "bool",
    "octet": "Annotated[int, pydantic.Field(ge=0, le=255)]",
    "float": "float",
    "double": "float",
    "int8": "Annotated[int, pydantic.Field(ge=-128, le=127)]",
    "int16": "Annotated[int, pydantic.Field(ge=-32768, le=32767)]",
    "int32": "Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]",
    "int64": "int",
    "uint8": "Annotated[int, pydantic.Field(ge=0, le=255)]",
    "uint16": "Annotated[int, pydantic.Field(ge=0, le=65535)]",
    "uint32": "Annotated[int, pydantic.Field(ge=0, le=4294967295)]",
    "uint64": "Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]",
}

DEFAULT_VALUES = {
    "boolean": "False",
    "octet": "0",
    "float": "0",
    "double": "0",
    "int8": "0",
    "int16": "0",
    "int32": "0",
    "int64": "0",
    "uint8": "0",
    "uint16": "0",
    "uint32": "0",
    "uint64": "0",
}

ARRAY_TYPES = {
    "octet": "bytes",
    "uint8": "bytes",
}

DEFAULT_ARRAY_VALUES = {
    "octet": "bytes()",
    "uint8": "bytes()",
}


class PydanticType:
    typename: str
    elem_type: "PydanticType"

    def __init__(self, ros_type: AbstractType):
        if isinstance(ros_type, BasicType):
            self.typename = PRIMITIVE_TYPES[ros_type.typename]
        elif isinstance(ros_type, AbstractString):
            self.typename = "str"
        elif isinstance(ros_type, NamespacedType):
            self.typename = "_".join(ros_type.namespaced_name())
        elif isinstance(ros_type, AbstractSequence) or isinstance(ros_type, Array):
            self.elem_type = PydanticType(ros_type.value_type)
            self.typename = self._get_array_type(ros_type, self.elem_type.typename)
            return
        else:
            raise ValueError(f"{type(ros_type)} is not supported")

    def _get_array_type(self, ros_type: AbstractSequence | Array, elem_typename: str):
        if elem_typename in ARRAY_TYPES:
            return ARRAY_TYPES[elem_typename]

        if isinstance(ros_type, BoundedSequence):
            return f"Annotated[list[{elem_typename}], pydantic.Field(max_length={ros_type.maximum_size})]"
        elif isinstance(ros_type, UnboundedSequence):
            return f"list[{elem_typename}]"
        elif isinstance(ros_type, Array):
            return f"Annotated[list[{elem_typename}], pydantic.Field(min_length={ros_type.size}, max_length={ros_type.size})]"
        else:
            raise ValueError(f"{type(ros_type)} is not supported")


def relative_import(base: Message, to: Message) -> str:
    base_ns = list(base.idl.structure.namespaced_type.namespaces)
    to_ns = list(to.idl.structure.namespaced_type.namespaces)
    i = 0
    for a, b in zip(base_ns, to_ns):
        if a != b:
            break
        i += 1
    relative = "." * (len(base_ns) - i) + ".".join((*to_ns[i:], to.name))
    full_alias = to.full_type_name.replace("/", "_")
    return f"from .{relative} import {to.name} as {full_alias}"


def generate_messages(roslib: RosLibrary, pkg: str, outdir: str):
    template = template_env.get_template("msg.j2")
    pkg_index = roslib.get_package_index(pkg)
    for msg_type, _ in pkg_index.root_ns.all_messages():
        msg = roslib.get_message(msg_type)
        outpath = joinp(outdir, f"{msg.full_type_name}.py")
        os.makedirs(os.path.dirname(outpath), exist_ok=True)
        print(outpath)
        template.stream(msg=msg, relative_import=relative_import).dump(outpath)


def generate_init(namespace: Namespace, outdir: str):
    with open(
        joinp(outdir, namespace.full_name, "__init__.py"), mode="w", encoding="utf8"
    ) as f:
        for m in namespace.messages.values():
            name = m.structure.namespaced_type.name
            f.write(f"from .{name} import {name}\n")
        for ns in namespace.namespaces.values():
            f.write(f"from . import {ns.name}\n")
    for ns in namespace.namespaces.values():
        generate_init(ns, outdir)


def generate_modules(pkgs: Sequence[str], outdir: str):
    roslib = RosLibrary(type_processor=PydanticType)
    all_pkg_index = roslib.get_all_interfaces(*pkgs)

    for pkg_index in all_pkg_index:
        pkg_outdir = joinp(outdir, pkg_index.pkg_name)
        makedirs(pkg_outdir, exist_ok=True)
        generate_messages(roslib, pkg_index.pkg_name, outdir)
        generate_init(pkg_index.root_ns, outdir)

    with open(joinp(outdir, "__init__.py"), mode="w", encoding="utf8"):
        pass


def generate(pkgs: Sequence[str], outdir: str):
    print("Generating pydantic interfaces")
    generate_modules(pkgs, outdir)
