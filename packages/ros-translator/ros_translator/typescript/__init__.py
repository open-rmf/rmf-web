import os
from os import makedirs
from os.path import dirname
from os.path import join as joinp
from typing import Sequence

from jinja2 import Environment, FileSystemLoader
from ros_translator.library import Message, RosLibrary, Service
from rosidl_parser.definition import (
    AbstractSequence,
    AbstractString,
    AbstractType,
    Array,
    BasicType,
    BoundedSequence,
    NamespacedType,
)

template_loader = FileSystemLoader(searchpath=joinp(dirname(__file__), "templates"))
template_env = Environment(loader=template_loader)
template_env.trim_blocks = True
template_env.keep_trailing_newline = True


# These types does not need a 'new' keyword
PRIMITIVE_TYPES = {
    "boolean": "boolean",
    "octet": "number",
    "char": "number",
    "float": "number",
    "double": "number",
    "int8": "number",
    "int16": "number",
    "int32": "number",
    "int64": "number",
    "string": "string",
    "wstring": "string",
    "uint8": "number",
    "uint16": "number",
    "uint32": "number",
    "uint64": "number",
}

PRIMITIVE_JS_TYPES = [
    "string",
    "boolean",
    "number",
]

PRIMITIVE_TYPES_DEFAULT_VALUES = {
    "boolean": "false",
    "octet": "0",
    "char": "0",
    "float": "0",
    "double": "0",
    "int8": "0",
    "int16": "0",
    "int32": "0",
    "int64": "0",
    "string": "''",
    "wstring": "''",
    "uint8": "0",
    "uint16": "0",
    "uint32": "0",
    "uint64": "0",
}

TYPED_ARRAY_TYPES = {
    "char": "Uint8Array",
    "octet": "Uint8Array",
    "float": "Float32Array",
    "double": "Float64Array",
    "int8": "Int8Array",
    "int16": "Int16Array",
    "int32": "Int32Array",
    "int64": "BigInt64Array",
    "uint8": "Uint8Array",
    "uint16": "Uint16Array",
    "uint32": "Uint32Array",
    "uint64": "BigUint64Array",
}


class JsType:
    def __init__(self, ros_type: AbstractType):
        self.type: str
        self.full_type: str
        self.elem_type: JsType
        self.ros_type = ros_type
        self.is_primitive: bool = False
        self.is_typed_array: bool = False
        self.is_array: bool = False
        self.is_bounded: bool = False
        self.array_size: int
        self.default_value: str

        if isinstance(ros_type, BasicType):
            self.type = PRIMITIVE_TYPES[ros_type.typename]
            self.full_type = self.type
            self.is_primitive = True
            self.default_value = PRIMITIVE_TYPES_DEFAULT_VALUES[ros_type.typename]
        elif isinstance(ros_type, AbstractString):
            self.type = "string"
            self.full_type = self.type
            self.is_primitive = True
            self.default_value = "''"
        elif isinstance(ros_type, NamespacedType):
            self.type = ros_type.name
            self.full_type = ".".join(ros_type.namespaces) + f".{ros_type.name}"
            self.default_value = f"new {self.full_type}()"
        elif isinstance(ros_type, AbstractSequence) or isinstance(ros_type, Array):
            self.is_array = True
            self.elem_type = JsType(ros_type.value_type)
            if isinstance(self.elem_type.ros_type, BasicType):
                self.type = TYPED_ARRAY_TYPES[self.elem_type.ros_type.typename]
                self.is_typed_array = True
                self.default_value = f"new {self.type}()"
            else:
                self.type = f"Array<{self.elem_type.full_type}>"
                self.default_value = "[]"
            self.full_type = self.type
            if isinstance(ros_type, BoundedSequence):
                self.is_bounded = True
                self.array_size = ros_type.maximum_size
            if isinstance(ros_type, Array):
                self.array_size = ros_type.size
        else:
            raise ValueError(f"{type(ros_type)} is not supported")


def generate_message(msg: Message, dstdir: str):
    template = template_env.get_template("ts-definition.j2")
    output_fpath = joinp(dstdir, f"{msg.full_type_name}.ts")
    makedirs(dirname(output_fpath), exist_ok=True)
    print(f"Generating model {msg.full_type_name}")  # type: ignore
    template.stream(msg=msg).dump(output_fpath)


def generate_service(srv: Service, dstdir: str):
    generate_message(srv.request, dstdir)
    generate_message(srv.response, dstdir)
    output_fpath = joinp(dstdir, f"{srv.full_type_name}.ts")
    template = template_env.get_template("srv-ts-definition.j2")
    template.stream(srv=srv).dump(output_fpath)


def generate_modules(pkgs: Sequence[str], dstdir: str):
    roslib = RosLibrary(type_processor=JsType)
    all_pkgs = set((*pkgs, *roslib.get_all_package_dependencies(*pkgs)))
    all_pkg_index = [roslib.get_package_index(p) for p in all_pkgs]
    all_messages = [
        roslib.get_message(name)
        for p in all_pkg_index
        for name, _ in p.root_ns.all_messages()
    ]
    all_services = [
        roslib.get_service(name)
        for p in all_pkg_index
        for name, _ in p.root_ns.all_services()
    ]

    for msg in all_messages:
        generate_message(msg, dstdir)

    for srv in all_services:
        generate_service(srv, dstdir)


def generate_index(dstdir: str):
    for dirpath, dirnames, filenames in os.walk(dstdir):
        with open(joinp(dirpath, "index.ts"), mode="w") as f:
            f.write(
                "\n".join(
                    f"export * from './{os.path.splitext(m)[0]}'"
                    for m in filenames
                    if m != "index.ts"
                )
            )
            f.write("\n\n")
            f.write("\n".join(f"export * as {m} from './{m}'" for m in dirnames))
            f.write("\n")


def generate(pkgs: Sequence[str], outdir: str):
    print("Generating typescript interfaces")
    generate_modules(pkgs, outdir)
    print("Generating index")
    generate_index(outdir)
    print("Successfully generated typings")
