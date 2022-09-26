import argparse
import sys
from copy import copy
from os import makedirs
from os.path import basename, dirname, exists
from os.path import join as joinp
from typing import List, Sequence

import jinja2
from jinja2 import Environment, FileSystemLoader
from ros_translator.library import Message, PostProcessors, RosLibrary, Service

template_loader = FileSystemLoader(searchpath=joinp(dirname(__file__), "templates"))
template_env = Environment(loader=template_loader)
template_env.trim_blocks = True
template_env.keep_trailing_newline = True


# These types does not need a 'new' keyword
PRIMITIVE_TYPES = {
    "bool": "boolean",
    "byte": "number",
    "char": "number",
    "float32": "number",
    "float64": "number",
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
    "bool": "false",
    "byte": "0",
    "char": "0",
    "float32": "0",
    "float64": "0",
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
    "byte": "Uint8Array",
    "float32": "Float32Array",
    "float64": "Float64Array",
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
    def __init__(self, ros_type):
        self.type = None
        self.base_type = None
        self.is_primitive = False
        self.is_typed_array = False
        self.is_array = False
        self.default_value = None
        if hasattr(ros_type, "is_array") and ros_type.is_array:
            self.type = TYPED_ARRAY_TYPES.get(ros_type.type)
            self.default_value = "[]"
            if self.type is None:
                if ros_type.is_primitive_type():
                    self.type = f"{PRIMITIVE_TYPES[ros_type.type]}[]"
                else:
                    self.type = f"{ros_type.type}[]"
                self.is_array = True
                ros_element_type = copy(ros_type)
                ros_element_type.is_array = False
                self.element_type = JsType(ros_element_type)
            else:
                self.is_typed_array = True
            self.base_type = self.type[: len(self.type) - 2]
        else:
            self.type = PRIMITIVE_TYPES.get(ros_type.type)
            if ros_type.is_primitive_type():
                self.default_value = PRIMITIVE_TYPES_DEFAULT_VALUES[ros_type.type]
            else:
                self.default_value = f"new {ros_type.type}()"
            if self.type is None:
                self.type = ros_type.type
            self.base_type = self.type
        self.is_primitive = self.base_type in PRIMITIVE_JS_TYPES


def augment_message(msg: Message):
    for f in msg.spec.fields:
        f.js_type = JsType(f.type)
    return msg


def generate_modules(pkgs: Sequence[str], dstdir: str):
    roslib = RosLibrary(post_processors=PostProcessors(message=augment_message))
    all_pkgs = roslib.get_all_dependent_packages(*pkgs)
    all_pkg_index = [roslib.get_package_index(p) for p in all_pkgs]
    all_messages: List[Message] = []
    for p in all_pkg_index:
        all_messages.extend((roslib.get_message(m) for m in p.messages))

    all_services: List[Service] = []
    for p in all_pkg_index:
        all_services.extend((roslib.get_service(s) for s in p.services))

    template = template_env.get_template("ts-definition.j2")
    modules = []

    for msg in all_messages:
        if not isinstance(msg.spec, str):
            base_type = msg.spec.base_type
            pkg_name = base_type.pkg_name
            output_fpath = joinp(
                dstdir, pkg_name, msg.rel_dir, f"{msg.spec.base_type.type}.ts"
            )

            if not exists(dirname(output_fpath)):
                makedirs(dirname(output_fpath))

            print(f"Generating model {base_type}")
            template.stream(msg=msg).dump(output_fpath)
            modules.append(f"./{pkg_name}/{msg.rel_dir}/{base_type.type}")

    template = template_env.get_template("srv-ts-definition.j2")
    for srv in all_services:
        if not isinstance(srv.spec, str):
            output_fpath = joinp(
                dstdir, srv.spec.pkg_name, srv.rel_dir, f"{srv.spec.srv_name}.ts"
            )

            if not exists(dirname(output_fpath)):
                makedirs(dirname(output_fpath))

            print(f"Generating model {srv.full_type}")
            template.stream(srv=srv).dump(output_fpath)
            modules.append(f"./{srv.spec.pkg_name}/{srv.rel_dir}/{srv.spec.srv_name}")
    return modules


def generate_index(modules, dstdir):
    template = template_env.get_template("index.j2")

    output_fpath = joinp(dstdir, "index.ts")
    template.stream(
        modules=modules,
    ).dump(output_fpath)


def generate(pkgs: Sequence[str], outdir: str):
    print("Generating typescript interfaces")
    modules = generate_modules(pkgs, outdir)
    print("Generating index")
    generate_index(modules, outdir)
    print("Successfully generated typings")
