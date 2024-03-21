from os import makedirs
from os.path import dirname
from os.path import join as joinp
from typing import Sequence

from jinja2 import Environment, FileSystemLoader
from ros_translator.library import Message, PackageIndex, PostProcessors, RosLibrary

template_loader = FileSystemLoader(searchpath=dirname(__file__))
template_env = Environment(loader=template_loader)
template_env.trim_blocks = True
template_env.keep_trailing_newline = True

PRIMITIVE_TYPES = {
    "bool": "bool",
    "byte": "Annotated[int, pydantic.Field(ge=0, le=255)]",
    "char": "Annotated[int, pydantic.Field(ge=0, le=255)]",
    "float32": "float",
    "float64": "float",
    "int8": "Annotated[int, pydantic.Field(ge=-128, le=127)]",
    "int16": "Annotated[int, pydantic.Field(ge=-32768, le=32767)]",
    "int32": "Annotated[int, pydantic.Field(ge=-2147483648, le=2147483647)]",
    "int64": "int",
    "string": "str",
    "wstring": "str",
    "uint8": "Annotated[int, pydantic.Field(ge=0, le=255)]",
    "uint16": "Annotated[int, pydantic.Field(ge=0, le=65535)]",
    "uint32": "Annotated[int, pydantic.Field(ge=0, le=4294967295)]",
    "uint64": "Annotated[int, pydantic.Field(ge=0, le=18446744073709551615)]",
}

DEFAULT_VALUES = {
    "bool": "False",
    "byte": "0",
    "char": "0",
    "float32": "0",
    "float64": "0",
    "int8": "0",
    "int16": "0",
    "int32": "0",
    "int64": "0",
    "string": '""',
    "wstring": '""',
    "uint8": "0",
    "uint16": "0",
    "uint32": "0",
    "uint64": "0",
}

ARRAY_TYPES = {
    "char": "bytes",
    "byte": "bytes",
    "uint8": "bytes",
}

DEFAULT_ARRAY_VALUES = {
    "char": "bytes()",
    "byte": "bytes()",
    "uint8": "bytes()",
}


class PydanticType:
    type: str
    default_value: str

    def __init__(self, ros_type):
        if ros_type.is_array:
            self._init_array_type(ros_type)
            return

        if ros_type.is_primitive_type():
            self.type = PRIMITIVE_TYPES[ros_type.type]
            self.default_value = DEFAULT_VALUES[ros_type.type]
        else:
            self.type = ros_type.type
            self.default_value = f"{ros_type.type}()"

    def _get_array_type(self, ros_type, elem_type):
        if ros_type.is_upper_bound:
            return f"Annotated[list[{elem_type}], pydantic.Field(max_items={ros_type.array_size})]"
        elif ros_type.array_size:
            return f"Annotated[list[{elem_type}], pydantic.Field(min_items={ros_type.array_size}, max_items={ros_type.array_size}]"
        else:
            return f"list[{elem_type}]"

    def _init_array_type(self, ros_type):
        if ros_type.is_primitive_type():
            if ros_type.type in ARRAY_TYPES:
                self.type = ARRAY_TYPES[ros_type.type]
                self.default_value = DEFAULT_ARRAY_VALUES[ros_type.type]
            else:
                self.type = self._get_array_type(
                    ros_type, PRIMITIVE_TYPES[ros_type.type]
                )
                self.default_value = "[]"
        else:
            self.type = self._get_array_type(ros_type, ros_type.type)
            self.default_value = "[]"


def augment_message(msg: Message):
    for field in msg.spec.fields:
        field.pydantic_type = PydanticType(field.type)
    msg.commented_raw = "".join(  # type: ignore
        map(lambda x: f"# {x}", msg.raw.splitlines(keepends=True))
    )
    return msg


def generate_messages(roslib: RosLibrary, pkg: str, outdir: str):
    template = template_env.get_template("msg.j2")
    pkg_index = roslib.get_package_index(pkg)
    for msg_type in pkg_index.messages:
        msg = roslib.get_message(msg_type)
        outpath = joinp(outdir, msg.pkg, f"{msg.spec.base_type.type}.py")
        print(outpath)
        template.stream(msg=msg).dump(outpath)


def generate_modules(pkgs: Sequence[str], outdir: str):
    roslib = RosLibrary(post_processors=PostProcessors(message=augment_message))
    all_pkg_index = roslib.get_all_interfaces(*pkgs)

    with open(joinp(outdir, "__init__.py"), "w", encoding="utf-8"):
        pass
    for pkg_name, pkg_index in all_pkg_index.items():
        pkg_index: PackageIndex
        pkg_outdir = joinp(outdir, pkg_name)
        makedirs(pkg_outdir, exist_ok=True)
        with open(joinp(pkg_outdir, "__init__.py"), "w", encoding="utf-8") as f:
            for msg in (
                roslib.get_message(msg_type) for msg_type in pkg_index.messages
            ):
                short_type = msg.spec.base_type.type
                f.write(f"from .{short_type} import {short_type}\n")
        generate_messages(roslib, pkg_name, outdir)


def generate(pkgs: Sequence[str], outdir: str):
    print("Generating pydantic interfaces")
    generate_modules(pkgs, outdir)
