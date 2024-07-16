from dataclasses import dataclass, field
from typing import Any, Callable, Generator, Generic, Sequence, TypeVar

import ament_index_python
from rosidl_parser.definition import (
    AbstractNestedType,
    AbstractType,
    Constant,
    IdlLocator,
)
from rosidl_parser.definition import Member as IdlMember
from rosidl_parser.definition import Message as IdlMessage
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service as IdlService
from rosidl_parser.parser import parse_idl_file

TypeT = TypeVar("TypeT")


@dataclass
class Interface:
    name: str
    full_type_name: str
    pkg: str


@dataclass
class Member(Generic[TypeT]):
    name: str
    type: TypeT
    idl: IdlMember


@dataclass
class Message:
    name: str
    full_type_name: str
    pkg: str
    members: list[Member]
    constants: list[Constant]
    dependencies: set["Message"]
    idl: IdlMessage

    def __eq__(self, other):
        return type(other) is Message and self.full_type_name == other.full_type_name

    def __hash__(self):
        return hash(self.full_type_name)

    def package_dependencies(self) -> set[str]:
        deps: set[str] = set()
        for d in self.dependencies:
            deps.add(str(d.idl.structure.namespaced_type.namespaced_name()[0]))
        return deps


@dataclass
class Service:
    name: str
    full_type_name: str
    pkg: str
    request: Message
    response: Message
    dependencies: set[Message]
    idl: IdlService

    def __eq__(self, other):
        return type(other) is Service and self.full_type_name == other.full_type_name

    def __hash__(self):
        return hash(self.full_type_name)


@dataclass
class Namespace:
    name: str
    full_name: str
    namespaces: dict[str, "Namespace"] = field(default_factory=lambda: {})
    messages: dict[str, IdlMessage] = field(default_factory=lambda: {})
    services: dict[str, IdlService] = field(default_factory=lambda: {})

    def get_namespace(self, namespaces: Sequence[str]):
        if namespaces[0] != self.name:
            raise ValueError(f"{namespaces} is not in {self.name}")

        ns = self
        full_name = self.full_name
        for n in namespaces[1:]:
            if n not in ns.namespaces:
                full_name += f"/{n}"
                ns.namespaces[n] = Namespace(name=n, full_name=full_name)
            ns = ns.namespaces[n]
        return ns

    def get_message(self, namespaced_type: Sequence[str]) -> IdlMessage:
        name = namespaced_type[-1]
        namespaces = namespaced_type[:-1]
        ns = self.get_namespace(namespaces)
        return ns.messages[name]

    def all_messages(self) -> Generator[tuple[str, IdlMessage], Any, Any]:
        for k, v in self.messages.items():
            yield f"{self.full_name}/{k}", v
        for ns in self.namespaces.values():
            yield from ns.all_messages()

    def add_message(self, msg: IdlMessage):
        namespaces = tuple(msg.structure.namespaced_type.namespaces)
        name = msg.structure.namespaced_type.name
        self.get_namespace(namespaces).messages[name] = msg

    def get_service(self, namespaced_type: Sequence[str]) -> IdlService:
        name = namespaced_type[-1]
        namespaces = namespaced_type[:-1]
        ns = self.get_namespace(namespaces)
        return ns.services[name]

    def all_services(self) -> Generator[tuple[str, IdlService], Any, Any]:
        for k, v in self.services.items():
            yield f"{self.full_name}/{k}", v
        for ns in self.namespaces.values():
            yield from ns.all_services()

    def add_service(self, srv: IdlService):
        namespaces = tuple(srv.namespaced_type.namespaces)
        name = srv.namespaced_type.name
        self.get_namespace(namespaces).services[name] = srv


@dataclass
class PackageIndex:
    pkg_name: str
    pkg_share_dir: str
    root_ns: Namespace


class RosLibrary(Generic[TypeT]):
    def __init__(self, type_processor: Callable[[AbstractType], TypeT]):
        self.type_processor = type_processor
        self._messages: dict[str, Message] = {}
        self._services: dict[str, Service] = {}
        self._package_index: dict[str, PackageIndex] = {}

    def get_all_interfaces(self, *pkgs: str) -> list[PackageIndex]:
        """
        Recursively gets the package index for packages and their dependencies.
        """
        all_pkgs = self.get_all_package_dependencies(*pkgs).union(pkgs)
        return [self.get_package_index(p) for p in all_pkgs]

    def get_message(self, full_type_name: str) -> Message:
        """
        :param namespaced_name: A tuple of strings containing the full name, e.g. ("std_msgs", "msgs", "Bool")
        """
        if full_type_name not in self._messages:
            namespaced_type = full_type_name.split("/")
            pkg_index = self.get_package_index(namespaced_type[0])
            self._messages[full_type_name] = self._parse_idl_message(
                pkg_index.root_ns.get_message(namespaced_type)
            )
        return self._messages[full_type_name]

    def get_service(self, full_type_name: str) -> Service:
        """
        :param namespaced_name: A tuple of strings containing the full name, e.g. ("my_msgs", "srv", "MyService")
        """
        if full_type_name not in self._services:
            namespaced_type = full_type_name.split("/")
            pkg_index = self.get_package_index(namespaced_type[0])
            self._services[full_type_name] = self._parse_idl_service(
                pkg_index.root_ns.get_service(namespaced_type)
            )
        return self._services[full_type_name]

    def get_package_index(self, pkg_name: str) -> PackageIndex:
        if pkg_name not in self._package_index:
            self._package_index[pkg_name] = self._parse_rosidl_index(pkg_name)
        return self._package_index[pkg_name]

    def get_all_package_dependencies(self, *pkgs: str) -> set[str]:
        dep_pkgs = set(pkgs)
        for p in pkgs:
            if p not in self._package_index:
                self._package_index[p] = self._parse_rosidl_index(p)
            pkg_index = self._package_index[p]
            for n, _ in pkg_index.root_ns.all_messages():
                msg = self.get_message(n)
                dep_pkgs.update(m.pkg for m in msg.dependencies)
            for n, _ in pkg_index.root_ns.all_services():
                srv = self.get_service(n)
                dep_pkgs.update(m.pkg for m in srv.dependencies)
        dep_pkgs.difference_update(pkgs)
        if len(dep_pkgs) != 0:
            return dep_pkgs.union(self.get_all_package_dependencies(*dep_pkgs))
        else:
            return dep_pkgs

    @staticmethod
    def get_full_type_name(t: NamespacedType) -> str:
        return "/".join(t.namespaced_name())

    def _get_msg_dependencies(self, idl: IdlMessage) -> set[Message]:
        depends: set[Message] = set()
        for m in idl.structure.members:
            m: IdlMember
            if isinstance(m.type, NamespacedType):
                namespaced_name = self.get_full_type_name(m.type)
                depends.add(self.get_message(namespaced_name))
            elif isinstance(m.type, AbstractNestedType) and isinstance(
                m.type.value_type, NamespacedType
            ):
                namespaced_name = self.get_full_type_name(m.type.value_type)
                depends.add(self.get_message(namespaced_name))
        return depends

    def _parse_idl_message(self, idl: IdlMessage) -> Message:
        namespaced_name = "/".join(idl.structure.namespaced_type.namespaced_name())
        members: list[Member] = []
        for m in idl.structure.members:
            m: IdlMember
            members.append(Member(name=m.name, type=self.type_processor(m.type), idl=m))
        return Message(
            name=idl.structure.namespaced_type.name,
            full_type_name=namespaced_name,
            pkg=str(idl.structure.namespaced_type.namespaced_name()[0]),
            members=members,
            constants=idl.constants,
            dependencies=self._get_msg_dependencies(idl),
            idl=idl,
        )

    def _parse_idl_service(self, idl: IdlService) -> Service:
        namespaced_name = self.get_full_type_name(idl.namespaced_type)
        req_type_name = self.get_full_type_name(
            idl.request_message.structure.namespaced_type
        )
        resp_type_name = self.get_full_type_name(
            idl.response_message.structure.namespaced_type
        )
        dependencies = set(
            (
                *self._get_msg_dependencies(idl.request_message),
                *self._get_msg_dependencies(idl.response_message),
            )
        )
        return Service(
            name=idl.namespaced_type.name,
            full_type_name=namespaced_name,
            pkg=str(idl.namespaced_type.namespaced_name()[0]),
            request=self.get_message(req_type_name),
            response=self.get_message(resp_type_name),
            dependencies=dependencies,
            idl=idl,
        )

    def _parse_rosidl_index(self, package: str):
        base_dir = ament_index_python.get_package_share_directory(package)
        interface_files = ament_index_python.get_resource("rosidl_interfaces", package)[
            0
        ].split("\n")
        idl_files = [x for x in interface_files if x.endswith(".idl")]
        idls = [parse_idl_file(IdlLocator(base_dir, x)).content for x in idl_files]
        idl_elems = [e for i in idls for e in i.elements]
        root_ns = Namespace(name=package, full_name=package)
        for e in idl_elems:
            if isinstance(e, IdlMessage):
                root_ns.add_message(e)
            elif isinstance(e, IdlService):
                root_ns.add_service(e)
                root_ns.add_message(e.request_message)
                root_ns.add_message(e.response_message)
        return PackageIndex(
            pkg_name=package,
            pkg_share_dir=base_dir,
            root_ns=root_ns,
        )
