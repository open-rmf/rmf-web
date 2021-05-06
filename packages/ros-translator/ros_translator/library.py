import os.path
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, cast

import ament_index_python
from rosidl_adapter.parser import (
    MessageSpecification,
    ServiceSpecification,
    parse_message_file,
    parse_service_file,
)


@dataclass
class Interface:
    pkg: str
    file: str
    base_dir: str
    rel_dir: str
    full_type: str
    full_type_with_path: str
    raw: str


@dataclass
class Message(Interface):
    spec: MessageSpecification
    dependent_types: List["Message"]


@dataclass
class Service(Interface):
    spec: ServiceSpecification


@dataclass
class PackageIndex:
    pkg_share_dir: str
    messages: Dict[str, str]  # dict of message type and the definition file
    services: Dict[str, str]  # dict of service type and the definition file


@dataclass
class PostProcessors:
    message: Optional[Callable[[Message], Any]] = None
    service: Optional[Callable[[Service], Any]] = None


class RosLibrary:
    def __init__(self, post_processors: Optional[PostProcessors] = None):
        self.post_processors = post_processors or PostProcessors()
        self._messages = cast(Dict[str, Message], {})
        self._services = cast(Dict[str, Service], {})
        self._package_index = cast(Dict[str, PackageIndex], {})

    def get_message(self, full_msg_type: str) -> Message:
        """
        :param full_msg_type: The full message type .e.g "std_msgs/String"
        """
        if full_msg_type not in self._messages:
            self._messages[full_msg_type] = self._parse_message(full_msg_type)
        return self._messages[full_msg_type]

    def get_all_interfaces(self, *pkgs: Iterable[str]) -> Dict[str, PackageIndex]:
        """
        Recursively gets the package index for packages and their dependencies.
        """
        all_pkgs = self.get_all_dependent_packages(*pkgs)
        return {p: self.get_package_index(p) for p in all_pkgs}

    def get_service(self, full_srv_type: str) -> Service:
        """
        :param full_srv_type: The full service type
        """
        if full_srv_type not in self._services:
            self._services[full_srv_type] = self._parse_service(full_srv_type)
        return self._services[full_srv_type]

    def get_package_index(self, pkg_name: str) -> PackageIndex:
        if pkg_name not in self._package_index:
            self._package_index[pkg_name] = self._parse_rosidl_index(pkg_name)
        return self._package_index[pkg_name]

    def get_all_dependent_packages(self, *pkgs: Iterable[str]) -> List[str]:
        def recur(pkg_name_):
            if pkg_name_ not in self._package_index:
                self._package_index[pkg_name_] = self._parse_rosidl_index(pkg_name_)
            pkg_index = self._package_index[pkg_name_]
            msgs = [self.get_message(m) for m in pkg_index.messages]
            dep_msgs = []
            for m in msgs:
                dep_msgs.extend([dm for dm in m.dependent_types if dm.pkg != pkg_name_])
            dep_pkgs = {}
            for m in dep_msgs:
                for dp in recur(m.pkg):
                    dep_pkgs[dp] = dp
                dep_pkgs[m.pkg] = m.pkg
            return dep_pkgs

        all_dep_pkgs = {}
        for p in pkgs:
            for dp in recur(p):
                all_dep_pkgs[dp] = dp
            all_dep_pkgs[p] = p
        return list(all_dep_pkgs.keys())

    @staticmethod
    def _parse_rosidl_index(package: str):
        base_dir = ament_index_python.get_package_share_directory(package)
        messages = {}
        services = {}
        interfaces = ament_index_python.get_resource("rosidl_interfaces", package)[
            0
        ].split("\n")
        for i in interfaces:
            if i.endswith(".msg"):
                short_type = i[i.rindex("/") + 1 : i.rindex(".")]
                full_type = f"{package}/{short_type}"
                messages[full_type] = f"{base_dir}/{i}"
            if i.endswith(".srv"):
                short_type = i[i.rindex("/") + 1 : i.rindex(".")]
                full_type = f"{package}/{short_type}"
                services[full_type] = f"{base_dir}/{i}"
        return PackageIndex(
            pkg_share_dir=base_dir,
            messages=messages,
            services=services,
        )

    @staticmethod
    def _get_full_type_with_path(
        package: str, file: str, base_dir: dir, short_type: str
    ):
        rel_path = os.path.relpath(file, base_dir)
        return f"{package}/{rel_path[:rel_path.rindex('/')]}/{short_type}"

    @staticmethod
    def _get_rel_dir(full_type_with_path: str):
        return f"{full_type_with_path[full_type_with_path.index('/')+1:full_type_with_path.rindex('/')]}"

    def _get_msg_dependent_types(self, msgspec: MessageSpecification):
        dependent_types = [
            f"{f.type.pkg_name}/{f.type.type}"
            for f in msgspec.fields
            if not f.type.is_primitive_type()
        ]
        dedup = {f: f for f in dependent_types}
        return [self.get_message(m) for m in dedup.keys()]

    def _parse_message(self, full_msg_type: str):
        parts = full_msg_type.split("/")
        if len(parts) != 2:
            raise RuntimeError("message type must be in the form '<package>/<type>'")
        package = parts[0]

        if package not in self._package_index:
            self._package_index[package] = self._parse_rosidl_index(package)

        file = self._package_index[package].messages[full_msg_type]
        base_dir = self._package_index[package].pkg_share_dir
        spec = parse_message_file(package, file)
        full_type_with_path = self._get_full_type_with_path(
            package,
            file,
            base_dir,
            spec.base_type.type,
        )
        with open(file, "r") as f:
            raw = f.read()
        message = Message(
            pkg=package,
            file=file,
            base_dir=base_dir,
            rel_dir=self._get_rel_dir(full_type_with_path),
            full_type=full_msg_type,
            full_type_with_path=full_type_with_path,
            raw=raw,
            spec=spec,
            dependent_types=self._get_msg_dependent_types(spec),
        )
        if self.post_processors.message:
            message = self.post_processors.message(message)
        return message

    def _parse_service(self, full_srv_type: str):
        parts = full_srv_type.split("/")
        if len(parts) != 2:
            raise RuntimeError("service type must be in the form '<package>/<type>'")
        package = parts[0]

        if package not in self._package_index:
            self._package_index[package] = self._parse_rosidl_index(package)

        file = self._package_index[package].services[full_srv_type]
        base_dir = self._package_index[package].pkg_share_dir
        spec = parse_service_file(package, file)
        full_type_with_path = self._get_full_type_with_path(
            package,
            file,
            base_dir,
            spec.srv_name,
        )
        with open(file, "r") as f:
            raw = f.read()
        service = Service(
            pkg=package,
            file=file,
            base_dir=base_dir,
            rel_dir=self._get_rel_dir(full_type_with_path),
            full_type=full_srv_type,
            full_type_with_path=full_type_with_path,
            spec=spec,
            raw=raw,
        )
        if self.post_processors.service:
            service = self.post_processors.service(service)
        return service
