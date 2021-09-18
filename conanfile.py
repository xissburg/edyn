from conans import ConanFile, tools, CMake
from conans.errors import ConanException
import os
import re
import shutil
import textwrap


class EdynConan(ConanFile):
    name = "edyn"
    def set_version(self):
        cmakelists_txt = tools.load(os.path.join(self.recipe_folder, "CMakeLists.txt"))
        try:
            self.version = next(re.finditer(r"^project\(Edyn VERSION ([0-9.-]+) LANGUAGES", cmakelists_txt, flags=re.M)).group(1)
        except StopIteration:
            raise ConanException("Could not extract version string from CMakeLists.txt")

    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "floating_type": ["float", "double"],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "floating_type": "float",
    }
    settings = "os", "arch", "compiler", "build_type"

    generators = "cmake", "cmake_find_package"
    no_copy_source = True

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            del self.options.fPIC

    def requirements(self):
        self.requires("entt/3.8.0")

    def export_sources(self):
        self.copy("AUTHORS")
        self.copy("LICENSE")
        self.copy("CMakeLists.txt")
        for folder in ("cmake", "docs", "include", "src"):
            shutil.copytree(folder, os.path.join(self.export_sources_folder , folder))

    def build(self):
        def no_backslashes(path: str) -> str:
            return path.replace("\\", "/")
        # Create CMake wrapper to configure the toolchain (Conan 2.0 won't need this anymore)
        tools.save("CMakeLists.txt", textwrap.dedent(f"""\
            cmake_minimum_required(VERSION 3.0)
            project(cmake_wrapper)
            include("{no_backslashes(self.build_folder)}/conanbuildinfo.cmake")
            conan_basic_setup(TARGETS)

            add_subdirectory("{no_backslashes(self.source_folder)}" edyn)
        """))
        cmake = CMake(self)
        cmake.definitions["EDYN_BUILD_EXAMPLES"] = False
        cmake.definitions["EDYN_BUILD_TESTS"] = False
        cmake.definitions["EDYN_INSTALL"] = True
        cmake.definitions["EDYN_CONFIG_DOUBLE"] = self.options.floating_type == "double"
        cmake.definitions["EDYN_DISABLE_ASSERT"] = True
        cmake.configure(source_folder=self.build_folder)
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        libsuffix = "_d" if self.settings.build_type == "Debug" else ""
        self.cpp_info.libs = ["Edyn" + libsuffix]
        if self.settings.os == "Linux":
            self.cpp_info.system_libs = ["dl", "pthread"]
        elif self.settings.os == "Windows":
            self.cpp_info.system_libs = ["winmm"]

        self.cpp_info.names["cmake_find_package"] = "Edyn"
        self.cpp_info.names["cmake_find_package_multi"] = "Edyn"
