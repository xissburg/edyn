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
        "enable_assert": [True, False],
        "enable_sanitizer": [True, False],
        "floating_type": ["float", "double"],
        "build_tests": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "enable_assert": False,
        "enable_sanitizer": False,
        "floating_type": "float",
        "build_tests": False,
        "gtest:no_main": False,
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
        if self.options.build_tests:
            self.requires("gtest/1.11.0", private=True)

    def export_sources(self):
        self.copy("AUTHORS")
        self.copy("LICENSE")
        self.copy("CMakeLists.txt")
        for folder in ("cmake", "docs", "examples", "include", "src", "test"):
            shutil.copytree(folder, os.path.join(self.export_sources_folder , folder))

    def package_id(self):
        del self.info.options.build_tests

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
        cmake.verbose = True
        cmake.definitions["EDYN_BUILD_EXAMPLES"] = self.options.build_tests
        cmake.definitions["EDYN_BUILD_TESTS"] = self.options.build_tests
        cmake.definitions["EDYN_INSTALL"] = True
        cmake.definitions["EDYN_CONFIG_DOUBLE"] = self.options.floating_type == "double"
        cmake.definitions["EDYN_DISABLE_ASSERT"] = not self.options.enable_assert
        cmake.definitions["EDYN_ENABLE_SANITIZER"] = self.options.enable_sanitizer
        cmake.configure(source_folder=self.build_folder)
        cmake.build()
        if self.options.build_tests:
            cmake.test()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        libsuffix = "_d" if self.settings.build_type == "Debug" else ""
        self.cpp_info.libs = ["Edyn" + libsuffix]

        if self.settings.os == "Linux":
            self.cpp_info.system_libs = ["pthread"]
        elif self.settings.os == "Windows":
            self.cpp_info.system_libs = ["winmm"]

        self.cpp_info.names["cmake_find_package"] = "Edyn"
        self.cpp_info.names["cmake_find_package_multi"] = "Edyn"
