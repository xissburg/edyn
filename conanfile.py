from conan import ConanFile
from conan.tools.files import save, load
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, get, rm, rmdir
from conan.tools.scm import Version
import os
import re

required_conan_version = ">=1.53.0"


class EdynConan(ConanFile):
    name = "edyn"
    description = "Edyn is a real-time physics engine organized as an ECS"
    license = "MIT"
    url = "https://github.com/conan-io/conan-center-index"
    homepage = "https://github.com/xissburg/edyn"
    topics = ("physics", "game-development", "ecs")
    package_type = "library"
    settings = "os", "arch", "compiler", "build_type"
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
        "gtest/*:no_main": False,
    }

    def set_version(self):
        cmakelists_txt = load(self, os.path.join(self.recipe_folder, "CMakeLists.txt"))
        try:
            self.version = next(re.finditer(r"^project\(Edyn VERSION ([0-9.-]+) LANGUAGES", cmakelists_txt, flags=re.M)).group(1)
        except StopIteration:
            raise ConanException("Could not extract version string from CMakeLists.txt")

    @property
    def _min_cppstd(self):
        return "17"

    @property
    def _compilers_minimum_version(self):
        return {
            "gcc": "9.3", # GCC 9.3 started supporting attributes in constructor arguments
            "clang": "8",
            "apple-clang": "10",
            "Visual Studio": "16",
            "msvc": "192",
        }

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def requirements(self):
        self.requires("entt/3.14.0", transitive_headers=True)
        if self.options.build_tests:
            self.requires("gtest/1.11.0")

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, self._min_cppstd)
        minimum_version = self._compilers_minimum_version.get(str(self.settings.compiler), False)
        if minimum_version and Version(self.settings.compiler.version) < minimum_version:
            raise ConanInvalidConfiguration(
                f"{self.ref} requires C++{self._min_cppstd}, which your compiler does not support."
            )

    def source(self):
        get(self, **self.conan_data["sources"][self.version], strip_root=True)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["EDYN_CONFIG_DOUBLE"] = self.options.floating_type == "double"
        tc.variables["EDYN_INSTALL"] = True
        tc.variables["EDYN_BUILD_EXAMPLES"] = False
        tc.variables["EDYN_BUILD_TESTS"] = False
        tc.variables["CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS"] = True
        tc.variables["EDYN_DISABLE_ASSERT"] = not self.options.enable_assert
        tc.variables["EDYN_ENABLE_SANITIZER"] = self.options.enable_sanitizer
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if self.options.build_tests:
            cmake.test()

    def package(self):
        copy(self, pattern="LICENSE", src=self.source_folder, dst=os.path.join(self.package_folder, "licenses"))
        cmake = CMake(self)
        cmake.install()
        rmdir(self, os.path.join(self.package_folder, "lib", "cmake"))
        rmdir(self, os.path.join(self.package_folder, "share"))
        rm(self, pattern="*.pdb", folder=self.package_folder, recursive=True)

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "Edyn")
        self.cpp_info.set_property("cmake_target_name", "Edyn::Edyn")
        suffix = "_d" if self.settings.build_type == "Debug" else ""
        self.cpp_info.libs = [f"Edyn{suffix}"]
        if self.settings.os in ["Linux", "FreeBSD"]:
            self.cpp_info.system_libs += ["m", "pthread"]
        elif self.settings.os == "Windows":
            self.cpp_info.system_libs = ["winmm"]

        #  TODO: to remove in conan v2 once cmake_find_package_* generators removed
        self.cpp_info.names["cmake_find_package"] = "Edyn"
        self.cpp_info.names["cmake_find_package_multi"] = "Edyn"
