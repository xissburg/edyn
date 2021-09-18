from conans import CMake, ConanFile, tools


class TestPackage(ConanFile):
    settings = "os", "arch", "compiler", "build_type"
    generators = "cmake", "cmake_find_package"

    default_options = {
        "gtest:no_main": False,
    }

    def requirements(self):
        self.requires("gtest/1.11.0")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        if not tools.cross_building(self):
            cmake = CMake(self)
            cmake.test()
