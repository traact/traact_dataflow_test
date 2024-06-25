# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "traact_dataflow_test"
    description = "Based on Meta-Package. Runtime tests for dataflow engine"
    url = "https://github.com/traact/traact_dataflow_test.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "CMakeLists.txt", "src/", "tests/"

    def requirements(self):
        self.traact_requires("traact_run_env", "latest")
        self.traact_requires("traact_core", "latest")
        self.traact_requires("traact_spatial", "latest")
        self.traact_requires("dataflow_use_case_test", "latest")
        self.requires("gtest/1.14.0")
        self.requires("benchmark/[>=1.6.1]")
