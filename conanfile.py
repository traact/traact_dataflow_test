# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class Traact(ConanFile):
    name = "traact_dataflow_test"
    version = "0.1.0"

    description = "Based on Meta-Package. Runtime tests for dataflow engine"
    url = ""
    license = "MIT"
    author = "Frieder Pankratz"

    short_paths = True

    generators = "cmake", "TraactVirtualRunEnvGenerator"
    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"
    options = {
        "shared": [True, False],
    }

    default_options = {
        "shared": True,
    }

    exports_sources = "CMakeLists.txt", "src/", "tests/"

    def requirements(self):
        self.requires("traact_run_env/[>=1.0.0]@camposs/stable")
        self.requires("traact_core/[>=0.1.0]@camposs/stable")
        self.requires("traact_spatial/[>=0.1.0]@camposs/stable")
        self.requires("dataflow_use_case_test/0.0.1@camposs/stable")
        self.requires("gtest/[>=1.10.0]")
        self.requires("benchmark/[>=1.6.1]")

    def configure(self):
        self.options['traact_core'].shared = self.options.shared
        self.options['traact_spatial'].shared = self.options.shared
