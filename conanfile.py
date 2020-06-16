# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class Traact(ConanFile):
    name = "traact_install"
    version = "0.0.1"
    
    description = "Meta-Package to gather different libraries in one module. E.g. generic root target for workspace build"
    url = ""
    license = "BSD 3-Clause"
    author = "Frieder Pankratz"

    short_paths = True

    generators = "cmake", "traact_virtualrunenv_generator"
    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"
    options = {
        "shared": [True, False],
        "with_tests": [True, False]
    }

    default_options = {
        "shared": True,
        "with_tests": True
    }

    exports_sources = "CMakeLists.txt", "main.cpp"

    def requirements(self):        
        self.requires("traact_run_env/%s@camposs/stable" % self.version)
        self.requires("traact_core/%s@camposs/stable" % self.version)
        self.requires("traact_facade/%s@camposs/stable" % self.version)
        self.requires("traact_spatial/%s@camposs/stable" % self.version)
        self.requires("traact_vision/%s@camposs/stable" % self.version)
        self.requires("traact_kinect_azure/%s@camposs/stable" % self.version)
        self.requires("traact_point_cloud/%s@camposs/stable" % self.version)


    def configure(self):
        self.options['traact_core'].shared = self.options.shared
        self.options['traact_facade'].shared = self.options.shared
        self.options['traact_spatial'].shared = self.options.shared
        self.options['traact_vision'].shared = self.options.shared





