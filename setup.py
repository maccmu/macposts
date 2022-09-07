"""Setup module based on setuptools.

It uses CMake to build an extension, but there are also normal Python files in
the package, making it a bit more complicated than usual.

Currently the Python package tooling is changing. This project now does not
closely follow the trend because it seems that the other tools and practices are
not very mature and we want to preserve a bit of backward compatibility.
However, we may need to and would like to do so in the future.

Ref:
- https://github.com/pypa/sampleproject
- https://github.com/pybind/cmake_example

"""

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from pathlib import Path
import os
import sys
from subprocess import check_call

here = Path(__file__).parent.resolve()


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = Path(sourcedir).resolve()


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        # Configuration
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()
        debug = (int(os.environ.get("DEBUG", "0"))
                 if self.debug is None
                 else self.debug)
        cfg = "Debug" if debug else "Release"

        # CMake arguments
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
            "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DCMAKE_BUILD_TYPE={}".format(cfg)
        ]
        cmake_args_env = os.environ.get("CMAKE_ARGS")
        if cmake_args_env is not None:
            cmake_args += [arg for arg in cmake_args_env.split(" ") if arg]

        # Build arguments
        build_args = []
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += ["-j{}".format(self.parallel)]

        # Build extension
        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True, exist_ok=False)
        check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        check_call(["cmake", "--build", "."] + build_args, cwd=build_temp)

setup(
    ext_modules=[CMakeExtension("_macposts_ext")],
    cmdclass={"build_ext": CMakeBuild}
)
