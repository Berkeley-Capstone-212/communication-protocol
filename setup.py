"""Builds the C extension; package metadata is in pyproject.toml."""
import sys
import sysconfig

from setuptools import Extension, setup

# The stable ABI gives one wheel per platform for every CPython >= 3.10.
# Free-threaded builds do not support it, so they get a regular extension.
LIMITED = not sysconfig.get_config_var("Py_GIL_DISABLED")

setup(
    ext_modules=[
        Extension(
            "swarm_protocol._swarm",
            sources=["src/swarm.c", "python/swarm_protocol/_swarm.c"],
            include_dirs=["include"],
            depends=["include/swarm.h"],
            define_macros=[("Py_LIMITED_API", "0x030A0000")] if LIMITED else [],
            libraries=[] if sys.platform == "win32" else ["m"],
            py_limited_api=LIMITED,
        )
    ],
    options={"bdist_wheel": {"py_limited_api": "cp310"}} if LIMITED else {},
)
