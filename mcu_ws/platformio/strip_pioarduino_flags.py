"""
Strip pioarduino platform flags that break the micro-ROS cmake toolchain.

pioarduino 54.x injects Matter/CHIP build flags containing `<angle-bracket>`
header paths (e.g. -DCHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER=<lib/.../foo.h>)
into CCFLAGS. The vendored micro-ROS extra_script.py naively joins these into
a single string for platformio_toolchain.cmake; CMake then treats the angle
brackets as list boundaries and emits literal `;` tokens into flags.make,
which /bin/sh fails to parse ("Syntax error: ';' unexpected"), breaking the
microcdr build.

We monkey-patch `microros_utils.library_builder.CMakeToolchain` at import time
so the filter runs at the exact moment the toolchain file is about to be
written — after the platform has finished populating flags, and before
the problematic string hits disk. Patching here keeps the vendored
micro_ros_platformio submodule fully stock.
"""
import os
import sys

Import("env")


def _filter_flag_string(flag_str):
    """Drop tokens containing shell-unsafe <> characters."""
    return " ".join(tok for tok in flag_str.split() if "<" not in tok and ">" not in tok)


# Add the vendored micro_ros_platformio dir to sys.path so we can import its
# microros_utils package and patch it before the lib's extra_script.py runs.
_lib_root = os.path.normpath(
    os.path.join(env["PROJECT_DIR"], "..", "..", "libs_external", "esp32", "micro_ros_platformio")
)
if _lib_root not in sys.path:
    sys.path.insert(0, _lib_root)

try:
    from microros_utils import library_builder

    _original_init = library_builder.CMakeToolchain.__init__

    def _patched_init(self, path, cc, cxx, ar, cflags, cxxflags):
        _original_init(
            self,
            path,
            cc,
            cxx,
            ar,
            _filter_flag_string(cflags),
            _filter_flag_string(cxxflags),
        )

    library_builder.CMakeToolchain.__init__ = _patched_init
    print("[strip_pioarduino_flags] patched microros_utils.library_builder.CMakeToolchain")
except Exception as e:
    print(f"[strip_pioarduino_flags] patch failed: {e}")
