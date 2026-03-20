"""Build the veos_cosim pybind11 extension (Windows x64 + VeosCoSimAppl)."""

from __future__ import annotations

import os
from pathlib import Path

from setuptools import setup

from pybind11.setup_helpers import Pybind11Extension, build_ext

ROOT = Path(__file__).resolve().parent
PLATFORM = os.environ.get("VEOSCOSIM_PLATFORM", "x64")
CONFIG = os.environ.get("VEOSCOSIM_CONFIG", "Debug")

CLIENT_DIR = ROOT.parent / "VeosCoSim_Client" / "client" / PLATFORM / CONFIG
INC_DIR = CLIENT_DIR / "include"
LIB_DIR = CLIENT_DIR / "lib"

if not INC_DIR.is_dir():
    raise RuntimeError(
        f"VeosCoSim include dir not found: {INC_DIR}. "
        "Set VEOSCOSIM_PLATFORM / VEOSCOSIM_CONFIG."
    )

if not LIB_DIR.is_dir():
    raise RuntimeError(
        f"VeosCoSim library dir not found: {LIB_DIR}. "
        "Set VEOSCOSIM_PLATFORM / VEOSCOSIM_CONFIG."
    )

ext_modules = [
    Pybind11Extension(
        "veos_cosim._veos_cosim",
        sources=["src/veos_cosim_binding.cpp"],
        include_dirs=[str(INC_DIR)],
        library_dirs=[str(LIB_DIR)],
        libraries=["VeosCoSimAppl"],
        cxx_std=17,
        define_macros=[("VEOSCOSIM_IMPORT", None)],
    ),
]

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
