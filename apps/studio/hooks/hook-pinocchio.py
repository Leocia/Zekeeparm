"""Collect native Conda dependencies outside the Python package directory."""
from PyInstaller.compat import is_pure_conda
from PyInstaller.utils.hooks import collect_dynamic_libs

binaries = collect_dynamic_libs("pinocchio")
if is_pure_conda:
    from PyInstaller.utils.hooks import conda_support
    binaries += conda_support.collect_dynamic_libs("pinocchio", dependencies=True)
