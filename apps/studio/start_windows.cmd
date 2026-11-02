@echo off
setlocal DisableDelayedExpansion
if not "%~1"=="" set "STUDIO_CONDA=%~1"
cd /d "%~dp0"
call "%~dp0find_conda.cmd"
if errorlevel 1 (
  pause
  exit /b 1
)
call "%STUDIO_CONDA%" run --no-capture-output -n zekeep-studio python studio.py
if errorlevel 1 pause
