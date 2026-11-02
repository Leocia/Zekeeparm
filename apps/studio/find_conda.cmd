@echo off
if defined STUDIO_CONDA if exist "%STUDIO_CONDA%" exit /b 0
set "STUDIO_CONDA="
if defined CONDA_EXE if exist "%CONDA_EXE%" set "STUDIO_CONDA=%CONDA_EXE%"
for %%C in (conda.bat conda.exe) do (
  for /f "delims=" %%P in ('where %%C 2^>nul') do (
    if not defined STUDIO_CONDA set "STUDIO_CONDA=%%P"
  )
)
if not defined STUDIO_CONDA (
  for %%D in ("%USERPROFILE%\miniforge3" "%LOCALAPPDATA%\miniforge3" "%USERPROFILE%\Miniconda3" "%LOCALAPPDATA%\miniconda3" "%ProgramData%\miniforge3" "%ProgramData%\Miniconda3") do (
    if exist "%%~D\condabin\conda.bat" if not defined STUDIO_CONDA set "STUDIO_CONDA=%%~D\condabin\conda.bat"
  )
)
if not defined STUDIO_CONDA (
  echo Miniforge was not found. Install Miniforge for Windows x64, then retry.
  echo See README_Windows.md in the extracted package.
  exit /b 1
)
exit /b 0
