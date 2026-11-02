@echo off
setlocal
cd /d "%~dp0"
call "%~dp0find_conda.cmd"
if errorlevel 1 goto failed
if not exist "public\vendor\blockly\blockly_compressed.js" (
  echo Missing Blockly assets. Extract the complete Windows source package.
  goto failed
)
call "%STUDIO_CONDA%" env update --file environment.yml
if errorlevel 1 goto failed
call "%STUDIO_CONDA%" run --no-capture-output -n zekeep-studio python studio.py --self-test
if errorlevel 1 goto failed
call "%~dp0create_desktop_shortcut.cmd" --no-pause
if errorlevel 1 echo You can retry 4-create-desktop-shortcut.cmd after setup.
echo Setup complete. Use the desktop shortcut or 2-start.cmd to open Studio.
pause
exit /b 0
:failed
echo Setup failed. Keep this window open and check the error above.
pause
exit /b 1
