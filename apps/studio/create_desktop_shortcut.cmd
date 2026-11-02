@echo off
setlocal DisableDelayedExpansion
set "STUDIO_LAUNCHER=%~dp0start_windows.cmd"
call "%~dp0find_conda.cmd" >nul 2>&1
powershell.exe -NoProfile -STA -Command ^
 "$ErrorActionPreference='Stop';" ^
 "$conda=$env:STUDIO_CONDA;" ^
 "if (-not $conda) { Add-Type -AssemblyName System.Windows.Forms; $picker=New-Object System.Windows.Forms.OpenFileDialog; $picker.Title='Select Miniforge Scripts\conda.exe or condabin\conda.bat'; $picker.Filter='Conda (conda.exe or conda.bat)|conda.exe;conda.bat'; $picker.InitialDirectory=$env:USERPROFILE; if ($picker.ShowDialog() -ne 'OK') { exit 1 }; $conda=$picker.FileName };" ^
 "if (-not (Test-Path -LiteralPath $conda -PathType Leaf)) { throw 'Conda file was not found.' };" ^
 "if ([IO.Path]::GetFileName($conda) -notin @('conda.exe','conda.bat')) { throw 'Select conda.exe or conda.bat.' };" ^
 "$shell=New-Object -ComObject WScript.Shell; $desktop=[Environment]::GetFolderPath('Desktop');" ^
 "$link=$shell.CreateShortcut((Join-Path $desktop 'Zekeeparm Studio.lnk'));" ^
 "$link.TargetPath=$env:STUDIO_LAUNCHER; $link.Arguments=([string][char]34)+$conda+([char]34);" ^
 "$link.WorkingDirectory=Split-Path -LiteralPath $env:STUDIO_LAUNCHER; $link.WindowStyle=7;" ^
 "$link.IconLocation=(Join-Path $env:SystemRoot 'System32\shell32.dll')+',14'; $link.Description='Open Zekeeparm Studio'; $link.Save();" ^
 "Write-Host ('Desktop shortcut created: '+(Join-Path $desktop 'Zekeeparm Studio.lnk'))"
if errorlevel 1 goto failed
if /i not "%~1"=="--no-pause" pause
exit /b 0
:failed
echo Shortcut was not created. Select the Conda file from your Miniforge installation and retry.
if /i not "%~1"=="--no-pause" pause
exit /b 1
