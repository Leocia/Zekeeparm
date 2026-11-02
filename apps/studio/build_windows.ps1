param([switch]$Installer)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

function Assert-ExitCode {
    if ($LASTEXITCODE -ne 0) { throw "Build command failed with exit code $LASTEXITCODE" }
}

$condaCommand = 'conda'
if ($env:STUDIO_CONDA -and (Test-Path $env:STUDIO_CONDA)) {
    $condaCommand = $env:STUDIO_CONDA
} elseif (-not (Get-Command conda -ErrorAction SilentlyContinue)) {
    throw 'Install Miniforge and run this script from a terminal with conda available.'
}

& $condaCommand env update --file environment.yml
Assert-ExitCode
if (-not (Test-Path 'public\vendor\blockly\blockly_compressed.js')) {
    if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
        throw 'Missing Blockly assets. Extract the Windows source package or prepare assets with Node.js.'
    }
    npm.cmd ci
    Assert-ExitCode
}
& $condaCommand run --no-capture-output -n zekeep-studio python -m unittest discover -s tests -v
Assert-ExitCode
& $condaCommand run --no-capture-output -n zekeep-studio python build_bundle.py
Assert-ExitCode
& '.\dist\ZekeeparmStudio\ZekeeparmStudio.exe' --self-test
Assert-ExitCode

if ($Installer) {
    $compiler = Get-Command ISCC.exe -ErrorAction SilentlyContinue
    if (-not $compiler) { throw 'Install Inno Setup 6 and add ISCC.exe to PATH for -Installer.' }
    $env:STUDIO_VERSION = (Get-Content package.json -Raw | ConvertFrom-Json).version
    & $compiler.Source installer.iss
    Assert-ExitCode
}
Write-Host 'Bundle: dist\ZekeeparmStudio\ZekeeparmStudio.exe'
Write-Host 'Validate the bundle on a clean Windows computer before hardware use or distribution.'
