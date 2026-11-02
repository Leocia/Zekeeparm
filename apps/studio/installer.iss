#define AppVersion GetEnv('STUDIO_VERSION')
[Setup]
AppId={{D8D8CA35-E9D2-46BE-A390-93231B229E19}
AppName=Zekeeparm Studio
AppVersion={#AppVersion}
DefaultDirName={localappdata}\Programs\ZekeeparmStudio
DefaultGroupName=Zekeeparm Studio
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir=dist
OutputBaseFilename=ZekeeparmStudio-Setup
Compression=lzma2
SolidCompression=yes
CloseApplications=no
UninstallDisplayIcon={app}\ZekeeparmStudio.exe

[Files]
Source: "dist\ZekeeparmStudio\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\Zekeeparm Studio"; Filename: "{app}\ZekeeparmStudio.exe"
Name: "{userdesktop}\Zekeeparm Studio"; Filename: "{app}\ZekeeparmStudio.exe"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Run]
Filename: "{app}\ZekeeparmStudio.exe"; Description: "Open Zekeeparm Studio"; Flags: nowait postinstall skipifsilent
