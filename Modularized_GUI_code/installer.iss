#define AppName      "FlightGCS"
#define AppVersion   "1.0.0"
#define AppPublisher "Arjun Veeramony"
#define AppURL       "https://github.com/SoftwareEnthusiastArjun/ESP32_Based_FlightStabilizer"
#define AppExeName   "FlightGCS.exe"
#define BuildDir     "dist\FlightGCS"

[Setup]
AppId={{B7E2D4A1-3F8C-4B92-A5D6-1E9F0C3B7A82}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} v{#AppVersion}
AppPublisher={#AppPublisher}
AppPublisherURL={#AppURL}
AppSupportURL={#AppURL}
AppUpdatesURL={#AppURL}
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
AllowNoIcons=yes
DisableProgramGroupPage=yes
OutputDir=installer_output
OutputBaseFilename=FlightGCS_Setup_v{#AppVersion}
SetupIconFile=icon.ico
UninstallDisplayIcon={app}\{#AppExeName}
UninstallDisplayName={#AppName} v{#AppVersion}
LicenseFile=license.txt
Compression=lzma2/ultra64
SolidCompression=yes
WizardStyle=modern
WizardSizePercent=120
PrivilegesRequired=admin
PrivilegesRequiredOverridesAllowed=dialog
MinVersion=10.0
[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional shortcuts:"; Flags: unchecked
[Files]
Source: "{#BuildDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "README.md"; DestDir: "{app}"; Flags: ignoreversion isreadme
[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"; WorkingDir: "{app}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExeName}"; WorkingDir: "{app}"; Tasks: desktopicon
[Run]
Filename: "{app}\{#AppExeName}"; Description: "Launch {#AppName} now"; Flags: nowait postinstall skipifsilent; WorkingDir: "{app}"
[UninstallDelete]
Type: filesandordirs; Name: "{app}"
[Code]
procedure InitializeWizard();
begin
  WizardForm.WelcomeLabel2.Caption :=
    'Welcome to the ' + ExpandConstant('{#AppName}') + ' Setup Wizard.' +
    'FlightGCS is a Ground Control Station for the ESP32-based ' +
    'Flight Stabilizer project.' +
    'Features:' + #13#10 +
    '  - Real-time orientation HUD' + #13#10 +
    '  - RC input monitoring' + #13#10 +
    '  - PID tuning (Pitch and Roll)' + #13#10 +
    '  - WiFi credential management via Serial Monitor' + #13#10 +
    '  - Auto-discovery of ESP32 on local network' +
    'Click Next to read the License Agreement.';
end;
