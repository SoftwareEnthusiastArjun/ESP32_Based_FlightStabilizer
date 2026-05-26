#define AppName      "GCSHelper"
#define AppVersion   "1.0.0"
#define AppPublisher "Arjun Veeramony"
#define AppURL       "https://github.com/SoftwareEnthusiastArjun/ESP32_Based_FlightStabilizer"
#define AppExeName   "GCSHelper.exe"
#define BuildDir     "dist\GCSHelper"

[Setup]
AppId={{C9D3E5F7-A1B2-4C8D-9E0F-123456789ABC}
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
OutputBaseFilename=GCSHelper_Setup_v{#AppVersion}
SetupIconFile=icon.ico
UninstallDisplayIcon={app}\{#AppExeName}
UninstallDisplayName={#AppName} v{#AppVersion}
LicenseFile=LICENSE.txt
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
    'Welcome to GCSHelper v1.0.0' + #13#10 + #13#10 +
    'GCSHelper is the setup and configuration utility for the ' +
    'ESP32-based Flight Stabilizer — developed as an M.Sc. ' +
    'thesis project at FH Dortmund by Arjun Veeramony.' + #13#10 + #13#10 +
    'What it does:' + #13#10 +
    '  - Configure ESP32 WiFi credentials over USB serial' + #13#10 +
    '  - Flash firmware .bin files directly to the ESP32' + #13#10 +
    '  - Monitor serial output for debugging' + #13#10 + #13#10 +
    'Click Next to read the License Agreement.';
end;
