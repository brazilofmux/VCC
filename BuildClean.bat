@echo off
:: Clean Build VCC 

set VCC_CONFIG=Release
if exist "%VSINSTALLDIR%\MSBuild\Current\Bin\MSBuild.exe" (
  msbuild vcc.sln /t:Clean /p:Configuration=%VCC_CONFIG% /p:Platform=x86
  call Build.bat
) else (
  echo Build must be run from a Developer Command Prompt for Visual Studio
)
