@echo off
setlocal

set "APP_VERSION=1.0.0"
set "APP=%~dp0VMT_SETO\bin\Debug\net10.0-windows\VMT_SETO_v%APP_VERSION%.exe"

if not exist "%APP%" (
  echo VMT_SETO_v%APP_VERSION%.exe was not found.
  echo Build first:
  echo   dotnet build .\VMT_SETO.slnx
  pause
  exit /b 1
)

echo Starting:
echo   %APP%
start "" "%APP%"
