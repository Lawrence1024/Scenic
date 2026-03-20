@echo off
setlocal

set SCRIPT_DIR=%~dp0
set BUILD_DIR=%SCRIPT_DIR%bridge\build

echo SCRIPT_DIR=%SCRIPT_DIR%

where cmake >nul 2>nul
if errorlevel 1 (
  echo ERROR: cmake is not on PATH.
  exit /b 1
)

echo [1/2] Configuring bridge...
cmake -S "%SCRIPT_DIR%bridge" -B "%BUILD_DIR%" -A x64
if errorlevel 1 exit /b 1

echo [2/2] Building bridge...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b 1

echo.
echo SUCCESS: Built "%BUILD_DIR%\Release\veos_cosim_bridge.dll"
endlocal
