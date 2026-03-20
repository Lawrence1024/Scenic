@echo off
setlocal

set ROOT=%~dp0
set CLIENT_DIR=%ROOT%client
set BUILD_DIR=%CLIENT_DIR%\build
set VENDOR_ROOT=%ROOT%..\VeosCoSim_Client
set VENDOR_INCLUDE=%VENDOR_ROOT%\client\x64\Release\include
set VENDOR_LIB=%VENDOR_ROOT%\client\x64\Release\lib
set EXAMPLE_DIR=%VENDOR_ROOT%\examples\client

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo [1/2] Compiling VeosCoSimTestClientIpc.exe ...

pushd "%BUILD_DIR%"

cl /std:c++17 /EHsc /MD ^
  /I "%VENDOR_INCLUDE%" ^
  /I "%EXAMPLE_DIR%" ^
  "%CLIENT_DIR%\VeosCoSimTestClientIpc.cpp" ^
  "%EXAMPLE_DIR%\ClientServerTestHelper.cpp" ^
  "%EXAMPLE_DIR%\Generator.cpp" ^
  "%CLIENT_DIR%\TcpEventClient.cpp" ^
  /link ^
  /LIBPATH:"%VENDOR_LIB%" ^
  VeosCoSimApplStatic.lib Ws2_32.lib ^
  /OUT:"VeosCoSimTestClientIpc.exe"

if errorlevel 1 (
  echo Build failed.
  popd
  exit /b 1
)

popd

echo [2/2] Build finished: "%BUILD_DIR%\VeosCoSimTestClientIpc.exe"
endlocal