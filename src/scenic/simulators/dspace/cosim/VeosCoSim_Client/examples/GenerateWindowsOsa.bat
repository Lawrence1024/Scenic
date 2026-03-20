:: Copyright dSPACE GmbH. All rights reserved.

@echo off

setlocal enabledelayedexpansion

set target=HostPC64/GCC
set config=Release
set osaFileName=VeosCoSimOsaForWindows.osa

set currentDir=%~dp0
set sourceDir=%currentDir%
set exportDir=%currentDir%\windows
set osaFile=%exportDir%\%osaFileName%

if not exist "%exportDir%" (
    mkdir "%exportDir%" || (
        echo Could not create export directory "%exportDir%"
        exit /b 1
    )
)

:uniqLoop
set buildDir=%temp%\Build%RANDOM%
if exist "%buildDir%" goto :uniqLoop
mkdir "%buildDir%" || (
    echo Could not create build directory "%buildDir%"
    exit /b 1
)

call :ChooseInstallation || exit /b 1
echo Found veos-build: %veosBuild%

for /f "delims=" %%x in ('dir /b /s "%sourceDir%\*.bsc"') do (
    call :BuildBsc "%%x" || exit /b 1
)

"%veosModel%" connect --autoconnect-signals "%osaFile%" || exit /b 1

echo Building of OSA finished successfully.
exit /b 0

:ChooseInstallation
call :FindInstallation 2471a1ab-2b77-41d8-8eea-0be7e018a5c5 && exit /b 0
call :FindInstallation 725DF84E-457E-408D-BAD6-3FFA8610F410 && exit /b 0
echo Could not find VEOS 5.4 installation
exit /b 1

:FindInstallation
set key=HKLM\SOFTWARE\WOW6432Node\dSPACE\InstallationInformation\InstallationInstances\%1
set value=InstallationRoot
set veosBuild=
set veosModel=
for /f "usebackq tokens=2,*" %%a in (`reg query %key% /v %value% 2^>nul`) do (
    set veosBuild=%%b\Bin\veos-build.exe
    set veosModel=%%b\Bin\veos-model.exe
    if exist "!veosBuild!" exit /b 0
)
exit /b 1

:BuildBsc
"%veosBuild%" bsc %1 --build-directory "%buildDir%" --output-file "%osaFile%" --configuration %config% --target %target% --preprocessor-defines "BUILD_ID=0" || (
    echo Could not build the BSC file %1
    exit /b 1
)

set buildInfoFile=%exportDir%\*.dsbuildinfo
del /q "%buildInfoFile%" || (
    echo Could not delete the file "%buildInfoFile%"
    exit /b 1
)

rmdir /s /q "%buildDir%" || (
    echo Could not delete the directory "%buildDir%"
    exit /b 1
)

exit /b 0
