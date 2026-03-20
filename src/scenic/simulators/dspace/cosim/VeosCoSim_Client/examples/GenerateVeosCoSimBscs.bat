:: Copyright dSPACE GmbH. All rights reserved.

@echo off

setlocal

set currentDir=%~dp0
set generator=%currentDir%..\bscgenerator\bin\VeosCoSimBscGenerator.exe

for /f %%x in ('dir /s /b *.json') do (
    "%generator%" --input "%%x" --output "%currentDir%%%~nx.bsc" || (
        echo Could not generate "%%x" to "%currentDir%%%~nx.bsc"
        exit /b 1
    )
    echo.
)
