@echo off
setlocal enabledelayedexpansion

set EXE=%~dp0client\build\VeosCoSimTestClientIpc.exe
set VEOS_HOST=192.168.100.101
set VEOS_NAME=CoSimServerScenic
set IPC_PORT=50555
set PID_FILE=%TEMP%\scenic_cosim_ipc.pid

REM ── Check if IPC client is already running ────────────────────────────────
if exist "%PID_FILE%" (
    set /p EXISTING_PID=<"%PID_FILE%"
    tasklist /fi "PID eq %EXISTING_PID%" /fo csv 2>nul | findstr /i "VeosCoSimTestClientIpc" >nul
    if not errorlevel 1 (
        echo [Launcher] IPC client already running (pid=%EXISTING_PID%). Nothing to do.
        echo [Launcher] To force a restart: taskkill /im VeosCoSimTestClientIpc.exe /f
        goto :done
    )
    del "%PID_FILE%" >nul 2>&1
)

REM ── Start Docker stack in WSL ─────────────────────────────────────────────
echo [Launcher] Starting dSPACE Docker stack in WSL ...
wsl docker compose -f /home/bklfh/ros_ws/race_common/tools/dspace/dspace_art_stack.yml up -d
if errorlevel 1 (
    echo [Launcher] ERROR: docker compose up failed.
    exit /b 1
)

REM ── Wait for VEOS to be reachable via ctun VPN ───────────────────────────
echo [Launcher] Waiting for VEOS at %VEOS_HOST% to become reachable ...
:wait_loop
ping -n 1 -w 1000 %VEOS_HOST% >nul 2>&1
if errorlevel 1 (
    timeout /t 5 /nobreak >nul
    goto wait_loop
)
echo [Launcher] VEOS reachable. Waiting 30s for CoSim server init ...
timeout /t 30 /nobreak >nul

REM ── Launch IPC client in its own console window ───────────────────────────
echo [Launcher] Launching VeosCoSimTestClientIpc.exe ...
start "VEOS IPC Client" "%EXE%" --host %VEOS_HOST% --name %VEOS_NAME% --ipc-port %IPC_PORT%

REM Give the process a moment to appear in tasklist
timeout /t 2 /nobreak >nul

REM Save PID (best effort)
for /f "tokens=2 delims=," %%P in ('tasklist /fi "imagename eq VeosCoSimTestClientIpc.exe" /fo csv /nh 2^>nul') do (
    set RAW_PID=%%P
    set RAW_PID=!RAW_PID:"=!
    echo !RAW_PID!>"%PID_FILE%"
    echo [Launcher] IPC client PID=!RAW_PID! saved to %PID_FILE%
    goto :pid_saved
)
echo [Launcher] WARNING: could not determine IPC client PID.
:pid_saved

:done
echo [Launcher] Ready. Run Python/Scenic scripts normally.
echo [Launcher] The IPC client will reconnect to each new bridge automatically.
echo [Launcher] To stop: taskkill /im VeosCoSimTestClientIpc.exe /f ^&^& wsl docker compose -f /home/bklfh/ros_ws/race_common/tools/dspace/dspace_art_stack.yml down
