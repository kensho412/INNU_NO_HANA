@echo off
rem Start Go2 RC simulator + /gait/force OSC sender (double-click to run)
rem
rem Default venue: Zou-no-hana Terrace (venues\zounohana_terrace.usdc, built by tools\build_zounohana_terrace.py)
rem   Go2 starts on the plaza 6 m in front of the main (north-east) entrance, facing it.
rem   To use the demo obstacle course instead, make VENUE empty:  set VENUE=
rem Drag and drop a .usd/.usdc/.usda file (e.g. exported from Blender) onto this file to load that instead.
rem To send to the Mac, change HOST to the Mac's IP address.
set HOST=127.0.0.1
set VENUE=venues\zounohana_terrace.usdc
set SPAWN_YAW=-158
rem 8x8 stepping grid in front of Go2 (heights from Max: /grid/height to this PC, UDP 9872). To turn it off:  set GRID=
set GRID=--grid
rem In the terrace the grid sits inside the hall, 1.5 m past the main entrance (Go2 starts 6 m outside it)
set GRID_IN_VENUE=--grid-ahead 7.6
set OMNI_KIT_ACCEPT_EULA=YES
cd /d "%~dp0"
set PY="%~dp0env_isaacsim\Scripts\python.exe"
set ARGS=--host %HOST% --port 9871 --print-force %GRID%
if not "%~1"=="" (
    %PY% go2_rc.py %ARGS% --course none --usd "%~1"
) else if not "%VENUE%"=="" (
    %PY% go2_rc.py %ARGS% --course none --usd "%VENUE%" --spawn-yaw %SPAWN_YAW% %GRID_IN_VENUE%
) else (
    %PY% go2_rc.py %ARGS%
)
pause
