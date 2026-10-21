@echo off
rem Builds dist\RoverControlMap.exe - the rover telemetry dashboard (rover_gui_map.py).
rem RoverControl.exe (build_exe.bat) is not affected.
rem Run from this folder: build_exe_map.bat
cd /d "%~dp0"
python -m pip install -r requirements.txt || exit /b 1
python -m PyInstaller --onefile --windowed --name RoverControlMap rover_gui_map.py || exit /b 1
echo.
echo Built: %~dp0dist\RoverControlMap.exe
