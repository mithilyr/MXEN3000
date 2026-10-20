@echo off
rem Builds dist\RoverControl.exe - the single executable required for the competition.
rem Run from this folder: build_exe.bat
cd /d "%~dp0"
python -m pip install -r requirements.txt || exit /b 1
python -m PyInstaller --onefile --windowed --name RoverControl rover_gui.py || exit /b 1
echo.
echo Built: %~dp0dist\RoverControl.exe
