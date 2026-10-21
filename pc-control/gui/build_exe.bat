@echo off
rem Builds dist\RoverControlPC.exe - the minimal rover GUI (rover_gui_min.py), controller on the PC.
rem Flash ..\firmware\rover_io\rover_io.ino to the Nano to use it.
rem Run from this folder: build_exe.bat
cd /d "%~dp0"
python -m pip install -r requirements.txt || exit /b 1
python -m PyInstaller --onefile --windowed --name RoverControlPC rover_gui_min.py || exit /b 1
echo.
echo Built: %~dp0dist\RoverControlPC.exe
