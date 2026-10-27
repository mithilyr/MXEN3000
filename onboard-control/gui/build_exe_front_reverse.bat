@echo off
rem Builds dist\RoverControl_front_reverse.exe - RoverControl with only the Front and Reverse modes.
rem Pairs with firmware\rover2026_front_reverse. Run from this folder: build_exe_front_reverse.bat
cd /d "%~dp0"
python -m pip install -r requirements.txt || exit /b 1
python -m PyInstaller --onefile --windowed --name RoverControl_front_reverse rover_gui_front_reverse.py || exit /b 1
echo.
echo Built: %~dp0dist\RoverControl_front_reverse.exe
