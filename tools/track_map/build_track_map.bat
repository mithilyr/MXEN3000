@echo off
rem Builds TrackMap.exe into ..\..\onboard-control\gui\dist, next to RoverControl.exe,
rem so it finds the GUI's ir-log CSV files without choosing a folder.
rem For the PC-control build, pick pc-control\gui\dist\logs as the folder in TrackMap.
rem Run from this folder: build_track_map.bat
cd /d "%~dp0"
python -m pip install pyinstaller || exit /b 1
python -m PyInstaller --onefile --windowed --name TrackMap --distpath ..\..\onboard-control\gui\dist --workpath build --specpath build track_map.py || exit /b 1
echo.
echo Built: %~dp0..\..\onboard-control\gui\dist\TrackMap.exe
