@echo off
REM Change into the sketch folder
cd /d C:\Users\Woody\Desktop\gimbal-light-controller

echo =============================
echo Compiling Gimbal Light Controller...
echo =============================

REM Create temporary folder with correct naming structure for arduino-cli
if exist temp_sketch rmdir /s /q temp_sketch
mkdir temp_sketch
copy gimbal_controller.ino temp_sketch\temp_sketch.ino >nul

cd temp_sketch
arduino-cli compile --fqbn arduino:avr:mega .

IF %ERRORLEVEL% NEQ 0 (
    echo Compilation failed!
    cd ..
    rmdir /s /q temp_sketch
    pause
    exit /b %ERRORLEVEL%
)

echo =============================
echo Uploading to COM port...
echo (Change COM port below if needed)
echo =============================

arduino-cli upload -p COM6 --fqbn arduino:avr:mega .

IF %ERRORLEVEL% NEQ 0 (
    echo Upload failed!
    echo Check that:
    echo   - Arduino is connected
    echo   - Correct COM port is set in this script
    echo   - No other program is using the port
    cd ..
    rmdir /s /q temp_sketch
    pause
    exit /b %ERRORLEVEL%
)

REM Clean up
cd ..
rmdir /s /q temp_sketch

echo =============================
echo Done! Gimbal Controller uploaded.
echo Open Serial Monitor at 9600 baud to send commands.
echo =============================
echo.
echo Quick Start:
echo   - Type 'START' to begin swinging
echo   - Type 'STOP' to stop
echo   - Type 'PRESET:GENTLE' for slow swing
echo   - Type 'PRESET:DYING' for swing that fades out
echo   - Type 'STATUS' to see current settings
echo =============================
pause
