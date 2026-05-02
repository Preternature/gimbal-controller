@echo off
cd /d C:\Users\Woody\Desktop\Woody\gimbal-light-controller\servo_test

echo =============================
echo Compiling Servo Test...
echo =============================

arduino-cli compile --fqbn arduino:avr:mega .

IF %ERRORLEVEL% NEQ 0 (
    echo Compilation failed!
    pause
    exit /b %ERRORLEVEL%
)

echo =============================
echo Uploading to COM6...
echo (Edit this file to change COM port)
echo =============================

arduino-cli upload -p COM6 --fqbn arduino:avr:mega .

IF %ERRORLEVEL% NEQ 0 (
    echo Upload failed!
    echo Check that:
    echo   - Arduino is connected
    echo   - Correct COM port is set in this script
    echo   - No other program is using the port
    pause
    exit /b %ERRORLEVEL%
)

echo =============================
echo Done! Servo Test uploaded.
echo Open Serial Monitor at 9600 baud.
echo.
echo Commands:
echo   b = sweep base servo (pin 52)
echo   c = sweep camera servo (pin 36)
echo   a = sweep both
echo   n = center both
echo   0-9 = set angle (0=0deg, 5=90deg, 9=180deg)
echo =============================
pause
