@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0"

set "ZIP_NAME=code_export.zip"
set "ROOT=%CD%\"
set "ZIP_PATH=%CD%\%ZIP_NAME%"
set "STAGE=%TEMP%\robot_code_export_%RANDOM%_%RANDOM%"

echo.
echo Projektordner:
echo %CD%
echo.

if exist "%ZIP_PATH%" (
    del /f /q "%ZIP_PATH%"
)

if exist "%STAGE%" (
    rd /s /q "%STAGE%"
)

mkdir "%STAGE%"

set /a COUNT=0

for /r "%CD%" %%F in (*.ino *.h *.cpp) do (
    set "FULL=%%~fF"
    set "SKIP=0"

    rem ------------------------------------------------------------
    rem Unerwuenschte Ordner ausschliessen
    rem ------------------------------------------------------------
    if /I not "!FULL:\tools\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\.vs\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\.git\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\Debug\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\Release\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\x64\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\VMBuilds\=!"=="!FULL!" set "SKIP=1"
    if /I not "!FULL:\__vm\=!"=="!FULL!" set "SKIP=1"

    rem ------------------------------------------------------------
    rem Visual-Micro-Hilfsdateien ausschliessen
    rem ------------------------------------------------------------
    if /I not "!FULL:.vsarduino.h=!"=="!FULL!" set "SKIP=1"

    rem ------------------------------------------------------------
    rem Datei mit gleicher Ordnerstruktur in Temp-Ordner kopieren
    rem ------------------------------------------------------------
    if "!SKIP!"=="0" (
        set "REL=!FULL:%ROOT%=!"
        set "TARGET=%STAGE%\!REL!"

        for %%D in ("!TARGET!") do (
            if not exist "%%~dpD" mkdir "%%~dpD"
        )

        copy /y "%%~fF" "!TARGET!" >nul
        set /a COUNT+=1
        echo + !REL!
    )
)

echo.
echo Gefundene Quelldateien: %COUNT%
echo.

if "%COUNT%"=="0" (
    echo Keine .ino, .h oder .cpp Dateien gefunden.
    rd /s /q "%STAGE%"
    pause
    exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -Command "Compress-Archive -Path (Join-Path $env:STAGE '*') -DestinationPath $env:ZIP_PATH -Force"

if errorlevel 1 (
    echo.
    echo FEHLER: ZIP-Datei konnte nicht erzeugt werden.
    echo Temp-Ordner bleibt zur Kontrolle erhalten:
    echo %STAGE%
    pause
    exit /b 1
)

rd /s /q "%STAGE%"

echo.
echo Fertig.
echo ZIP-Datei wurde erzeugt:
echo %ZIP_PATH%
echo.

pause