@echo off
cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-ChildItem -Path . -Recurse -Include *.ino,*.h,*.cpp -File | Where-Object { $_.FullName -notmatch '\\tools\\' } | Sort-Object FullName | ForEach-Object { '============================================================'; 'FILE: ' + $_.FullName; '============================================================'; Get-Content -LiteralPath $_.FullName; '' } | Set-Content -LiteralPath 'code_ohne_tools.txt' -Encoding UTF8"

echo.
echo Fertig. Datei wurde erzeugt: code_ohne_tools.txt
pause