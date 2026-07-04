$avrdude   = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe"
$avrconf   = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf"
$hexHinten = "C:\Users\lucia\AppData\Local\Temp\VMBuilds\hinten\nano_atmega328old\Release\hinten.ino.hex"
$hexVorne  = "C:\Users\lucia\AppData\Local\Temp\VMBuilds\vorne\nano_atmega328old\Release\vorne.ino.hex"

Write-Host "=== Flash hinten (COM5) ===" -ForegroundColor Cyan
& $avrdude -C $avrconf -p atmega328p -c arduino -P COM5 -b 57600 -D -U flash:w:$hexHinten`:i
if ($LASTEXITCODE -ne 0) { Write-Host "FEHLER hinten!" -ForegroundColor Red; exit 1 }

Write-Host "=== Flash vorne (COM6) ===" -ForegroundColor Cyan
& $avrdude -C $avrconf -p atmega328p -c arduino -P COM6 -b 57600 -D -U flash:w:$hexVorne`:i
if ($LASTEXITCODE -ne 0) { Write-Host "FEHLER vorne!" -ForegroundColor Red; exit 1 }

Write-Host "=== Fertig ===" -ForegroundColor Green
