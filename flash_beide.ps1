$avrdude = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe"
$avrconf = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf"

$vmBuilds = "C:\Users\lucia\AppData\Local\Temp\VMBuilds"
$repo     = $PSScriptRoot

# Der Maschinencode ist bei 'atmega328' und 'atmega328old'
# identisch - die Varianten unterscheiden sich nur in der
# Upload-Geschwindigkeit. Darum wird einfach das neueste
# Hex genommen, egal welche Variante zuletzt gebaut wurde.
$variants = @("nano_atmega328", "nano_atmega328old")

# Quellordner, die in beide Firmwares einfliessen.
$sourceDirs = @(
    "1_Common\src",
    "2_Hardware\src",
    "3_Control\src",
    "4_Vehicle\src",
    "5_System\src"
)

function Get-NewestSourceTime
{
    param([string[]] $ProjectDirs)

    $dirs = @()
    $dirs += $sourceDirs
    $dirs += $ProjectDirs

    $newest = [DateTime]::MinValue

    foreach ($dir in $dirs)
    {
        $full = Join-Path $repo $dir
        if (-not (Test-Path $full)) { continue }

        $files = Get-ChildItem -Path $full -Recurse -File `
            -Include *.c, *.cpp, *.h, *.ino -ErrorAction SilentlyContinue

        foreach ($file in $files)
        {
            if ($file.LastWriteTime -gt $newest)
            {
                $newest = $file.LastWriteTime
            }
        }
    }

    return $newest
}

function Resolve-Hex
{
    param(
        [string]   $Project,
        [string[]] $ProjectDirs
    )

    $best = $null

    foreach ($variant in $variants)
    {
        $candidate = Join-Path $vmBuilds "$Project\$variant\Release\$Project.ino.hex"
        if (-not (Test-Path $candidate)) { continue }

        $item = Get-Item $candidate
        if ($null -eq $best -or $item.LastWriteTime -gt $best.LastWriteTime)
        {
            $best = $item
        }
    }

    if ($null -eq $best)
    {
        Write-Host "FEHLER: kein Hex fuer '$Project' gefunden." -ForegroundColor Red
        Write-Host "        Gesucht in: $vmBuilds\$Project\{$($variants -join ', ')}\Release" -ForegroundColor Red
        exit 1
    }

    # Schutz gegen das Flashen eines veralteten Builds:
    # ist irgendeine Quelldatei neuer als das Hex, wurde
    # nach der letzten Aenderung nicht neu kompiliert.
    $srcTime = Get-NewestSourceTime -ProjectDirs $ProjectDirs

    if ($srcTime -gt $best.LastWriteTime)
    {
        Write-Host "FEHLER: '$Project' ist aelter als der Quellcode!" -ForegroundColor Red
        Write-Host ("        Hex:    {0}  ({1})" -f $best.FullName, $best.LastWriteTime) -ForegroundColor Red
        Write-Host ("        Quelle: {0}" -f $srcTime) -ForegroundColor Red
        Write-Host "        Bitte in Visual Micro neu kompilieren." -ForegroundColor Red
        exit 1
    }

    Write-Host ("{0,-6} <- {1}" -f $Project, $best.FullName) -ForegroundColor DarkGray
    Write-Host ("{0,-6}    Stand {1}" -f "", $best.LastWriteTime) -ForegroundColor DarkGray

    return $best.FullName
}

$hexHinten = Resolve-Hex -Project "hinten" -ProjectDirs @("hinten\src")
$hexVorne  = Resolve-Hex -Project "vorne"  -ProjectDirs @("vorne\src")

# -b 57600 gehoert zum Bootloader im Chip, nicht zum Hex.
# Nanos mit altem Bootloader brauchen 57600, neuere 115200.
Write-Host "=== Flash hinten (COM5) ===" -ForegroundColor Cyan
& $avrdude -C $avrconf -p atmega328p -c arduino -P COM5 -b 57600 -D -U flash:w:$hexHinten`:i
if ($LASTEXITCODE -ne 0) { Write-Host "FEHLER hinten!" -ForegroundColor Red; exit 1 }

Write-Host "=== Flash vorne (COM6) ===" -ForegroundColor Cyan
& $avrdude -C $avrconf -p atmega328p -c arduino -P COM6 -b 57600 -D -U flash:w:$hexVorne`:i
if ($LASTEXITCODE -ne 0) { Write-Host "FEHLER vorne!" -ForegroundColor Red; exit 1 }

Write-Host "=== Fertig ===" -ForegroundColor Green
