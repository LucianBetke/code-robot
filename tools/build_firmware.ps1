# ============================================================
# build_firmware.ps1
#
# Kompiliert 'vorne' oder 'hinten' ausserhalb von Visual Studio
# und meldet Flash- und SRAM-Verbrauch. Reiner Pruef-Build:
# es wird nichts geflasht und nichts im Projekt veraendert.
#
# Der Build-Baum entsteht unter %TEMP%\robot_build_check.
# Welche Dateien zum Build gehoeren, wird der letzten
# Visual-Micro-Uebersetzung entnommen; die Inhalte kommen
# immer frisch aus dem Repository.
#
# Aufruf:  .\tools\build_firmware.ps1 -Project vorne
#
# Hinweis: Die Groessen liegen reproduzierbar rund 20 Byte
# unter dem offiziellen Visual-Micro-Ergebnis (andere
# Link-Reihenfolge). Fuer den verbindlichen Wert bitte in
# Visual Micro uebersetzen.
# ============================================================

param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("vorne", "hinten")]
    [string] $Project
)

$ErrorActionPreference = "Stop"

$gccDir = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin"
$gpp = Join-Path $gccDir "avr-g++.exe"
$gcc = Join-Path $gccDir "avr-gcc.exe"
$sizeExe = Join-Path $gccDir "avr-size.exe"

$coreA = "C:\Users\lucia\AppData\Local\Temp\VMBCore\arduino20x\064554253855f7bbb31645e3d6be7bd4\core.a"
$avrCore = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\hardware\avr\1.8.8\cores\arduino"
$avrVar = "C:\Users\lucia\AppData\Local\arduino15\packages\arduino\hardware\avr\1.8.8\variants\eightanaloginputs"

$repo = Split-Path -Parent $PSScriptRoot
$vmSource = "C:\Users\lucia\AppData\Local\Temp\VMBuilds\$Project\nano_atmega328old\Release"
$buildDir = Join-Path $env:TEMP "robot_build_check\$Project"

foreach ($required in @($gpp, $coreA, $vmSource))
{
    if (-not (Test-Path $required))
    {
        Write-Host "FEHLT: $required" -ForegroundColor Red
        exit 1
    }
}

# Schichtordner, aus denen die gemeinsamen Dateien stammen.
$layerDirs = @(
    "$repo\1_Common\src",
    "$repo\2_Hardware\src",
    "$repo\3_Control\src",
    "$repo\4_Vehicle\src",
    "$repo\5_System\src"
)

# --- Build-Baum aufbauen -----------------------------------

if (Test-Path $buildDir)
{
    Remove-Item $buildDir -Recurse -Force
}

New-Item -ItemType Directory -Path $buildDir -Force | Out-Null

Copy-Item "$vmSource\*" $buildDir -Recurse -Force

Get-ChildItem $buildDir -Recurse -File |
    Where-Object {
        $_.Extension -in @('.o', '.d', '.elf', '.hex', '.eep', '.bin') -or
        $_.Name -like '*.vmpreproc'
    } |
    Remove-Item -Force

Remove-Item "$buildDir\.vmintelli", "$buildDir\.vmpreproc" `
    -Recurse -Force -ErrorAction SilentlyContinue

# Sketch-Dateien direkt aus dem Projektordner.
foreach ($file in Get-ChildItem "$repo\$Project\src" -File)
{
    if ($file.Extension -in @('.cpp', '.h'))
    {
        Copy-Item $file.FullName "$buildDir\$($file.Name)" -Force
    }
}

# Gemeinsame Dateien aus den Schichtordnern nachziehen.
$refreshed = 0
$missing = @()

foreach ($file in Get-ChildItem "$buildDir\src" -Recurse -File)
{
    if ($file.Extension -notin @('.cpp', '.h'))
    {
        continue
    }

    $source = $null

    foreach ($dir in $layerDirs)
    {
        $candidate = Join-Path $dir $file.Name
        if (Test-Path $candidate)
        {
            $source = $candidate
            break
        }

        $candidate = Join-Path $dir "$($file.Directory.Name)\$($file.Name)"
        if (Test-Path $candidate)
        {
            $source = $candidate
            break
        }
    }

    if ($null -eq $source)
    {
        $missing += $file.Name
        continue
    }

    Copy-Item $source $file.FullName -Force
    $refreshed++
}

Write-Host ("Quellen aus Repo uebernommen: {0}" -f $refreshed) -ForegroundColor DarkGray

if ($missing.Count -gt 0)
{
    Write-Host ("Nicht im Repo gefunden: {0}" -f ($missing -join ', ')) -ForegroundColor Yellow
}

# --- Uebersetzen -------------------------------------------

$cflags = @(
    "-c", "-g", "-Os", "-w", "-std=gnu++11", "-fpermissive",
    "-fno-exceptions", "-ffunction-sections", "-fdata-sections",
    "-fno-threadsafe-statics", "-Wno-error=narrowing", "-MMD", "-flto",
    "-mmcu=atmega328p", "-DF_CPU=16000000L", "-DARDUINO=108010",
    "-DARDUINO_AVR_NANO", "-DARDUINO_ARCH_AVR",
    "-I$repo\$Project\src",
    "-I$repo\1_Common", "-I$repo\2_Hardware", "-I$repo\3_Control",
    "-I$repo\4_Vehicle", "-I$repo\5_System",
    "-I$avrCore", "-I$avrVar"
)

Push-Location $buildDir
try
{
    $objs = @()
    $failed = $false

    $sources = Get-ChildItem $buildDir -Recurse -File -Filter *.cpp |
        Sort-Object FullName

    foreach ($src in $sources)
    {
        $obj = "$($src.FullName).o"
        $objs += $obj

        & $gpp @cflags $src.FullName -o $obj

        if ($LASTEXITCODE -ne 0)
        {
            Write-Host "COMPILE-FEHLER: $($src.Name)" -ForegroundColor Red
            $failed = $true
        }
    }

    if ($failed)
    {
        Write-Host "=== $Project : BUILD FEHLGESCHLAGEN ===" -ForegroundColor Red
        exit 1
    }

    $elf = Join-Path $buildDir "$Project.ino.elf"

    $ldflags = @(
        "-w", "-Os", "-g", "-flto", "-fuse-linker-plugin",
        "-Wl,--gc-sections", "-mmcu=atmega328p"
    )

    & $gcc @ldflags -o $elf @objs $coreA "-L$buildDir" -lm

    if ($LASTEXITCODE -ne 0)
    {
        Write-Host "=== $Project : LINK FEHLGESCHLAGEN ===" -ForegroundColor Red
        exit 1
    }

    $text = 0
    $data = 0
    $bss = 0

    foreach ($line in (& $sizeExe -A $elf))
    {
        if ($line -match '^\.text\s+(\d+)') { $text = [int]$Matches[1] }
        if ($line -match '^\.data\s+(\d+)') { $data = [int]$Matches[1] }
        if ($line -match '^\.bss\s+(\d+)') { $bss = [int]$Matches[1] }
    }

    Write-Host "=== $Project : BUILD OK ===" -ForegroundColor Green
    Write-Host ("  Flash: {0,6} Bytes   ({1:N1} % von 30720)" -f ($text + $data), (($text + $data) / 30720 * 100))
    Write-Host ("  SRAM:  {0,6} Bytes   ({1:N1} % von 2048)" -f ($data + $bss), (($data + $bss) / 2048 * 100))
}
finally
{
    Pop-Location
}
