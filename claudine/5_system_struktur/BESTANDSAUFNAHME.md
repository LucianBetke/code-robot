# 5_System: Bestandsaufnahme und Umbauvorschläge

Stand: 11.08.2026
Ausgangspunkt: Commit `76e38ee` (Startsperre bei zu leerem Akku)

Diese Datei ist als Übergabe an eine neue Sitzung gedacht. Sie hält
fest, was untersucht wurde, was Befund und was Vermutung ist, und was
als Nächstes ansteht. **Am Code wurde nichts geändert.**

## Anlass

Beim Zählen der Zeilen in der Projektmappe fiel 5_System als mit
Abstand größte Schicht auf.

| Projekt | Dateien | Zeilen |
|---|---|---|
| 1_Common | 8 | 664 |
| 2_Hardware | 18 | 1.310 |
| 3_Control | 9 | 732 |
| 4_Vehicle | 6 | 385 |
| **5_System** | **19** | **2.703** |
| vorne | 5 | 643 |
| hinten | 3 | 421 |

Gezählt sind `.h`, `.cpp`, `.ino` inklusive Kommentaren und
Leerzeilen. Außerhalb der beiden Firmwares liegen zusätzlich
`funk_empfaenger` (246) und `tools` (2.150).

Die Größe allein ist kein Befund: 5_System ist die oberste Schicht
eines Systems mit zwei Nanos, zwei Protokollen und Telemetrie.

## Gewichtsverteilung innerhalb von 5_System

| Datei | Zeilen |
|---|---|
| UltrasonicManager.cpp | 515 |
| CommandRunner/CommandRunner.cpp | 459 |
| TelemetryPrinter.cpp | 325 |
| RearFrameClient.cpp | 243 |
| RadioTelemetrySender.cpp | 214 |
| UltrasonicManager.h | 124 |
| Parser/CommandParser.cpp | 119 |
| RearFrameClient.h | 114 |
| RadioTelemetrySender.h | 91 |
| CommandRunner/CommandRunner.h | 86 |
| TelemetryPrinter.h | 79 |
| Connection/ConnectionMonitor.cpp | 68 |
| TelemetryOutput.cpp | 53 |
| FrameScheduler.cpp | 50 |
| TelemetryOutput.h | 40 |
| FrameScheduler.h | 39 |
| Parser/CommandParser.h | 32 |
| Connection/ConnectionMonitor.h | 30 |
| TelemetryPrinterConfig.h | 22 |

## Befund 1: UltrasonicManager liegt eine Schicht zu hoch

**Belegt, nicht vermutet** — aus dem Dateikopf von
`UltrasonicManager.h`:

> HC-SR04 nicht blockierend ansteuern · Timeout, Guard-Time,
> Plausibilität und Medianfilter · aktuellen Datensatz für den
> UART-Transfer bereitstellen · Messbetrieb ein- und ausschalten

Das ist ein Sensortreiber ohne Anwendungslogik. Sein Gegenstück
`UltrasonicEchoCapture` liegt bereits in 2_Hardware, direkt neben
`Encoder` und `Motor`. Die beiden gehören nebeneinander.

Umfang: 639 Zeilen, knapp ein Viertel der Schicht.

### Aufwand

Voraussichtlich gering. Die Includes müssten vermutlich **nicht**
angefasst werden: jede Projektmappe legt ihr eigenes Verzeichnis über
`AdditionalIncludeDirectories` auf den Include-Pfad, `#include
"src/UltrasonicManager.h"` löst nach dem Umzug also weiterhin auf.
Das ist eine Vermutung aus dem Lesen der `.vcxitems` und muss beim
ersten Build geprüft werden.

Zu ändern: `2_Hardware/2_Hardware.vcxitems` und
`5_System/5_System.vcxitems`, jeweils `ClCompile` und `ClInclude`,
dazu die zugehörigen `.filters`.

**Beide Firmwares müssen neu übersetzt werden**, auch `vorne` — der
vordere Nano benutzt den Manager für die beiden seitlichen Sensoren.

## Befund 2: Die Ordnerstruktur ist halb angewandt

Es gibt drei Unterordner — `CommandRunner/`, `Connection/`,
`Parser/` — und daneben liegen neun Dateien lose im Wurzelverzeichnis,
darunter die vier zusammengehörigen Telemetriedateien
(`TelemetryPrinter`, `TelemetryOutput`, `RadioTelemetrySender`,
`TelemetryPrinterConfig`, zusammen 824 Zeilen).

Entweder Unterordner für alles oder für nichts. So muss man raten,
wonach gruppiert wurde.

Reines Verschieben, berührt aber viele Includes — lohnt sich eher
gemeinsam mit Befund 1.

## Befund 3: CommandRunner mischt vermutlich zwei Aufgaben

**Ausdrücklich Vermutung.** Grundlage ist nur `CommandRunner.h`, die
`.cpp` mit ihren 459 Zeilen wurde nicht gelesen.

Der Header zeigt zwei Themen nebeneinander:

- **Systemebene:** iteriert über eine Befehlsliste (`GetCmdFn`,
  `SizeFn`), gibt Protokollzeilen auf ein `Print`-Ziel aus, verwaltet
  CMDP-Ids.
- **Fahrzeugebene:** Zustandsmaschine für Translations- und
  Rotationsbefehle mit Fortschritt, Zieltoleranz, Timeouts und
  Settle-Phase; hängt an `VehicleController` und `MecanumOdometer`
  aus 4_Vehicle.

Ob sich das sinnvoll trennen lässt, ist ohne Lesen der `.cpp` nicht
zu beantworten. Es ist Umbauarbeit an Code, der fährt — deutlich
riskanter als Befund 1 und 2.

## Empfohlene Reihenfolge

1. `CommandRunner.cpp` lesen und beurteilen, ob Befund 3 trägt.
   Kostet nichts und ändert nichts.
2. Umzug des UltrasonicManagers nach 2_Hardware vorbereiten
   (Befund 1). Klarster Nutzen, geringstes Risiko.
3. Ordnerstruktur vereinheitlichen (Befund 2), am besten im selben
   Zug wie 2.
4. Befund 3 nur, wenn Schritt 1 ihn bestätigt.

## Randbedingungen

Aus `CLAUDE.md`:

- Vorschläge als vollständige Dateien nach `claudine`, nie direkt ins
  Projekt.
- Echte Projektdateien nur nach dem ausdrücklichen „Endfassung
  übernehmen".
- Keine Commits, kein Push, kein Upload ohne ausdrückliche Erlaubnis.

Zusätzlich aus der Erfahrung der letzten Sitzung: `flash_beide.ps1`
bricht ab, sobald eine Quelldatei neuer ist als das jeweilige Hex.
Ein Eingriff in die gemeinsamen Ordner macht damit **beide** Projekte
flashpflichtig. Für Änderungen, die nur einen Nano betreffen, ist es
sauberer, das betroffene Projekt einzeln aus Visual Micro zu bauen
und hochzuladen.
