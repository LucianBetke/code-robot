# 5_System flach legen, Gruppierung in die Ansicht

Stand: 11.08.2026
Ausgangspunkt: Commit `15176c8` (nach dem Umzug des UltrasonicManager)
Das ist Befund 2 aus [BESTANDSAUFNAHME.md](../5_system_struktur/BESTANDSAUFNAHME.md),
Richtung A.

**Am Projekt ist nichts geändert.** Alles hier ist Vorlage.

## Die Entscheidung

Auf der Platte flach, gruppiert wird in der Ansicht.

Drei Gründe für flach:

1. **Die Namen gruppieren schon.** `TelemetryOutput`,
   `TelemetryPrinter`, `TelemetryPrinterConfig` stehen alphabetisch
   ohnehin beieinander. Ein Ordner `Telemetry/` sagte dasselbe ein
   zweites Mal.
2. **17 Dateien sind übersichtlich.** 2_Hardware hat 18 und liest
   sich mühelos, weil jeder Name sein Thema trägt.
3. **Alle anderen vier Schichten sind flach.** Eine abweichende
   Schicht zwingt dazu, bei jeder anderen nachzusehen, wie sie es
   hält.

Und die Gruppierung geht trotzdem nicht verloren: die
`.vcxitems.filters` beschreibt einen eigenen Baum für den
Projektmappen-Explorer. Sie steuert nur die Ansicht, nicht den Bau —
der Compiler liest sie nie, die Dateiliste kommt aus der
`.vcxitems`. Bisher enthielt sie überhaupt keine Filterdefinitionen,
die drei Ordner waren in Visual Studio also gar nicht zu sehen.

Neu sind drei Gruppen:

| Gruppe | Dateien |
|---|---|
| Befehle | CommandParser, CommandRunner |
| Telemetrie | RadioTelemetrySender, TelemetryOutput, TelemetryPrinter, TelemetryPrinterConfig |
| Verbindung | ConnectionMonitor, FrameScheduler, RearFrameClient |

Offen und bewusst vertagt: auf GitHub sieht man die Platte und nicht
diese Ansicht. Wenn das Veröffentlichen konkret wird, ist das noch
einmal anzusehen.

## Geprüft, nicht vermutet

Beide Firmwares wurden je zweimal übersetzt, aus einer unveränderten
Spiegelung des Quellbaums und aus einer flach gelegten:

| Projekt | Flash vorher → nachher | SRAM vorher → nachher |
|---|---|---|
| vorne | 26.400 → 26.400 | 1.306 → 1.306 |
| hinten | 13.880 → 13.880 | 711 → 711 |

Byteweise identisch. Das Skript prüft zusätzlich, dass genau vier
Include-Zeilen ersetzt wurden, und bricht sonst ab.

Skript: `flachtest.ps1` im Scratchpad dieser Sitzung.

## Was zu tun ist

### 1. Sechs Dateien eine Ebene hoch, drei Ordner löschen

```
5_System/src/CommandRunner/CommandRunner.h    ->  5_System/src/
5_System/src/CommandRunner/CommandRunner.cpp  ->  5_System/src/
5_System/src/Connection/ConnectionMonitor.h   ->  5_System/src/
5_System/src/Connection/ConnectionMonitor.cpp ->  5_System/src/
5_System/src/Parser/CommandParser.h           ->  5_System/src/
5_System/src/Parser/CommandParser.cpp         ->  5_System/src/
```

Danach die leeren Ordner `CommandRunner/`, `Connection/` und
`Parser/` entfernen. Mit `git mv` bleibt die Historie erhalten.

### 2. Fünf Dateien aus diesem Ordner ersetzen

- `5_System/src/CommandRunner.h` — eine Include-Zeile
- `vorne/src/FrontApp.h` — zwei Include-Zeilen
- `hinten/src/RearApp.h` — eine Include-Zeile
- `5_System/5_System.vcxitems` — Pfade ohne Ordner, alphabetisch
  sortiert wie in 2_Hardware
- `5_System/5_System.vcxitems.filters` — dasselbe, dazu die drei
  Gruppen

Die übrigen fünf Dateien in `5_System/src/` dieses Ordners sind
unveränderte Kopien; sie liegen nur bei, damit die Vorlage
vollständig ist.

Kodierung und Zeilenenden bleiben erhalten: die Quelldateien sind
reines ASCII ohne BOM (`FrontApp.h` und `CommandRunner.h` mit CRLF,
`RearApp.h` mit LF), die beiden Projektdateien UTF-8 mit BOM, CRLF,
ohne abschließenden Umbruch.

Die Bezeichner der drei Filtergruppen sind fest vergeben, nicht
zufällig erzeugt. Ein erneutes Erzeugen der Datei liefert also
denselben Inhalt und keinen Diff-Lärm.

### 3. Beide Firmwares bauen und flashen

Wieder gemeinsame Ordner, also wieder beide Nanos.

Ein Unterschied zum Umzug des UltrasonicManager: dort blieben die
Zeitstempel unberührt, und `flash_beide.ps1` hielt hinten
fälschlich für aktuell. Hier ändern sich drei echte Quelldateien,
die Prüfung greift also.

Die Binärdateien bleiben trotzdem dieselben — nötig ist das Flashen
streng genommen nicht, sinnvoll nur, damit Visual Micro und die
Hex-Dateien zum Quellstand passen.

## Was sich nicht ändert

- Keine Zeile Programmlogik. Die vier geänderten Zeilen sind
  ausschließlich Include-Pfade.
- Keine `.vcxproj`. Sie importieren die `.vcxitems` und zählen
  keine Einzeldateien auf.
- Kein Verhalten am Fahrzeug.

## Zurückdrehen

Sechs Dateien zurückschieben, die fünf geänderten Dateien aus Git
wiederherstellen, beide Firmwares neu bauen.
