# Umzug: UltrasonicManager von 5_System nach 2_Hardware

Stand: 11.08.2026
Ausgangspunkt: Commit `76e38ee`
Das ist Befund 1 aus [BESTANDSAUFNAHME.md](../5_system_struktur/BESTANDSAUFNAHME.md).

**Am Projekt ist nichts geändert.** Alles hier ist Vorlage.
Befund 2 (Unterordner in 5_System) bleibt bewusst außen vor.

## Warum

`UltrasonicManager` ist ein Sensortreiber: Trigger, Timeout,
Guard-Time, Plausibilität, Medianfilter. Sein Gegenstück
`UltrasonicEchoCapture` liegt bereits in 2_Hardware.

Der Beleg ist die Abhängigkeitsrichtung. Die Datei bindet nichts ein
außer `Arduino.h` und `UltrasonicEchoCapture.h` — aus ihrer eigenen
Schicht braucht sie keine einzige Datei. Etwas, das nur nach unten
zeigt, gehört nach unten.

Wie sie überhaupt in 5_System landete, steht in
[BEFUND3_COMMANDRUNNER.md](../5_system_struktur/BEFUND3_COMMANDRUNNER.md)
nicht, sondern hier kurz: als sie entstand, hingen alle drei Sensoren
am hinteren Nano, und die Klasse bestimmte die Messreihenfolge für den
ganzen Roboter — eine Systemaufgabe. Seit die Seitensensoren vorne
sitzen, kommt der Takt von außen über `requestMeasurement()`. Die
Systemaufgabe ist aus der Klasse herausgewandert, die Datei ist
liegengeblieben.

## Geprüft, nicht vermutet

Beide Firmwares wurden je zweimal übersetzt, einmal aus einer
unveränderten Spiegelung des Quellbaums und einmal mit verschobener
Datei:

| Projekt | Flash vorher → nachher | SRAM vorher → nachher |
|---|---|---|
| vorne | 26.400 → 26.400 | 1.306 → 1.306 |
| hinten | 13.880 → 13.880 | 711 → 711 |

Byteweise identisch. Der Bau lief durch, **ohne dass eine einzige
Include-Zeile geändert wurde** — die Vermutung aus der
Bestandsaufnahme trägt: jede Schicht legt über
`AdditionalIncludeDirectories` ihr eigenes Verzeichnis auf den
Suchpfad, `#include "src/UltrasonicManager.h"` löst deshalb auch nach
dem Umzug auf.

Skript: `umzugtest.ps1` im Scratchpad dieser Sitzung. Es liest nur
aus dem Repository.

Zur Einordnung der Zahlen: die Basiswerte liegen 22 beziehungsweise
10 Byte unter dem Visual-Micro-Ergebnis. Das ist die Abweichung aus
der Link-Reihenfolge, die schon der Kopf von
`tools/build_firmware.ps1` nennt.

## Was zu tun ist

Vier Projektdateien ändern sich, zwei Quelldateien wandern. Sonst
nichts.

### 1. Quelldateien verschieben

```
5_System/src/UltrasonicManager.h    ->  2_Hardware/src/UltrasonicManager.h
5_System/src/UltrasonicManager.cpp  ->  2_Hardware/src/UltrasonicManager.cpp
```

Der Inhalt bleibt **unverändert**. Die Kopien in diesem Ordner sind
byteweise identisch mit den heutigen Dateien; sie liegen nur bei,
damit die Vorlage vollständig ist.

Falls du in Git verschiebst: `git mv` erhält die Historie.

### 2. Projektdateien ersetzen

Aus diesem Ordner an ihren Platz:

- `2_Hardware/2_Hardware.vcxitems` — zwei Einträge dazu
- `2_Hardware/2_Hardware.vcxitems.filters` — zwei Einträge dazu
- `5_System/5_System.vcxitems` — zwei Einträge weg
- `5_System/5_System.vcxitems.filters` — zwei Einträge weg

Mehr steht in keiner der vier Dateien anders. Kodierung (UTF-8 mit
BOM) und Zeilenenden sind erhalten — `2_Hardware.vcxitems` benutzt
LF mit abschließendem Umbruch, die anderen drei CRLF ohne. Die
Einfügestellen folgen der jeweiligen Ordnung: die `.vcxitems` sind
alphabetisch sortiert, die `.filters` in Reihenfolge des Hinzufügens.

### 3. Beide Firmwares neu bauen und hochladen

Der Eingriff liegt in gemeinsamen Ordnern, also sind **beide**
Projekte betroffen — auch `vorne`, das den Manager für die
Seitensensoren benutzt.

`flash_beide.ps1` bricht ab, sobald eine Quelldatei neuer ist als
das jeweilige Hex. Nach diesem Umzug trifft das auf beide zu. Sauberer
ist es, in Visual Micro erst `vorne` und dann `hinten` einzeln zu
bauen und hochzuladen.

## Was ausdrücklich nicht geändert wird

- **Kein Include.** `FrontApp.h:18` und `RearApp.h:9` bleiben bei
  `#include "src/UltrasonicManager.h"`.
- **Kein Quelltext.** Auch `UltrasonicManager.h:22` bleibt bei
  `#include "src/UltrasonicEchoCapture.h"`, obwohl die Datei danach
  daneben liegt. Beide Schreibweisen gibt es in 2_Hardware ohnehin
  schon nebeneinander: `Hardware.h:15` schreibt
  `"src/HardwarePins.h"`, `Hardware.cpp:14` dieselbe Datei als
  `"HardwarePins.h"`. Es gibt also keine Regel, gegen die der Umzug
  verstoßen würde.
- **Keine `.vcxproj`.** Die drei Projektdateien importieren die
  `.vcxitems` und zählen keine Einzeldateien auf. Geprüft: keine
  von ihnen erwähnt `UltrasonicManager`.

## Zurückdrehen

Zwei Dateien zurückschieben, die vier Projektdateien aus Git
wiederherstellen, beide Firmwares neu bauen. Es gibt keinen Zustand,
der sich nicht rückgängig machen ließe, und keine Datenänderung am
Roboter.

## Danach

Die Zeilenverteilung der Schichten, ohne Leerzeilen gezählt:

| Schicht | vorher | nachher |
|---|---|---|
| 1_Common | 659 | 659 |
| 2_Hardware | 1.300 | 1.938 |
| 3_Control | 724 | 724 |
| 4_Vehicle | 380 | 380 |
| 5_System | 2.695 | 2.057 |

5_System bliebe die größte Schicht, wäre aber kein Ausreißer mehr,
sondern gleichauf mit 2_Hardware. Innerhalb von 2_Hardware macht
Ultraschall danach mit rund 1.000 von 1.938 Zeilen über die Hälfte
aus — kein Fehler, sondern der Umstand, dass nicht blockierendes
HC-SR04 mit PCINT-Erfassung und Medianfilter das aufwendigste Stück
Hardware am Roboter ist.
