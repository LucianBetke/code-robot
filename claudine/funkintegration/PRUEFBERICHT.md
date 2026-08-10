# Prüfbericht Funkintegration — Claudine zu Chattys Vorschlag

Grundlage: `chatty/funkintegration/` (Stand 9. August 2026) gegen den
Projektstand auf Branch `rettung1`, Commit `daea8d5`.

Die korrigierten Dateien liegen unter `claudine/funkintegration/` in der
Ordnerstruktur ihres Zielorts. Es wurde nichts hochgeladen.

**Nachtrag vom selben Tag:** Auf ausdrücklichen Wunsch ist der Empfänger
bereits als eigenes Projekt in die Projektmappe übernommen — siehe §8. Die
Senderintegration wartet weiterhin auf „Endfassung übernehmen".

---

## 1. Was an Chattys Vorschlag richtig ist

Damit klar ist, was ich nicht angefasst habe:

- **Pinbelegung stimmt.** D7, D8, D11, D12 und D13 sind am vorderen Nano frei.
  Belegt sind laut `2_Hardware/src/HardwarePins.h`: D2, D3 (US-Trigger), D4
  (Sync), D5, D6, D9, D10 (Motoren), A0–A3 (Encoder), A4, A5 (US-Echo auf
  PORTC wegen PCINT1). Kein Konflikt mit SPI.
- **Paketformat ist gut.** 32 Byte exakt, `static_assert` darauf, Nachrichten-
  nummer plus Fragmentindex plus Fragmentzahl plus Längenangabe. Ich habe es
  unverändert übernommen.
- **Die Analyse des Telemetriepfads ist korrekt.** `Serial` ist am vorderen
  Nano gleichzeitig PC-Schnittstelle und UART zum hinteren Nano; nur `#`-Zeilen
  dürfen in den Funk. VSOL, VIST und Handshake bleiben direkt auf `Serial`.
- **Der Ansatz mit einem `Print`-Ableger als zentralem Übergabepunkt ist der
  richtige.** Er hält die Formatierung an einer Stelle, wie §6 der
  Arbeitsanweisung verlangt.
- **Ein Fragment je `update()`** ist die richtige Entscheidung gegen lange
  Blockaden.

---

## 2. Befunde

### B1 — Blocker: es käme dauerhaft nur jede vierte Telemetriezeile an

`RadioTelemetrySender::enqueueLine()` verwirft jede Zeile, solange `_busy`
gesetzt ist. Der vordere Nano gibt seine Zeilen aber in Stößen aus: in einem
einzigen Durchlauf von `FrontApp::handleIncomingLines()` entstehen
nacheinander `#WHEELS`, `#CNTF` und `#ODOM`
([FrontApp.cpp:201–224](../../vorne/src/FrontApp.cpp#L201)), `#US` kommt in
einem anderen Durchlauf dazu.

Die erste Zeile eines Stoßes belegt den Sender, alle folgenden fallen sofort
in den Drop-Zähler. Der Verlust ist damit **nicht zufällig, sondern immer
derselbe Zeilentyp**.

Simuliert mit der nachgebauten Logik, 50 Frames zu je vier Zeilen:

```
Einzelslot nimmt an: {'#WHEELS...': 50}   verworfen: 150
```

`#CNTF`, `#ODOM` und `#US` kämen also **nie** beim Empfänger an. Damit ist
Abnahmekriterium „dieselben Daten zusätzlich per Funk" nicht erfüllt.

**Korrektur:** Ringpuffer statt Einzelslot. Der Ring nimmt den Stoß eines
Schleifendurchlaufs auf und wird zwischen zwei Frames wieder leergefahren.

### B2 — Zwei Puffer für dieselbe Zeile, 312 Byte SRAM

`TelemetryOutput::_line[156]` und `RadioTelemetrySender::_line[156]` halten
nacheinander denselben Inhalt. Auf 2 KB SRAM sind zwei Kopien einer Zeile
nicht zu rechtfertigen; §6 verlangt ausdrücklich „keine unnötigen Kopien".

**Korrektur:** `TelemetryOutput` schreibt byteweise direkt in den Ring des
Senders und hält selbst keinen Puffer mehr. Der Ring ist zugleich die Lösung
für B1.

Ringgröße 208 Byte. Die Rechnung mit den tatsächlichen Zeilenlängen (aus dem
Format der vorhandenen Ausgaben ermittelt):

| Zeile | Länge |
|---|---:|
| `#WHEELS` | 61 |
| `#CNTF` | 43 |
| `#US` | 38 |
| `#ODOM` | 37 |
| Summe eines Frames plus vier Längenbytes | **183** |

208 fasst also einen kompletten Frame-Stoß, auch wenn währenddessen gar nicht
gesendet wird.

### B3 — Ein `#` mitten in einer Zeile startet die Aufzeichnung

In `TelemetryOutput::write()` lautet die Bedingung `if (_length == 0 && value
== '#')`. `_length` wächst nur, wenn bereits aufgezeichnet wird. Während einer
nicht aufgezeichneten Zeile — also jeder VSOL-, VIST- oder Handshake-Zeile —
bleibt `_length` deshalb den ganzen Zeilenverlauf über 0. Ein `#` an
beliebiger Stelle einer solchen Zeile startet die Aufzeichnung mitten drin.

Heute enthält keine UART-Zeile ein `#`, der Fehler ist also latent. Er wird
scharf, sobald das UART-Protokoll erweitert wird.

**Korrektur:** eigenes Flag `_atLineStart`, das nur beim ersten Zeichen einer
Zeile ausgewertet wird.

### B4 — Empfänger verwirft die halbe Nachricht bei wiederholten Paketen

Mit Auto-Ack wiederholt der Sender ein Paket, wenn das ACK verloren geht. Der
Empfänger sieht dasselbe Fragment dann zweimal. Chattys Prüfung
`packet.fragmentIndex != expectedFragment` schlägt zu und ruft
`discardAssembly()` — die bereits korrekt empfangenen Fragmente werden
weggeworfen.

Simuliert, 200 Zeilen:

| Duplikatrate | Chattys Empfänger | korrigiert |
|---:|---|---|
| 0 % | 200 Zeilen, 0 unvollständig | 200 Zeilen, 0 unvollständig |
| 10 % | 198 Zeilen, 23 unvollständig | 200 Zeilen, 0 unvollständig |
| 30 % | 187 Zeilen, 77 unvollständig | 200 Zeilen, 0 unvollständig |

Der Effekt ist real, aber moderat: die meisten Zeilen brauchen nur zwei
Fragmente, und ein wiederholtes Fragment 0 löst ohnehin einen sauberen
Neustart aus. Kein Blocker, aber leicht zu beheben.

**Korrektur:** `isDuplicate()` — ein Fragment mit passender `messageId` und
`fragmentIndex < expectedFragment` wird still gezählt und übergangen.

### B5 — Die Includes würden im Projekt nicht übersetzen

Das Projekt bindet schichtübergreifend über das Präfix `src/` ein, weil die
Schichtordner selbst im Include-Pfad stehen (`vorne.vcxproj`, `IncludePath`):

```cpp
#include "src/RobotConfig.h"        // aus 1_Common, so macht es TelemetryPrinter.h
```

Chatty schreibt `#include "RadioProtocol.h"` und `#include "Nrf24Radio.h"`.
Das funktioniert nur, solange alle Dateien in einem Ordner liegen — also
genau nicht nach der Übernahme in die Schichten.

**Korrektur:** `"src/RadioProtocol.h"`, `"src/Nrf24Radio.h"` usw.

### B6 — Die Schichten 1–5 gehören auch dem hinteren Nano

Das ist im Vorschlag gar nicht erwähnt, ist aber die riskanteste Stelle der
ganzen Übernahme: `hinten.vcxproj` importiert dieselben fünf `.vcxitems` wie
`vorne.vcxproj`, und der Build-Ordner von `hinten` enthält tatsächlich
`CommandRunner`, `TelemetryPrinter` und alles Übrige. **Jede Änderung an
`5_System` wird für den hinteren Nano mitkompiliert.**

Daraus folgen zwei Regeln, die ich in der korrigierten Fassung eingehalten
habe:

1. **Kein globales Funk- oder `TelemetryOutput`-Objekt.** Ein Global würde im
   hinteren Build mitkonstruiert und zöge RF24, SPI und den Ring dort in
   Flash und SRAM. Ausgabeziele werden stattdessen als `Print&` übergeben.
2. **`RadioTelemetrySender.h` bindet `RF24.h` nicht ein**, sondern deklariert
   `class Nrf24Radio;` vorwärts. Sonst landet RF24 in jeder Übersetzungs-
   einheit, die nur Telemetrie ausgibt.

Gemessen (siehe §4): der hintere Nano bleibt bitgenau unverändert.

### B7 — `vorne.ino` muss SPI und RF24 einbinden

Der Arduino-Build findet Bibliotheken über die Includes des Sketches. Liegt
`#include <RF24.h>` nur in `2_Hardware/src/Nrf24Radio.cpp`, ist das je nach
Builder nicht genug.

**Korrektur:** `#include <SPI.h>` und `#include <RF24.h>` in `vorne.ino`.

### B8 — Fehlerfälle aus §8 sind angelegt, aber nirgends sichtbar

- Der Sender meldet nicht, wenn das Modul beim Start nicht erkannt wird.
- Die Zähler in `Nrf24Radio` und `RadioTelemetrySender` werden nie ausgegeben.
- Der Empfänger zählt, gibt aber nichts aus; nach einem fehlgeschlagenen
  `radio.begin()` bleibt er dauerhaft stumm.

**Korrektur:**

- `#INFO,Radio,ok` bzw. `#INFO,Radio,fehlt` beim Start.
- alle 5 s eine Statuszeile vorne:
  `#RTX,sentLines,droppedLines,sentPackets,failedPackets,ringPeak,maxSendUs`
- alle 5 s eine Statuszeile am Empfänger:
  `#RSTAT,received,complete,dropped,duplicates,incomplete`
- Der Empfänger wiederholt die Fehlermeldung, statt stumm zu bleiben.

### B9 — 155 Byte Zeilenlänge sind großzügiger als nötig

Die längste Zeile, die die Firmware erzeugen kann, ist `#EVENT,startCmdp,…`
mit 114 Zeichen im Extremfall (alle Felder am Anschlag). Alle Zeilen, die
Python auswertet, liegen unter 62.

**Korrektur:** `MAX_LINE_LENGTH = 130` bei `MAX_FRAGMENT_COUNT = 5`. Das
deckt jede erzeugbare Zeile und begrenzt zugleich die Sendedauer einer
einzelnen Zeile.

### B10 — Zwei Plausibilitätsprüfungen fehlen im Empfänger

`payloadLength == 0` wird nicht abgefangen, und es wird nicht geprüft, dass
alle Fragmente außer dem letzten voll sind. Ohne diese Prüfung kann ein
gestörtes Paket eine falsch zusammengesetzte Zeile erzeugen, statt verworfen
zu werden.

### B11 — Umfang: nur vier Ausgabestellen ist zu wenig

Chatty stellt nur die vier von Python ausgewerteten Ausgaben um. Die Firmware
erzeugt aber auch `#INFO`, `#EVENT`, `#ERROR`, `#CNTF` und `#CHASSISDBG`.
§3 der Arbeitsanweisung nennt Statusmeldungen ausdrücklich, und gerade
`#ERROR` will man per Funk sehen, wenn der Roboter fährt.

**Korrektur:** `TelemetryPrinter` und `CommandRunner` bekommen beide ein
`Print&`-Ausgabeziel; damit gehen alle `#`-Zeilen denselben Weg. Die Auswahl
trifft weiterhin allein der `#`-Filter in `TelemetryOutput`.

---

## 3. Offene Frage aus dem Vorschlag: `setRetries(1, 2)` oder `0, 0`?

Chatty bittet um eine Bewertung. Meine Antwort, ausdrücklich als **Rechnung,
nicht als Messung**:

Ein 32-Byte-Paket bei 1 MBit/s dauert mit Präambel, Adresse, PCF und CRC rund
0,33 ms, dazu ACK und Umschaltzeiten rund 0,25 ms — grob 0,6 ms je Versuch.
Stufe 1 bedeutet 500 µs Wartezeit vor der Wiederholung. Bei drei Versuchen:

```
3 × 0,6 ms + 2 × 0,5 ms  ≈  2,8 ms
```

Das ist die schlechteste Blockierdauer eines `write()`. Zum Vergleich: eine
61 Zeichen lange `#WHEELS`-Zeile belegt `Serial` bei 115200 Baud bereits rund
5,3 ms, wovon der `Serial.print`-Aufruf blockiert, sobald der 64-Byte-
Sendepuffer voll ist. Die Größenordnung ist also nicht neu für dieses
Programm.

**Ich empfehle, `1, 2` zunächst zu lassen** — es ist der von der
Arbeitsanweisung vorgegebene Wert, und ohne Messung würde ich ihn nicht
ändern. Stattdessen habe ich den Messhaken eingebaut: `Nrf24Radio` misst die
längste `write()`-Dauer mit `micros()` und gibt sie als letztes Feld von
`#RTX` aus.

Entscheidungsregel für den Test am Fahrzeug:

- `maxSendUs` deutlich unter 3000 und `#WHEELS` kommt sauber → so lassen.
- `maxSendUs` regelmäßig am Anschlag oder die Regelung wird unruhig →
  `setRetries(0, 0)`. Achtung: dann liefert `write()` bei jedem verlorenen
  Paket `false`, und die aktuelle Logik bricht die ganze Zeile ab. In dem
  Fall ist `setAutoAck(false)` die bessere Wahl — konstante, kurze Sendezeit
  ohne Wiederholungen, dafür ohne Sendebestätigung.

Das ist genau der Test, der zwischen beiden Erklärungen entscheidet. Ich würde
ihn vor jeder Parameteränderung fahren.

---

## 4. Kompilierergebnis und Speicher

Gebaut mit `arduino-cli` (aus der Arduino-IDE-Installation), FQBN
`arduino:avr:nano:cpu=atmega328old`, RF24 1.6.1 — dieselbe Toolchain, die
Visual Micro benutzt. Der Baseline-Build stimmt bitgenau mit dem letzten
Visual-Micro-Build überein (22836 / 955), die Zahlen sind also vergleichbar.

**Nano vorne**

| | Flash | | SRAM | |
|---|---:|---:|---:|---:|
| vorher | 22836 | 74 % | 955 | 46 % |
| nachher | 26420 | 86 % | 1274 | 62 % |
| Zuwachs | **+3584** | | **+319** | |

Für Stack und lokale Variablen bleiben 774 Byte.

**Nano hinten** — unverändert, obwohl die geänderten Dateien mitkompiliert
werden:

| | Flash | | SRAM | |
|---|---:|---:|---:|---:|
| vorher | 13368 | 43 % | 706 | 34 % |
| nachher | 13368 | 43 % | 706 | 34 % |

Der Linker räumt den ungenutzten Funkcode restlos weg. Das gilt aber nur,
solange die Regeln aus B6 eingehalten werden.

**Empfänger:** 4338 Byte Flash (14 %), 396 Byte SRAM (19 %).

Zur Einordnung: Chattys Fassung hätte durch die doppelte Zeilenpufferung rund
100 Byte SRAM mehr gebraucht (**gerechnet**: 156 + 156 statt 208 Ring, jeweils
plus Zustand). Gebaut habe ich das nicht, weil dazu die Integration in
`FrontApp` gefehlt hätte.

Flash bei 86 % ist der eigentliche Engpass. Für weitere Ausbaustufen — IMU,
Rückkanal — bleiben knapp 4 KB.

---

## 5. Prüfung der Ringlogik

Kein C++-Hostcompiler auf diesem Rechner, deshalb habe ich Sender- und
Empfängerlogik 1:1 nach Python portiert und getestet. **Das prüft die
Index- und Wraparound-Rechnung, nicht das kompilierte Binary.**

| Test | Ergebnis |
|---|---|
| 200 Zeilen, vier je Frame, genug Sendezeit | alle 200 identisch empfangen, ringPeak 62 |
| 400 Zeilen zufälliger Länge, Wraparound erzwungen | alle identisch, ringPeak 131 |
| Zeile über 130 Byte | sauber verworfen, Ring bleibt heil, Folgezeile korrekt |
| Ring läuft voll (Sender steht) | 3 von 30 Zeilen angenommen, alle 3 korrekt — es fallen nur ganze Zeilen weg |
| 15 % Paketverlust | 141 von 200 Zeilen, **keine verfälschte Zeile** |
| 30 % Duplikate | 200 von 200 Zeilen korrekt |

Der vierte Fall ist der wichtige: bei Überlauf gehen ganze Zeilen verloren,
nie Bruchstücke. Es entsteht keine falsch zusammengesetzte Zeile.

Testskript: `ringtest.py`, `duptest.py` im Arbeitsverzeichnis dieser Sitzung.
Sag Bescheid, wenn du sie im Repo haben willst.

---

## 6. Was noch offen ist

Alles Folgende ist **nicht gemessen** und braucht Hardware:

1. **Tatsächliche Blockierdauer.** `#RTX`, letztes Feld. Erst im Stand, dann
   mit laufender Regelung.
2. **Ob die Regelung ruhig bleibt.** Erst aufgebockt, dann langsame Fahrt.
   Ohne Vorher-Aufzeichnung derselben Fahrt ist das nicht beurteilbar.
3. **Ringhöchststand am Fahrzeug.** `#RTX`, fünftes Feld. Bleibt er deutlich
   unter 208, kann der Ring später schrumpfen und Flash für den Rückkanal
   freigeben.
4. **Reichweite und Verlustrate** mit `RF24_PA_LOW`.
5. **Ob D13 stört.** Die Bord-LED des Nano hängt am SPI-Takt. Das ist eine
   Vermutung als Fehlerquelle, kein beobachtetes Problem — aber der erste
   Verdächtige, wenn die Übertragung unzuverlässig ist.

---

## 7. Was bei der Übernahme zu tun ist

Nur zur Information — ich fasse die echten Projektdateien erst an, wenn du
„Endfassung übernehmen" sagst.

| Datei aus `claudine/funkintegration/` | Ziel |
|---|---|
| `1_Common/src/RadioProtocol.h` | neu |
| `2_Hardware/src/Nrf24Radio.{h,cpp}` | neu |
| `5_System/src/RadioTelemetrySender.{h,cpp}` | neu |
| `5_System/src/TelemetryOutput.{h,cpp}` | neu |
| `5_System/src/TelemetryPrinter.{h,cpp}` | ersetzt |
| `5_System/src/CommandRunner/CommandRunner.{h,cpp}` | ersetzt |
| `vorne/src/{vorne.ino,FrontApp.h,FrontApp.cpp}` | ersetzt |

`1_Common/src/RadioProtocol.h` und der Empfänger sind bereits übernommen,
siehe §8.

Dazu:

- Die neuen Dateien in `2_Hardware.vcxitems` und `5_System.vcxitems`
  eintragen, sonst sieht Visual Studio sie nicht.
- Bau in Visual Micro gegenprüfen. Die Zahlen oben stammen aus `arduino-cli`
  mit derselben Toolchain, aber Visual Micro setzt eigene Bibliothekspfade.

---

## 8. Nachtrag: Empfänger ist in der Projektmappe

Angelegt:

| Datei | |
|---|---|
| `funk_empfaenger/funk_empfaenger.vcxproj` | neues Visual-Micro-Projekt, nach dem Muster von `hinten` |
| `funk_empfaenger/funk_empfaenger.vcxproj.filters` | |
| `funk_empfaenger/src/funk_empfaenger.ino` | |
| `funk_empfaenger/src/arduino folders read me.txt` | Kopie wie in `vorne` und `hinten` |
| `1_Common/src/RadioProtocol.h` | neue Datei, noch von nichts eingebunden |

Geändert, beides rein additiv:

| Datei | Änderung |
|---|---|
| `robot.sln` | Projekteintrag, Konfigurationszeilen, `SharedItemsImports` für 1_Common |
| `1_Common.vcxitems` und `.filters` | Eintrag für `RadioProtocol.h` |

Das Empfängerprojekt importiert **nur** `1_Common.vcxitems`, nicht die
Schichten 2 bis 5. Es benutzt damit dieselbe `RadioProtocol.h` wie der
Sender — die Kopie im Sketchordner, die ich in der ersten Fassung noch als
unvermeidbaren Wartungsfall beschrieben hatte, entfällt. Der Include lautet
jetzt `#include "src/RadioProtocol.h"`, wie überall sonst im Projekt.

`CommProtocol.cpp` und `ScaleUtils.cpp` aus 1_Common werden dabei
mitkompiliert, aber vom Linker entfernt: der Empfänger ist mit und ohne
1_Common gleich groß (4338 / 396).

**Nachgeprüft, dass sich für die Firmware nichts ändert:**

| | Flash | SRAM |
|---|---:|---:|
| vorne, Projektstand jetzt | 22836 | 955 |
| hinten, Projektstand jetzt | 13368 | 706 |

Beide bitgenau wie vorher. Ein Header, den niemand einbindet, kostet nichts.

Noch zu tun in Visual Studio: Projektmappe neu laden, den Empfänger auf
`Nano (Old Bootloader)` und den richtigen COM-Port stellen. Diese Einstellung
schreibt Visual Micro selbst in die `.vcxproj`, ich habe sie bewusst leer
gelassen.
