# Speicherbedarf einer BNO055 am hinteren Nano

Stand: 11.08.2026
Ausgangspunkt: Commit `76e38ee`
**Gemessen, nicht geschätzt.** Am Projekt wurde nichts geändert,
es wurde nichts geflasht.

## Ergebnis

| Variante | Flash | | SRAM statisch | |
|---|---|---|---|---|
| hinten, aktueller Stand | 13.880 / 30.720 | 45,2 % | 711 / 2.048 | 34,7 % |
| hinten + BNO055 | 18.222 / 30.720 | 59,3 % | 972 / 2.048 | 47,5 % |
| **Differenz** | **+4.342** | | **+261** | |

Frei bleiben hinten 12.498 Byte Flash und 1.076 Byte SRAM.

Zum Vergleich der vordere Nano im heutigen Zustand, aus dem
Visual-Micro-Build vom 10.08.: 26.422 Byte Flash (86,0 %) und
1.306 Byte SRAM (63,8 %). Dort sind nur 4.298 Byte Flash frei.

## Was in den 4.342 Byte steckt

Übersetzt und gelinkt wurden zusätzlich:

- `Wire` (inklusive `utility/twi.c`)
- `Adafruit_Sensor`
- `Adafruit_BusIO` (`Adafruit_I2CDevice`, `Adafruit_BusIO_Register`,
  `Adafruit_SPIDevice`)
- `Adafruit_BNO055`

An Benutzung enthält der Messbau das, was eine echte Anbindung
mindestens braucht: Objekt anlegen, `begin()`, pro Durchlauf
`getVector(VECTOR_EULER)` und `getCalibration()`. Die Ergebnisse
gehen in `volatile`-Variablen, damit `--gc-sections` nichts
wegwirft.

Die Versionen sind die, die auf diesem Rechner unter
`OneDrive\Dokumente\Arduino\libraries` installiert sind.

## Wie gemessen wurde

Skript: `messbau.ps1` im Scratchpad dieser Sitzung
(`%LOCALAPPDATA%\Temp\claude\C--Eigene-Projekte-repos-robot\...`).
Es liest ausschließlich aus dem Repository und schreibt nur in den
Scratchpad.

Es übersetzt die hintere Firmware zweimal — einmal wie sie ist,
einmal mit `main_bno055.cpp` statt `main_baseline.cpp` — und
vergleicht `.text + .data` gegen `.data + .bss`.

Grenzwerte: 30.720 Byte Flash und 2.048 Byte SRAM, entsprechend
`nano_atmega328old` aus der `boards.txt`.

### Warum nicht `tools/build_firmware.ps1`

Das Skript im Projekt bricht seit dem Funkeinbau ab: ihm fehlt der
Include-Pfad zur RF24-Bibliothek, und die Bibliotheksquellen holt es
aus dem Visual-Micro-Baum, wo nur `.o`-Dateien liegen, die es vorher
selbst löscht. Das ist ein eigener kleiner Reparaturpunkt, unabhängig
von der IMU.

### Gegenprobe

Der Basisbau ergibt 13.880 Byte Flash gegen 13.890 Byte der
Visual-Micro-ELF vom 11.08. 14:25, der SRAM-Wert stimmt exakt. Die
Abweichung von 10 Byte liegt im Rahmen der 20 Byte, die schon der
Kopf von `tools/build_firmware.ps1` als Effekt der Link-Reihenfolge
nennt.

## Was die Zahl nicht enthält

- **Den Stack.** Beide Werte sind statischer Verbrauch.
  `getVector()` rechnet mit Fließkomma und braucht Stack, der in
  keiner dieser Zahlen auftaucht. Hinten unkritisch, vorne wäre es
  zu beobachten.
- **Die vordere Seite.** Wenn der Winkel zur Odometrie nach vorne
  muss, wächst auch die vordere Firmware um eine Protokollzeile
  samt Parser. Nicht gemessen, weil es die Erweiterung noch nicht
  gibt. Größenordnung einige hundert Byte — Schätzung.
- **Laufzeit.** `Wire` blockiert; ein Lesevorgang von 14 Byte bei
  100 kHz dauert gut 1 ms. Ob das den 20-ms-Regeltakt hinten stört,
  ist offen und müsste am Aufbau gemessen werden.
