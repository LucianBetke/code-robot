# Testergebnis: erste Funkfahrt

10. August 2026. Roboter aufgebockt, Fahrbefehl `#CMDP_BEGIN,1,30,0,0,100` —
100 cm bei 30 cm/s. Beide Monitore gleichzeitig mitgeschnitten, COM6 über USB
am vorderen Nano, COM7 am Funkempfänger.

---

## 1. Vollständigkeit

Auf COM6 sind während des Laufs **123** Telemetriezeilen entstanden, die über
Funk gehen sollen:

| Zeilentyp | erzeugt | per Funk angekommen |
|---|---:|---:|
| `#INFO,Radio,ok` | 1 | 1 |
| `#CMDP_BEGIN` | 1 | 1 |
| `#WHEELS` | 43 | 43 |
| `#ODOM` | 43 | **42** |
| `#US` | 33 | 33 |
| `#RTX` | 2 | 2 |
| **Summe** | **123** | **122** |

**Genau eine Zeile ist verloren gegangen:** `#ODOM,1,1360,3923,3923,-11,13`.
Das sind 0,8 %.

Der Empfänger meldet unabhängig davon dasselbe: `#RSTAT,229,122,0,0,1` —
122 vollständige Nachrichten, eine unvollständige. Die beiden Zählungen
stimmen überein.

Die `#WHEELS`-Zeitstempel laufen lückenlos von 0 bis 3360 in 80-ms-Schritten,
die `#US`-Folgenummern sind auf beiden Seiten identisch. Die Lücken darin
(6, 11, 16, 21, 26, 31, 35, 41) stehen schon auf COM6 und stammen vom hinteren
Nano, nicht vom Funk.

**Kein einziger Wert weicht ab.** Stichproben über den ganzen Lauf: die
Zeilen sind auf beiden Monitoren zeichengleich. Verworfene Pakete am
Empfänger: 0. Duplikate: 0.

## 2. Warum der Sendezähler pessimistisch ist

Der Sender meldet `#RTX,117,5,223,5,112,3108`, also **5 verworfene Zeilen**.
Verloren ist aber nur **eine**. Der Unterschied ist erklärbar und kein Fehler:

`RF24::write()` liefert `false`, wenn keine Bestätigung eintrifft. Das heißt
aber nicht, dass die Daten nicht angekommen sind — es kann auch nur das ACK
verloren gegangen sein. Trifft das das **letzte** Fragment einer Zeile, dann
hat der Empfänger die Zeile bereits vollständig, während der Sender sie als
gescheitert zählt.

Von 5 Fehlschlägen betrafen also vier das jeweils letzte Fragment, einer ein
früheres. Genau dieser eine erzeugt beim Empfänger die eine unvollständige
Nachricht und die eine fehlende Zeile.

**Für die Beurteilung heißt das:** `droppedLines` und `failedPackets` im
`#RTX` sind eine Obergrenze, nicht der tatsächliche Verlust. Maßgeblich ist
`incompleteMessages` im `#RSTAT` des Empfängers.

Rückblickend gilt das auch für den ersten Lauf: dort meldete der Sender 7
verworfene Zeilen, der Empfänger aber nur 2 unvollständige Nachrichten.

## 3. Blockierdauer

`maxSendUs` = **3108** im zweiten Lauf, 3104 im Zwischenstand, 3140 im ersten
Lauf. Der Wert ist stabil bei rund **3,1 ms**.

Meine Rechnung lag bei 2,8 ms und ich hatte sie „konservativ" genannt. Sie
war es nicht — der gemessene Wert liegt darüber. Der Fehler steckt
wahrscheinlich im Aufwand der Bibliothek und im SPI-Verkehr, die ich mit
„vernachlässigbar" angesetzt hatte.

Auf dem 20-ms-Raster des Radreglers sind 3,1 ms rund 16 % Jitter. Die
Regelung hat das im Stand weggesteckt, siehe unten.

## 4. Ringpuffer

`ringPeak` = **112** von 240, in beiden Läufen identisch. Kein einziger
Verlust durch Ringüberlauf; alle Verluste kamen vom Funk.

Die Reserve ist damit reichlich. Chattys 240 sind gegen den rechnerischen
Extremfall bemessen, im Betrieb werden knapp die Hälfte davon gebraucht.
Wenn später Flash oder SRAM knapp werden, ist das eine Stellschraube — aber
erst, wenn `PRINTER_ENABLE_COUNTS` weiterhin aus bleibt.

## 5. Regelung

Ziel 100 cm, gefahren 99,32 cm laut `#ODOM`. Querablage am Ende 0,10 cm,
Drehung 0,19°. Die Radgeschwindigkeiten liegen über den ganzen Lauf bei
29 bis 33 cm/s bei Sollwert 30.

**Das ist aufgebockt gemessen, ohne Last.** Ob die Regelung unter Last
unruhiger wird, ist damit nicht beantwortet.

## 6. Zwei Beobachtungen

**Verbindungsmeldungen gehen nicht über Funk.** `#WAIT`, `#CON`, `#HS1` und
`#DIS` stehen nur auf COM6. Sie entstehen in `ConnectionMonitor`, das direkt
auf `Serial` schreibt und nicht auf `TelemetryOutput`. Das ist erklärbar,
aber §3 der Arbeitsanweisung nennt Statusmeldungen ausdrücklich. Behebbar wie
bei `TelemetryPrinter`: ein `Print&` statt `Serial`.

**Der Abbruch am Ende des Laufs ist erklärt.** Auf COM6 steht nach den letzten
`KA`-Zeilen ein `#DIS`, danach `#WAIT` und wieder `PING` — der vordere Nano
hat über den Watchdog neu gestartet. Ursache war kein Fehler: der Schalter
zwischen den Nanos wurde am Ende des Laufs von Hand geöffnet. Das Verhalten
ist damit das erwartete.

## 7. Bewertung gegen die Abnahmekriterien

| §11 | Stand |
|---|---|
| Nano vorne startet normal | erfüllt |
| Kommunikation mit hinten funktioniert | erfüllt, siehe aber §6 |
| Motorregelung unverändert | im Stand erfüllt, unter Last offen |
| serielle Telemetrie weiterhin an Python | erfüllt |
| dieselben Daten zusätzlich per Funk | erfüllt, 122 von 123 Zeilen |
| Empfänger gibt vollständige Zeilen aus | erfüllt |
| lange Nachrichten korrekt zerlegt | erfüllt, `#WHEELS` braucht 4 Fragmente |
| Verluste erzeugen keine falschen Zeilen | erfüllt, 0 verfälschte Zeilen |
| keine Speicherüberläufe | kein Hinweis darauf |
| kompiliert für alten Bootloader | erfüllt |

Offen bleibt allein der Test unter Last.
