# Funkintegration Nano vorne – Vorschlag für Claudine

## Ergebnis der Bestandsanalyse

- D7 (CE) und D8 (CSN) sind am vorderen Nano frei.
- D11 (MOSI), D12 (MISO) und D13 (SCK) sind frei und werden von Hardware-SPI benutzt.
- `Serial` ist zugleich PC-Schnittstelle und UART zum hinteren Nano. Funk darf daher nicht blind den gesamten seriellen Verkehr spiegeln, sondern nur die mit `#` beginnenden PC-Telemetriezeilen.
- Python verarbeitet derzeit `#CMDP_BEGIN`, `#ODOM`, `#WHEELS` und `#US`.
- Die Ausgabe ist noch nicht zentral: `#ODOM`/`#WHEELS` kommen aus `TelemetryPrinter`, `#CMDP_BEGIN` aus `CommandRunner`, `#US` aus `FrontApp`.
- Claudines vorhandene Dateien enthalten noch keinen Funkentwurf.

## Paketformat (exakt 32 Byte)

| Byte | Feld | Bedeutung |
|---:|---|---|
| 0 | version | Protokollversion 1 |
| 1–2 | messageId | fortlaufende Nachrichtennummer |
| 3 | fragmentIndex | Teilnummer, beginnend mit 0 |
| 4 | fragmentCount | Gesamtzahl Teile, 1 bis 6 |
| 5 | payloadLength | gültige Bytes im Nutzfeld, 1 bis 26 |
| 6–31 | payload | Ausschnitt der Telemetriezeile ohne Zeilenende |

Maximale Zeilenlänge: 130 Byte. Nach Claudines Prüfung wurde der Einzelslot durch einen 240-Byte-Ring ersetzt. Er nimmt auch den theoretisch maximalen Dreierstoß aus `#WHEELS`, `#CNTF` und `#ODOM` vollständig auf. Je Aufruf von `update()` wird weiterhin höchstens ein Fragment gesendet. Ist der Ring voll, werden ausschließlich vollständige neue Zeilen verworfen.

## Vorgesehener zentraler Übergabepunkt

Die beiliegende `TelemetryOutput`-Klasse ist von Arduino `Print` abgeleitet. Sie leitet jedes Byte unverändert an `Serial` weiter, sammelt ausschließlich vollständige `#`-Zeilen und ruft bei Zeilenende `RadioTelemetrySender::enqueueLine()` auf. Bei der Übernahme werden nur die vier Python-Ausgabeorte auf dieses Objekt umgestellt:

1. `TelemetryPrinter`: Ausgabeziel als `Print&` statt festem `Serial`.
2. `CommandRunner`: `#CMDP_BEGIN` über dasselbe Ausgabeziel.
3. `FrontApp::handleUltrasonicLine`: `#US` über dasselbe Ausgabeziel.
4. `FrontApp::update`: einmal `radioTelemetry.update()` pro Schleifendurchlauf.

UART-Protokollzeilen (`VSOL`, `VIST`, Handshake) bleiben direkt auf `Serial` und gelangen nicht in den Funk. Die bestehende USB-Ausgabe bleibt parallel erhalten.

## Hardwareparameter

`RF24 radio(7, 8)`, Adresse `IGOR1`, `RF24_PA_LOW`, `RF24_1MBPS`, Kanal 76. `setRetries(1, 2)` begrenzt einen fehlgeschlagenen `write()` bewusst; Claudine soll bewerten, ob für die reale Regeltaktreserve noch `0,0` nötig ist.

## Noch vor Übernahme zu prüfen

- RF24-`write()` ist trotz kurzer Retry-Einstellung synchron. Laufzeit im schlechtesten Funkfall messen.
- Vollständige Integration der vier Ausgabeorte erstellen und für `nano_atmega328old` bauen.
- Flash/SRAM vorher und nachher aus dem Buildbericht notieren.
- Paketverlust, falsche Reihenfolge, neue Nachricht während alter Nachricht und 130-Byte-Grenze testen.
- Empfänger benötigt denselben `RadioProtocol.h` im Sketchordner.

Die Dateien sind ein Review-Vorschlag. Es wurde nichts in den echten Projektordnern geändert und nichts hochgeladen.
