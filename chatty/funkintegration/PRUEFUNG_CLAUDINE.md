# Chattys Gegenprüfung und Endkorrektur

## Befunde B1 bis B11

| Befund | Urteil | Begründung |
|---|---|---|
| B1 | bestätigt, aber Mengenangabe korrigiert | Mehrere Ausgaben entstehen ohne `radioTelemetry.update()` dazwischen. Beim aktuellen Printerprofil sind es regelmäßig `#WHEELS` und `#ODOM`; `#CNTF` ist deaktiviert und `#US` kommt in einem eigenen UART-Durchlauf. Der Einzelslot verliert dennoch systematisch mindestens `#ODOM`. |
| B2 | bestätigt | Zwei Puffer mit je 156 Byte sind unnötig. Byteweises Schreiben in einen Ring ist besser. Claudines Ringgröße 208 ist jedoch für die von ihr geforderte Datentyp-Grenzrechnung zu klein; korrigiert auf 240 Byte. |
| B3 | bestätigt | Ohne `_atLineStart` kann ein späteres `#` in einer Nicht-Telemetriezeile die Aufnahme starten. |
| B4 | bestätigt | Ein verlorenes ACK kann ein bereits empfangenes Fragment duplizieren. Ein Duplikat derselben `messageId` mit kleinerem Index wird ignoriert. |
| B5 | bestätigt | Nach Einordnung in die Schichten müssen Includes das im Projekt übliche `src/`-Schema verwenden. Die flachen Includes in Chattys Reviewordner sind nur dort lokal gültig. |
| B6 | bestätigt | Vorne und hinten importieren die gemeinsamen Schichten. Vorwärtsdeklaration und ausschließlich in `FrontApp` konstruierte Funkobjekte verhindern Funkkosten hinten. |
| B7 | bestätigt | Die Bibliotheksincludes im Sketch sichern die Arduino-Library-Erkennung. |
| B8 | bestätigt | Startmeldung, periodische Sender-/Empfängerzähler sowie `maxSendMicros` sind sinnvoll. |
| B9 | teilweise bestätigt | 130 Byte reichen. Die genannten Längen der normalen Telemetrie waren aber keine Datentyp-Maxima. Nur die 114 Zeichen für `#EVENT,startCmdp` stimmen als Maximum. |
| B10 | bestätigt | Nullnutzlast und verkürzte Nicht-Endfragmente müssen abgewiesen werden. |
| B11 | bestätigt | `Print&` in `TelemetryPrinter` und `CommandRunner` erfasst auch Status- und Fehlerzeilen zentral, ohne Formatduplikate. |

## Unabhängige maximale Zeilenlängen

Die Rechnung enthält Präfix, alle Kommata und ein mögliches Minuszeichen, aber nicht `CR/LF`:

- `#WHEELS`: 102 Zeichen. Präfix 8, `uint32_t t` 10, zwölf `int16_t`-Felder je 6 sowie zwölf Trennkommata.
- `#CNTF`: 64 Zeichen. Präfix 6, Zeit 10, vier `int32_t` je 11 und vier Kommata.
- `#ODOM`: 70 Zeichen. Präfix 6, ID 5, Zeit 10, vier `long` je 11 und fünf Kommata.
- `#US`: 49 Zeichen. Präfix 4, sieben `uint16_t` je 5, ein `uint8_t` mit 3 und sieben Kommata.
- `#EVENT,startCmdp`: 114 Zeichen bei `int16_t`-Minima, `uint16_t`-Maximum, Einheitsvektorwerten `-1000` und zehnstelliger Dauer.

Damit ist `MAX_LINE_LENGTH = 130` korrekt. Für den theoretischen Dreierstoß aus WHEELS, CNTF und ODOM sind einschließlich dreier Längenbytes 239 Ringbytes nötig. Die Endkorrektur verwendet 240.

## Ringprüfung

- `(uint16_t)_dataPos + offset` und `(uint16_t)_dataPos + _lineLength` müssen vor dem Modulo mindestens 16 Bit breit sein; Claudines Klammerung ist richtig.
- `_committed` enthält auch die gerade gesendete Zeile bis `finishMessage()`. Der Schreiber kann sie daher nicht überschreiben.
- `rollbackLine()` setzt den Schreibzeiger auch nach Ring-Wrap korrekt auf das reservierte Längenbyte zurück.
- Ist nur noch Platz für das Längenbyte, wird der erste Nutzbyteversuch als Overflow erkannt und beim Commit die ganze Zeile zurückgerollt.
- Bei `RING_SIZE = 240` bleiben `_committed`, `_pending` und alle Positionen in `uint8_t` darstellbar. Eine Größe von 256 wäre mit diesen Typen falsch.

## Blockierdauer

Bei 1 MBit/s benötigt ein Datenpaket ungefähr 329 µs auf der Luft: 1 Byte Präambel, 5 Byte Adresse, 9 Bit Packet-Control-Field, 32 Byte Nutzlast und 2 Byte CRC. Ein ACK ohne Nutzlast benötigt etwa 73 µs. Mit ungefähr 130 µs RX/TX-Umschaltung ergibt das rund 532 µs pro erfolgreichen Versuch. Drei Versuche plus zweimal 500 µs Retry-Abstand liegen bei ungefähr 2,60 ms; SPI- und Bibliotheksaufwand bringen den realen Wert plausibel in die Größenordnung 2,7–2,8 ms. Claudines 2,8 ms sind damit eine brauchbare konservative Rechnung, aber kein garantierter Grenzwert. Maßgeblich bleibt `maxSendMicros()` am Fahrzeug.

## Übersehener Punkt und Entscheidung

Claudine hat beim Ring die beobachteten bzw. üblichen Zahlenwerte mit den geforderten Datentyp-Maxima vermischt. Dadurch war ihre Kapazitätsgarantie für einen vollständig aktivierten `WHEELS + CNTF + ODOM`-Stoß falsch. Die Korrektur auf 240 Byte erhöht den gemeldeten SRAM-Stand voraussichtlich von 1274 auf etwa 1306 Byte; rund 742 Byte bleiben für Stack und lokale Variablen. Das muss durch einen neuen Build bestätigt werden.

Ein fehlgeschlagenes Fragment bricht weiterhin die ganze Zeile ab. Das ist bewusst: Wiederholen außerhalb der RF24-Hardware-Retries würde die Regelung unbestimmt weiter blockieren.

Pinbelegung, Adresse `IGOR1`, `RF24_PA_LOW`, `RF24_1MBPS`, Kanal 76 und das 32-Byte-Paketformat bleiben unverändert.
