# Phase 4: Auftrag an Chatty

## Kurzfassung zum Einfügen ins Chatfenster

Diese Zeilen ins Chatfenster:

> Claudine hat deinen Funkentwurf geprüft und eine korrigierte Fassung
> geschrieben. Beides liegt im Projekt unter `claudine/funkintegration/`:
> `PRUEFBERICHT.md` mit elf Befunden und den Begründungen, `AN_CHATTY.md` mit
> dem vollständigen Prüfauftrag an dich, dazu die korrigierten Dateien.
>
> Bitte arbeite `AN_CHATTY.md` ab. Wichtig: nimm Claudines Befunde nicht als
> gegeben, sondern rechne sie nach. Ein pauschales Einverständnis hilft mir
> nicht — ich will wissen, wo du widersprichst und warum.
>
> Der schwerste Befund ist B1: der Sender verwirft jede Zeile, solange er
> beschäftigt ist, und weil `#WHEELS`, `#CNTF` und `#ODOM` in einem einzigen
> Schleifendurchlauf entstehen, kämen dauerhaft nur die `#WHEELS`-Zeilen an.
> Prüf das zuerst.
>
> Am gründlichsten nachrechnen sollst du die maximalen Zeilenlängen (davon
> hängt `MAX_LINE_LENGTH = 130` ab — ist der Wert zu klein, verschwinden
> Zeilen stillschweigend), die Ringpufferlogik in `RadioTelemetrySender.cpp`
> (in Python getestet, nicht in C++) und die gerechnete Blockierdauer von
> 2,8 ms gegen das nRF24L01+-Datenblatt.
>
> Nicht ändern ohne Begründung: Pinbelegung, Adresse `IGOR1`, `RF24_PA_LOW`,
> `RF24_1MBPS`, Kanal 76, Paketformat.
>
> Wenn du korrigierst: vollständige Dateien, keine Ausschnitte.

---

## Vollständiger Prüfauftrag

---

Claudine hat deinen Funkentwurf geprüft und eine korrigierte Fassung
geschrieben. Beides liegt in `claudine/funkintegration/`, die Begründungen
in `PRUEFBERICHT.md`.

Du hast Zugriff auf den Projektordner. Lies die genannten Stellen im
Originalquelltext nach, statt dich auf Claudines Zitate zu verlassen —
besonders `vorne/src/FrontApp.cpp`, `5_System/src/TelemetryPrinter.cpp` und
`5_System/src/CommandRunner/CommandRunner.cpp`. Die alten Fassungen dieser
Dateien stehen unverändert im Projekt, die geänderten daneben in
`claudine/funkintegration/`; ein Vergleich zeigt dir genau, was umgestellt
wurde.

**Nimm die Befunde nicht als gegeben.** Der Sinn dieses Ablaufs ist, dass wir
uns gegenseitig kontrollieren. Prüf sie nach, und widersprich, wo du es
anders siehst — mit Begründung, nicht mit einem allgemeinen Einverständnis.
Wenn du einen meiner Befunde für falsch hältst, sag es deutlich.

## Was du am Quelltext nachrechnen kannst

Diese Punkte lassen sich ohne Hardware entscheiden. Prüf sie selbst nach,
statt Claudines Zahlen zu übernehmen:

1. **Der Blocker (B1).** Die Behauptung ist: in einem einzigen Durchlauf von
   `FrontApp::handleIncomingLines()` entstehen `#WHEELS`, `#CNTF` und `#ODOM`
   direkt hintereinander, ohne dass dazwischen `update()` läuft. Damit belegt
   die erste Zeile den Sender und die übrigen fallen in den Drop-Zähler.
   Sieh dir den Block ab `rearFrameClient.handleVistLine(line)` an und
   entscheide, ob das stimmt.

2. **Zeilenlängen.** Claudine hat als längste Zeilen ermittelt: `#WHEELS` 61,
   `#CNTF` 43, `#US` 38, `#ODOM` 37 und `#EVENT,startCmdp` im Extremfall 114
   Zeichen. Daraus folgen zwei Festlegungen:
   - `MAX_LINE_LENGTH = 130` bei 5 Fragmenten
   - Ringgröße 208 Byte

   **Rechne beide Längen selbst nach**, Feld für Feld, mit den größtmöglichen
   Werten je Datentyp — `t_ms` kann zehnstellig werden, PWM-Werte sind
   vorzeichenbehaftet. Kommt bei dir eine Zeile über 130 heraus, wird sie im
   Betrieb stillschweigend verworfen. Das ist der Punkt, an dem ein
   Rechenfehler am teuersten wäre.

3. **Die Blockierdauer.** Claudine rechnet für `setRetries(1, 2)` rund 2,8 ms
   im schlechtesten Fall: drei Versuche à etwa 0,6 ms plus zweimal 500 µs
   Wartezeit. Das ist eine Rechnung, keine Messung. Prüf die Luftzeit gegen
   das nRF24L01+-Datenblatt: Präambel, Adressbreite, PCF, 32 Byte Nutzlast,
   CRC, ACK, Umschaltzeiten. Kommst du auf eine andere Größenordnung, sag es.

4. **Die Ringpufferlogik** in `RadioTelemetrySender.cpp`. Sie ist in Python
   nachgebaut und getestet worden, nicht in C++. Sieh dir gezielt an:
   - `(uint16_t)_dataPos + offset` vor dem Modulo — passt die Summe wirklich
     nicht mehr in `uint8_t`, und ist die Klammerung richtig?
   - `rollbackLine()`, wenn die Zeile beim Schreiben über das Ringende
     gelaufen ist
   - die Buchführung von `_committed`, während eine Nachricht gerade gesendet
     wird: wird der belegte Bereich zuverlässig gegen Überschreiben geschützt?
   - der Fall, dass `beginLine()` gar keinen Platz mehr für das Längenbyte
     findet

5. **Die Rückwirkung auf den hinteren Nano (B6).** `hinten.vcxproj` importiert
   dieselben `.vcxitems` wie `vorne.vcxproj`, jede Änderung an `5_System` wird
   also für hinten mitkompiliert. Claudine hat daraus zwei Regeln abgeleitet:
   kein globales Funk- oder `TelemetryOutput`-Objekt, und `RF24.h` bleibt per
   Vorwärtsdeklaration aus `RadioTelemetrySender.h` heraus. Prüf, ob die
   korrigierten Dateien diese Regeln durchgängig einhalten.

## Entscheidungen, bei denen du anderer Meinung sein darfst

Das sind keine Fehler, sondern Abwägungen. Wenn du sie anders triffst,
begründe es:

- **Ein fehlgeschlagenes Fragment bricht die ganze Zeile ab.** Alternative
  wäre, das Fragment zu wiederholen. Claudine gibt der Regelung Vorrang.
- **Alle `#`-Zeilen gehen in den Funk**, nicht nur die vier, die Python
  auswertet. Begründung: §3 nennt Statusmeldungen, und `#ERROR` will man
  während der Fahrt sehen. Kostet Funklast.
- **`setRetries(1, 2)` bleibt vorerst**, statt gleich auf `0, 0` oder
  `setAutoAck(false)` zu gehen. Begründung: erst messen, dann ändern. Dafür
  ist `maxSendMicros()` eingebaut und wird als letztes Feld von `#RTX`
  ausgegeben.
- **Ringgröße 208 statt zweier 156-Byte-Puffer.** Spart rund 100 Byte SRAM
  und löst zugleich den Blocker.

## Was du nicht nachprüfen kannst

Die Speicherzahlen stammen aus einem echten Build mit `arduino-cli` und
derselben Toolchain, die Visual Micro benutzt; der Ausgangsstand stimmt
bitgenau mit dem letzten Visual-Micro-Build überein.

- vorne: 22836 → 26420 Byte Flash, 955 → 1274 Byte SRAM
- hinten: unverändert 13368 / 706
- Empfänger: 4338 / 396

Flash bei 86 % ist der eigentliche Engpass. Wenn du Änderungen vorschlägst,
sag dazu, ob sie Flash kosten.

## Was du liefern sollst

1. Für jeden Befund B1 bis B11: bestätigt, widerlegt oder anders gelöst — mit
   Begründung.
2. Deine eigenen Zahlen zu Punkt 2 und 3 oben.
3. Fehler, die Claudine übersehen hat. Besonders in der Ringpufferlogik und
   in der Umstellung von `TelemetryPrinter` und `CommandRunner` auf `Print&`.
4. Falls du korrigierst: vollständige Dateien, keine Ausschnitte.

Nicht ändern ohne ausdrückliche Begründung: Pinbelegung, Adresse `IGOR1`,
`RF24_PA_LOW`, `RF24_1MBPS`, Kanal 76 und das Paketformat.

Der Empfänger ist bereits in die Projektmappe übernommen und liegt unter
`funk_empfaenger/`. Die Senderintegration ist noch nicht übernommen.
