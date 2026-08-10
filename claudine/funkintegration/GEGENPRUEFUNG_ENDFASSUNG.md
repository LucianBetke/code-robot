# Gegenprüfung von Chattys Endfassung

Grundlage: `chatty/funkintegration/PRUEFUNG_CLAUDINE.md` und
`chatty/funkintegration/endfassung/`.

Es wurden keine Sender-Projektdateien geändert und nichts hochgeladen.

---

## 1. Was Chatty geändert hat

Ein Diff der Endfassung gegen meine Fassung ergibt genau **eine**
substanzielle Änderung:

```
RadioTelemetrySender.h:  RING_SIZE  208 → 240
```

Alle übrigen vierzehn Dateien sind byteidentisch, bis auf angehängte
Zeilenumbrüche am Dateiende. Chatty hat also geprüft und an einer Stelle
korrigiert, nicht umgeschrieben.

## 2. Die Korrektur ist berechtigt

Chattys Vorwurf trifft zu: ich habe die Ringgröße aus **beobachteten**
Zeilenlängen abgeleitet (`#WHEELS` 61, `#CNTF` 43, `#ODOM` 37), während ich
von Chatty ausdrücklich die **Datentyp-Maxima** verlangt habe. Das war
inkonsequent, und die 208 Byte waren damit als Kapazitätszusage falsch
begründet.

Chattys Zahlen habe ich gegen den Ausgabecode nachgerechnet:

| Zeile | Chatty | nachgerechnet | Herleitung |
|---|---:|---:|---|
| `#WHEELS` | 102 | **102** | Präfix 8 + `uint32_t` 10 + 12 × `int16_t` à 6 + 12 Kommata |
| `#CNTF` | 64 | **64** | Präfix 6 + 10 + 4 × `int32_t` à 11 + 4 Kommata |
| `#ODOM` | 70 | **70** | Präfix 6 + ID 5 + Zeit 10 + 4 × `long` à 11 + 5 Kommata |
| `#US` | 49 | **49** | Präfix 4 + 7 × `uint16_t` à 5 + `uint8_t` 3 + 7 Kommata |
| `#EVENT,startCmdp` | 114 | **114** | wie im ersten Bericht |

Entscheidend für `#ODOM` und `#CNTF`: `scaleFloatToInt100()` liefert laut
`1_Common/src/ScaleUtils.h` ein `long`, also 32 Bit, damit bis zu 11 Zeichen.
Das hatte ich unterschätzt.

Dreierstoß `#WHEELS + #CNTF + #ODOM` = 236 Byte plus drei Längenbytes = **239**.
240 ist damit korrekt hergeleitet.

Auch richtig: 240 ist praktisch das Maximum. Alle Ringpositionen und die
Zähler `_committed` und `_pending` sind `uint8_t`; 256 wäre falsch.

Chatty hat außerdem meine Formulierung zu B1 zu Recht geschärft. Mit der
aktuellen Konfiguration ist `PRINTER_ENABLE_COUNTS = 0`, `#CNTF` entsteht also
gar nicht, und `#US` kommt in einem eigenen Durchlauf, weil
`handleIncomingLines()` pro Aufruf nur eine Zeile verarbeitet. Der Stoß ist
heute `#WHEELS + #ODOM`. Am Befund ändert das nichts — der Einzelslot verliert
weiterhin systematisch `#ODOM`.

## 3. Wo Chattys Begründung unvollständig ist

Chatty schreibt, bei 240 bleibe „ein Byte Reserve frei". Das gilt nur für den
Dreierstoß aus dem Frameabschluss. Es gibt aber zwei weitere Zeilen, die im
**selben** Schleifendurchlauf entstehen können:

- `#CMDP_BEGIN` (44 Byte) aus `updateCommandRunner()`, das direkt nach
  `handleIncomingLines()` läuft und nicht mehr blockiert ist, sobald
  `rearFrameClient` den Frame abgeschlossen hat.
- `#RTX` (58 Byte) aus `updateRadioStatus()` am Ende desselben Durchlaufs.
  Die Zeile habe ich selbst eingeführt, Chatty hat sie in der Rechnung nicht
  berücksichtigt.

Nachgerechnet, Bedarf einschließlich Längenbyte je Zeile:

| Stoß | Bedarf | passt in 240 |
|---|---:|---|
| Frame wie heute konfiguriert | 174 | ja |
| Frame heute + Kommandostart | 219 | ja |
| Frame heute + Statuszeile | 233 | ja |
| Frame heute + Kommandostart + Statuszeile | 278 | **nein** |
| Frame mit COUNTS an | 239 | ja, knapp |
| Frame mit COUNTS an + Kommandostart | 284 | **nein** |
| Frame mit COUNTS an + Status + Kommandostart | 343 | **nein** |

240 ist also keine lückenlose Garantie, sondern deckt den regelmäßigen
Frame-Stoß ab. Das ist vertretbar, sollte aber so dastehen und nicht als
Vollabdeckung.

**Wichtig ist, was bei Überlauf passiert.** In allen sieben Fällen habe ich
den Stoß in den Ring gefüllt, ohne zu senden, und danach leer gefahren:

```
Frame mit COUNTS an + Kommandostart
  angenommen 3 von 4, empfangen 3, Inhalt korrekt: True
Frame mit COUNTS an + Status + Kommandostart
  angenommen 3 von 5, empfangen 3, Inhalt korrekt: True
```

Es fallen ganze Zeilen weg, nie Bruchstücke, und keine empfangene Zeile ist
verfälscht. Der Verlust wird gezählt und ist in `#RTX` sichtbar. Genau dafür
war die Konstruktion gedacht.

## 4. Ein Vorschlag, der nichts kostet

Die `#RTX`-Zeile ist die einzige der drei Zusatzzeilen, deren Zeitpunkt frei
wählbar ist. Wenn sie nur ausgegeben wird, solange der Sender nichts zu tun
hat, verschwindet sie aus jedem Stoß:

```cpp
void FrontApp::updateRadioStatus(uint32_t now)
{
    // Die Statuszeile hat keine Eile. Sie soll sich nicht mit dem
    // Frame-Stoss um den Ringplatz streiten.
    if (radioTelemetry.busy())
    {
        return;
    }
    ...
```

`busy()` gibt es bereits. Damit sinkt der schlechteste Fall bei der heutigen
Konfiguration von 278 auf 219 Byte und passt wieder. Bei eingeschaltetem
`PRINTER_ENABLE_COUNTS` bleibt der Fall „Frame + Kommandostart" mit 284 Byte
offen; dann geht bei jedem Kommandostart eine Zeile verloren, sichtbar im
Zähler. Das ist hinnehmbar.

Ich habe die Zeile **nicht** eingebaut — die Endfassung gehört Chatty, und du
wolltest keine Änderung an den Senderdateien.

## 5. Blockierdauer

Chatty rechnet 2,60 ms für drei Versuche und hält meine 2,8 ms für eine
brauchbare konservative Schätzung. Beide Rechnungen liegen in derselben
Größenordnung, und Chatty sagt richtig, dass das kein garantierter Grenzwert
ist. Es bleibt dabei: maßgeblich ist `maxSendMicros()` am Fahrzeug, letztes
Feld von `#RTX`.

## 6. Kompilierergebnis der Endfassung

`arduino-cli`, FQBN `arduino:avr:nano:cpu=atmega328old`, RF24 1.6.1.

| | Flash | | SRAM | | frei für Stack |
|---|---:|---:|---:|---:|---:|
| vorne, vorher | 22836 | 74 % | 955 | 46 % | 1093 |
| vorne, meine Fassung (208) | 26420 | 86 % | 1274 | 62 % | 774 |
| **vorne, Endfassung (240)** | **26422** | **86 %** | **1306** | **63 %** | **742** |
| hinten, vorher und nachher | 13368 | 43 % | 706 | 34 % | 1342 |
| Empfänger | 4338 | 14 % | 396 | 19 % | 1652 |

Chattys Vorhersage von „etwa 1306 Byte" trifft auf das Byte zu. Der hintere
Nano bleibt bitgenau unverändert, die Regeln aus B6 halten also.

## 7. Ringlogik bei 240

Alle sechs Tests aus dem ersten Bericht laufen mit `RING_SIZE = 240`
unverändert durch: Wraparound, überlange Zeile, Überlauf, 15 % Paketverlust,
30 % Duplikate. Keine verfälschte Zeile in keinem Fall.

## 8. Urteil

Die Endfassung ist aus meiner Sicht übernahmereif. Die Korrektur auf 240 Byte
ist berechtigt und richtig gerechnet, der Rest entspricht der geprüften
Fassung, und alle drei Ziele kompilieren.

Offen bleibt allein, was ohne Hardware nicht zu klären ist: die tatsächliche
Blockierdauer, das Verhalten der Regelung und die Verlustrate. Dafür sind die
Zähler in `#RTX` und `#RSTAT` da.
