# Spannungskontrolle: Startsperre bei leerem Akku

Stand: 10.08.2026

Randbedingungen aus der Abstimmung: 2S LiPo, ein Akku für Logik und
Motoren, Sperre nur beim Start. Der STBY-Eingang der vorderen Treiber
hängt an derselben Leitung wie der hintere, D8 am hinteren Nano
schaltet also alle vier Räder. Der Pulldown nach GND ist vorhanden.

## 1. Schaltung

```
                         Knoten A
   Akku (+) ── Schalter ──┬──────────────── Step-Down (IN+)
                          │                 Treiber VM
                          │
                        [ R1 = 97k ]
                          │
                          ├──────────────── A7   (hinterer Nano)
                 Knoten B │
                          ├───[ C = 100nF ]──┐
                          │                  │
                        [ R2 = 10k ]         │
                          │                  │
   Akku (−) ──────────────┴──────────────────┴── GND (hinterer Nano)
                         Knoten C
```

### Bauteile

| Bauteil | Wert | Anmerkung |
|---|---|---|
| R1 | 100,2 kΩ | nachgemessen; Wert unkritisch, siehe unten |
| R2 | 9,84 kΩ | nachgemessen |
| C | 100 nF | Keramik, ungepolt |

Verlustleistung: 8,4 V an 110 kΩ ≈ 0,64 mW. Die Bauform ist also egal,
jeder Widerstand aus der Bastelkiste kann das.

**Zur Genauigkeit der Widerstände:** Der absolute Wert spielt keine
Rolle. Das Teilerverhältnis geht zusammen mit der Referenzstreuung in
denselben Korrekturfaktor ein (Abschnitt 4) und wird dort mit
abgeglichen. Wichtig sind nur zwei Dinge: das Verhältnis muss den
Abgriff bei voller Ladung unter 1,1 V halten — das gemessene
Verhältnis 11,18 liefert bei 8,4 V rund 0,75 V, also reichlich Luft,
die Messgrenze läge bei 12,3 V — und die beiden Widerstände sollten
über die Zeit stabil
bleiben. Selbst Kohleschicht driftet über 20 K nur um Bruchteile
eines Prozents, und im Teiler wirkt ohnehin nur die *Differenz* der
Drift beider Widerstände. Das liegt weit unter dem, was hier zählt.

Nach der Kalibrierung begrenzt die ADC-Auflösung die Genauigkeit:
ein Digit entspricht 1,1 V / 1023 · 10,7 ≈ 11,5 mV am Akku. Gegenüber
einer Schwelle, die man auch 200 mV anders ansetzen könnte, ist das
belanglos.

### Die drei Knoten

**Knoten A — Abgriff am Pluspol.** Ein Bein von R1 kommt an denselben
Punkt, an dem der Step-Down seinen Eingang hat und die Motortreiber
ihre VM-Versorgung holen: also **hinter** dem Hauptschalter, nicht
davor. Zwei Gründe: vor dem Schalter zöge der Teiler dauerhaft 76 µA
aus dem Pack, auch wenn der Roboter wochenlang steht (rund 55 mAh im
Monat). Und einen Nutzen hätte es nicht — bei offenem Schalter ist der
Nano ohnehin aus und misst nichts.

**Knoten B — der Abgriff.** Hier treffen sich das zweite Bein von R1,
ein Bein von R2, ein Bein des Kondensators und die Leitung zu A7.
Sonst nichts.

**Knoten C — Masse.** Zweites Bein von R2 und zweites Bein des
Kondensators gehen an einen GND-Pin des hinteren Nano. Nicht an
irgendeinen Massepunkt im Motorstrompfad: über eine Litze fließen bei
2 A schnell einige zehn Millivolt ab, und die addieren sich direkt auf
den Messwert, wenn Teiler und Nano an verschiedenen Punkten hängen.
Beim Start wird stromlos gemessen, das ist also unkritisch — sauberer
ist es trotzdem.

Ein- und Ausgangsmasse des Step-Down müssen durchverbunden sein, damit
Akkuminus und Nano-GND dasselbe Potential sind. Bei den üblichen nicht
isolierten Modulen ist das der Fall.

### Am Nano

Auf der analogen Pinreihe liegen von der USB-Buchse aus gesehen:

```
D13 · 3V3 · AREF · A0 · A1 · A2 · A3 · A4 · A5 · A6 · A7 · 5V · RST · GND · VIN
                    └──── Encoder ────┘   └─ IMU ─┘    ▲                 ▲
                                                       │                 │
                                                   Abgriff             Masse
```

A7 liegt also direkt neben dem 5-V-Pin, der nächste GND-Pin ist drei
Positionen weiter am Ende der Reihe. Kurz genug, um R2 und C direkt
zwischen A7 und diesen GND-Pin zu löten.

### Aufbau in der Praxis

Knoten B ist mit 9,1 kΩ Quellwiderstand vergleichsweise hochohmig und
fängt sich Störungen ein. Deshalb: **den Teiler beim Nano aufbauen,
nicht beim Akku.** Die lange Leitung ist dann die vom Pluspol zu R1 —
die ist niederohmig und stört sich nicht. Die Leitung von Knoten B zu
A7 bleibt kurz, im Idealfall wenige Zentimeter.

Der Kondensator ist nicht Kosmetik: der ATmega will an einem
ADC-Eingang höchstens etwa 10 kΩ Quellwiderstand sehen, der Teiler
liegt mit 9,1 kΩ knapp darunter. Ohne Stützkondensator wird der Wert
unruhig.

### Status-LED an D11/D12

```
   D11 ──[ LED ]──[ 330 Ω ]── D12
  (Plus)                     (Minus)
```

Die LED hängt zwischen zwei Ausgängen statt zwischen Pin und Masse:
D11 führt HIGH, D12 liegt LOW und übernimmt die Rolle der Masse. Am
hinteren Nano läuft kein SPI, D11 und D12 sind deshalb frei. Welcher
Pin welche Rolle hat, steht in `HardwarePins.h`.

Strom: bei rund 4,0 V wirksamer Spannung über LED und Widerstand
fließen mit 330 Ω etwa 6 mA bei einer roten LED, bei blau oder weiß
wegen der höheren Flussspannung eher 3 mA. Der ATmega darf 20 mA je
Pin dauerhaft und 40 mA absolut — es ist also reichlich Luft. Falls
die LED zu dunkel wirkt, sind 150 Ω noch problemlos, dann sind es
höchstens 15 mA.

**Beim Aufbau passiert:** Zuerst steckte ein 330-kΩ-Widerstand statt
330 Ω — die beiden unterscheiden sich nur im Multiplikatorring. Die
LED lief damit mit etwa 12 µA, war also elektrisch in Betrieb und
trotzdem völlig unsichtbar. Wenn eine LED „gar nicht geht", obwohl
die Ansteuerung nachweislich stimmt, lohnt der Blick auf den
Widerstandswert vor jeder weiteren Fehlersuche.

Zwei Eigenschaften dieser Verdrahtung sind praktisch:

- **Die Polarität ist Softwaresache.** Leuchtet die LED nicht, müssen
  `STATUS_LED_ANODE_PIN` und `STATUS_LED_CATHODE_PIN` in
  `HardwarePins.h` getauscht werden — kein Umlöten.
- **Im Ruhezustand liegen beide Pins LOW**, die LED ist also stromlos.
  Während Reset und Bootloader sind beide Pins hochohmig, sie bleibt
  dann ebenfalls dunkel.

Einschränkung: D11 und D12 sind gleichzeitig MOSI und MISO. Über den
USB-Bootloader zu flashen stört das nicht. Nur falls du den hinteren
Nano irgendwann über einen ISP-Programmer bespielen willst, hängt die
LED als Last an diesen Leitungen — dann gegebenenfalls abziehen.

### Vor dem Anstecken prüfen

Ein Verdrahtungsfehler an dieser Stelle setzt 8,4 V direkt auf einen
ADC-Eingang und zerstört ihn. Deshalb in dieser Reihenfolge:

1. Teiler fertig aufbauen, die Leitung zu A7 aber noch **offen**
   lassen.
2. Akku anklemmen, Schalter ein.
3. Mit dem Multimeter Knoten B gegen GND messen.
4. Erwartungswert ist Akkuspannung geteilt durch das
   Teilerverhältnis. Am gebauten Aufbau sind es 11,18
   (aus 100,2 kΩ / 9,84 kΩ; die Spannungsprobe 7,56 V zu
   0,675 V bestätigt das mit 11,20):

   | Akku | an Knoten B |
   |---|---|
   | 8,4 V | 0,75 V |
   | 7,6 V | 0,68 V |
   | 7,4 V | 0,66 V |
   | 7,2 V | 0,64 V |
   | 7,0 V | 0,63 V |

5. Erst wenn das passt, die Leitung an A7 anschließen.

Liegt dort mehr als etwa 1 V, stimmt etwas nicht — dann sind
vermutlich R1 und R2 vertauscht. Das ist der Fehler, der weh tut:
vertauscht kämen 7,6 V an den Pin.

Ein beruhigender Nebeneffekt der Dimensionierung: durch die 100 kΩ in
Reihe kann selbst ein Kurzschluss der A7-Leitung gegen Masse nur
etwa 84 µA ziehen. Der Abgriff ist von sich aus ungefährlich für den
Akku.

## 2. Warum gegen die interne 1,1-V-Referenz gemessen wird

Mit der Standardeinstellung ist die ADC-Referenz gleich VCC. Da die
5 V bei dir aus einem Step-Down kommen, brechen sie über den nutzbaren
Akkubereich nicht ein — der schlimmste Fall, eine im interessanten
Bereich blinde Messung, tritt also nicht auf.

Die Messung hinge dafür an der Ausgangsgenauigkeit des Wandlers. Die
üblichen Module liegen je nach Last und Exemplar zwischen etwa 4,9 und
5,2 V, das sind ±3 % — und dieser Fehler ginge unbemerkt direkt in die
Schwelle ein, bei 7,2 V also gut 0,2 V. Dazu käme, dass eine
Kalibrierung nur für den Lastzustand gilt, in dem du kalibriert hast.

Die interne Bandgap-Referenz hängt an nichts davon: sie ist einmal
kalibrierbar und bleibt es. Deshalb `analogReference(INTERNAL)`.

Zwei Nebenwirkungen davon sind im Code berücksichtigt:

- Nach dem Umschalten braucht die Referenz einige Millisekunden.
  Die ersten Wandlungen sind unbrauchbar und werden verworfen
  (`BATTERY_DISCARD_SAMPLES`).
- Der ADC steht danach dauerhaft auf 1,1 V. Das ist unkritisch, weil
  im Projekt sonst kein `analogRead()` vorkommt — die Encoder- und
  Echo-Pins auf A0–A5 werden alle digital gelesen. Falls später doch
  eine analoge Messung dazukommt, muss die Referenz umgeschaltet und
  jedes Mal neu eingeschwungen werden.

## 3. Warum A7

Hinten sind A0–A3 die Encoder, A4/A5 für die IMU reserviert. A6 und A7
sind beim Nano reine ADC-Eingänge — sie können gar nicht digital, sind
deshalb ohnehin ungenutzt und kosten dich keinen Pin. Sie brauchen auch
kein `pinMode()`.

## 4. Kalibrierung — nicht überspringen

Die interne Referenz ist mit 1,1 V nur nominell angegeben und streut
fertigungsbedingt etwa zwischen 1,0 und 1,2 V. Ohne Kalibrierung liegt
die Schwelle um bis zu 10 % daneben, bei 2S sind das über 0,7 V. Damit
wäre die ganze Übung wertlos.

Ein einziger Korrekturfaktor deckt die Streuung der Referenz **und**
die Toleranz der beiden Widerstände gleichzeitig ab:

1. Akku anklemmen, Roboter aufgebockt, keine Fahrbefehle.

   Der Ladezustand ist weitgehend egal — es geht um einen reinen
   Skalierungsfaktor, der über den ganzen Bereich gilt. Ideal ist ein
   Punkt in der Nähe der Schwelle, also etwa 7,2 bis 7,6 V: der
   Restfehler des ADC wird genau dort null, wo du kalibrierst, und
   wächst mit dem Abstand. Zwischen dort und vollem Akku liegen aber
   nur rund 20 mV Unterschied am Entscheidungspunkt. Nimm also
   getrost den Akku, der gerade dranhängt.

   Wichtiger als der Ladezustand: der Akku muss **zur Ruhe gekommen**
   sein. Direkt nach dem Laden liegt die Spannung zu hoch und fällt
   über die ersten Minuten ab, direkt nach der Fahrt umgekehrt. Ein
   paar Minuten warten, und nicht am Ladegerät hängend messen.
2. Mit dem Multimeter an Knoten A gegen GND messen → `U_mult` in mV.
3. Seriellen Monitor öffnen (115200). Beim Start kommt
   `#BATT,OK,<mV>` → `U_gem`.
4. Neuen Wert rechnen:

   ```
   BATTERY_REF_MV_neu = BATTERY_REF_MV_alt * U_mult / U_gem
   ```

   Beispiel: alt 1100, Multimeter 8,21 V, gemeldet 7,94 V
   → 1100 · 8210 / 7940 ≈ 1137.

5. Wert in `BatteryGuard.h` eintragen, neu flashen, gegenprüfen.
   Abweichung sollte danach unter etwa 50 mV liegen.

Beide Messungen müssen im selben Lastzustand stattfinden, sonst
kalibrierst du den Innenwiderstand des Akkus mit ein.

## 5. Schwelle

Eingetragen ist `BATTERY_MIN_START_MV = 7200`, also 3,6 V je Zelle im
Leerlauf.

Das ist bewusst konservativer als die üblichen 3,5 V, und zwar wegen
der Entscheidung „nur beim Start": während der Fahrt schaut niemand
mehr hin. Was die Prüfung passiert, muss also eine ganze Fahrt lang
reichen. Zum Einordnen (2S, Leerlaufspannung):

| Spannung | je Zelle | Bedeutung |
|---|---|---|
| 8,4 V | 4,20 V | voll |
| 7,4 V | 3,70 V | Nennspannung, grob halbvoll |
| 7,2 V | 3,60 V | **eingestellte Startschwelle** |
| 7,0 V | 3,50 V | übliche Abschaltschwelle im Betrieb |
| 6,6 V | 3,30 V | Untergrenze, darunter Zellschaden |

Wenn dir 7,2 V zu früh sperrt, ist 7,0 V vertretbar — dann aber mit dem
Wissen, dass eine anschließende Fahrt den Akku in den Bereich unter
3,3 V/Zelle ziehen kann.

## 6. Verhalten beim Start

Die Prüfung sitzt in `RearApp::begin()` direkt nach `hardware_begin()`
und damit **vor** `conn.begin(true)`, das blockierend auf den vorderen
Nano wartet. Bei leerem Akku kommt die Meldung so sofort und nicht erst
nach dem Handshake.

Nach `hardware_begin()` ist D8 Ausgang und liegt LOW, beide Achsen sind
also schon stromlos, wenn gemessen wird.

- **Akku in Ordnung:** Status-LED geht auf **Dauerlicht**,
  `#BATT,OK,<mV>` auf der seriellen Schnittstelle, danach läuft alles
  wie bisher. Die Motoren werden kurz darauf freigegeben, sobald die
  Verbindung zum vorderen Nano steht.
- **Akku zu leer:** `hardware_disableMotors()`, dann Endlosschleife.
  Status-LED **blinkt** gleichmäßig im halbsekundentakt, alle vier
  Blinks kommt `#BATT,LOW,<mV>`. Es geht erst nach Akkuwechsel und
  Reset weiter.

Dauerlicht heißt genau genommen nicht „Akku ist gerade in Ordnung",
sondern „die Startprüfung wurde bestanden". Während der Fahrt schaut
niemand mehr hin, die LED bleibt also an, bis der Roboter neu startet.
Erst mit einer Überwachung im Betrieb (bewusst nicht gebaut, siehe
Abschnitt 7) würde daraus eine echte Live-Anzeige.

Das `#`-Präfix ist bewusst gewählt: der vordere Nano verwirft Zeilen,
die damit beginnen (`UartLink::update`, `_buf[0] == '#'`), genau wie
bei `#SYNC`.

Die Anzeige ist wichtiger, als sie aussieht. Ohne sichtbares Signal
ist „Roboter fährt nicht" nicht von einem Softwarefehler zu
unterscheiden.

## 7. Was diese Lösung nicht leistet

- **Kein Schutz während der Fahrt.** Startest du mit 7,3 V, kann der
  Akku am Ende der Fahrt deutlich unter 3,3 V/Zelle liegen. Wenn dir
  der Akku wichtig ist, gehört ein unabhängiger LiPo-Warner (die
  kleinen Piepser für den Balancerstecker) dazu — der hängt nicht an
  deiner Firmware und meldet zellenweise.
- **Kein Bremsen.** STBY LOW schaltet die Ausgänge hochohmig, die
  Motoren laufen frei. Für die Startsperre ohne Belang, für eine
  spätere Abschaltung im Betrieb relevant.
- **Keine Aussage über Zellsymmetrie.** Gemessen wird die Packspannung.
  Eine eingebrochene Einzelzelle fällt damit nicht auf.

## 8. Versorgung über Step-Down: Folgen

Der Nano bekommt seine 5 V aus einem Step-Down, nicht über VIN. Damit
ist die Brownout-Frage erledigt: die Logik bleibt über den gesamten
nutzbaren Akkubereich stabil, ein Reset mitten in der Fahrt wegen
sackender Akkuspannung ist nicht zu erwarten.

Das hat aber eine Kehrseite, die zur Startsperre gehört. Bei
VIN-Versorgung hätte ein leerer Akku sich von selbst bemerkbar
gemacht — der Roboter wäre zickig geworden und irgendwann stehen
geblieben, unschön, aber immerhin ein Signal. Der Step-Down regelt
stur weiter, bis weit unter 3,3 V je Zelle. Es gibt also keine
Vorwarnung mehr aus dem Verhalten des Roboters.

Zwei Konsequenzen:

- Die Schwelle von 7,2 V sollte bleiben. Sie ist jetzt die einzige
  Instanz, die auf den Akku aufpasst.
- Der unabhängige LiPo-Warner am Balancerstecker (Abschnitt 7) ist
  damit kein Luxus mehr, sondern die einzige Absicherung während der
  Fahrt.

Zwei Punkte zur Verdrahtung:

- Der Teiler wird **vor** dem Step-Down abgegriffen, an den
  Akkuklemmen. Am 5-V-Ausgang gemessen wäre der Wert konstant und
  wertlos.
- Ein- und Ausgangsmasse des Step-Down müssen durchverbunden sein,
  sonst hat der ADC keinen gemeinsamen Bezug mit dem Teiler. Bei den
  üblichen nicht isolierten Modulen ist das der Fall.

## 9. Prüfung ohne leeren Akku

Der entscheidende Test lässt sich machen, ohne einen Akku
leerzufahren — die Schwelle wird einfach verstellt:

1. `BATTERY_MIN_START_MV` testweise auf 8500 setzen, flashen.
   → Erwartung: Roboter sperrt, D13 blinkt, `#BATT,LOW,<mV>`, keine
   Räderbewegung an **beiden** Achsen. Das prüft die gemeinsame
   STBY-Leitung gleich mit.
2. Wert auf 5000 setzen, flashen.
   → Erwartung: `#BATT,OK,<mV>`, normaler Start.
3. Wert auf 7200 zurücksetzen, flashen.
4. Gemeldeten Wert gegen das Multimeter halten (siehe Abschnitt 4).

Wenn du ein Labornetzteil hast, ist die sauberere Variante, den Teiler
allein daraus zu speisen und die Schwelle von oben anzufahren — dann
siehst du auch, ob der Übergang bei der richtigen Spannung liegt.

### Stand der Prüfung (11.08.2026)

Beide vereinbarten Zustände sind am fertigen Aufbau vorgeführt, mit
laufendem vorderem Nano und abgesetztem Fahrbefehl:

| Schwelle | Akku | LED | Motoren | seriell |
|---|---|---|---|---|
| 7200 | 7,56 V | leuchtet ruhig | laufen | `#BATT,OK,7565` |
| 8000 | 7,56 V | **blinkt** | **laufen nicht** | `#BATT,LOW` |

Der Unterschied liegt allein an der Schwelle — gleicher Akku, gleicher
Aufbau, gleiche Bedienung. Damit ist belegt, dass die Prüfung greift
und nicht etwa immer oder nie sperrt.

Weitere Nachweise nebenbei:

| geprüft | Ergebnis |
|---|---|
| Teilerverhältnis | 11,18, zweifach bestätigt (Widerstände und Spannungen) |
| Kalibrierung | `#BATT,OK,7553` gegen 7,55 V am Multimeter — unter einem ADC-Digit Abweichung |
| Sperre ohne Spannung | bei ausgeschaltetem Akku `#BATT,LOW,12`, gesperrt |

**Nicht durchgeführt:** die Messung an D8 im Vergleich. Sie wäre nur
aussagekräftig, wenn der vordere Nano mitläuft — sonst bleibt D8 auch
im Normalfall auf 0 V, weil der hintere in `#WAIT` hängt und
`hardware_enableMotors()` nie erreicht. Nach dem Fahrtest oben ist sie
allerdings entbehrlich: dass die Räder bei anliegendem Fahrbefehl
stehenbleiben, sagt dasselbe aus und ist direkter.

## 10. Dateien in diesem Ordner

| Datei | Änderung |
|---|---|
| `2_Hardware/src/BatteryGuard.h` | neu |
| `2_Hardware/src/BatteryGuard.cpp` | neu |
| `2_Hardware/src/StatusLed.h` | neu |
| `2_Hardware/src/StatusLed.cpp` | neu |
| `2_Hardware/src/HardwarePins.h` | `BATTERY_SENSE_PIN = A7` und die beiden LED-Pins in `PinsRear`, Kommentar zur gemeinsamen STBY-Leitung; zusätzlich `SYNC_INPUT_PIN` und `CONNECTION_LED_PIN` aufgenommen (siehe unten) |
| `2_Hardware/2_Hardware.vcxitems` | `BatteryGuard` und `StatusLed` eingetragen |
| `hinten/src/RearApp.h` | `checkBatteryOrHalt()`, `haltOnLowBattery()` |
| `hinten/src/RearApp.cpp` | Prüfung in `begin()`, Sperrzustand mit Blinkmuster |

`BATTERY_SENSE_PIN` steht bewusst **nicht** in `HardwarePinSet`: den
Teiler gibt es nur hinten, sonst müsste vorne ein Platzhalterwert
eingetragen werden.

### Mit aufgeräumt

Zwei Pinnummern standen bisher hart in `RearApp.cpp` und sind jetzt
ebenfalls in `PinsRear` gewandert:

| vorher | jetzt |
|---|---|
| `static const uint8_t REAR_SYNC_INPUT_PIN = 2;` | `PinsRear::SYNC_INPUT_PIN` |
| `conn(uart, 13)` | `PinsRear::CONNECTION_LED_PIN` |

Reine Umschichtung, kein Verhaltensunterschied. Damit gibt es in
`RearApp.cpp` keine Pinnummer mehr — einzige Quelle ist
`HardwarePins.h`. `SYNC_INPUT_PIN` ist auch namentlich das Gegenstück
zu `PinsFront::SYNC_OUTPUT_PIN`.

Am vorderen Nano ändert sich nichts. Er läuft bei gesperrtem Akku
normal hoch und erzeugt Fahrbefehle — die Räder bleiben trotzdem
stehen, weil sein Treiber-STBY an derselben Leitung hängt.
