# Befund 3 geprüft: mischt der CommandRunner zwei Aufgaben?

Stand: 11.08.2026
Grundlage: `5_System/src/CommandRunner/CommandRunner.{h,cpp}` vollständig
gelesen, dazu alle Aufrufstellen in `vorne/src/FrontApp.{h,cpp}`.
**Am Code wurde nichts geändert.**

Dies ist Schritt 1 der empfohlenen Reihenfolge aus
[BESTANDSAUFNAHME.md](BESTANDSAUFNAHME.md).

## Kurzantwort

Befund 3 trägt zur Hälfte. Es liegen tatsächlich zwei Aufgaben in der
Datei — aber nicht in dem Verhältnis, das der Header vermuten ließ, und
der naheliegende Umbau (Fahrzeugteil eine Schicht tiefer nach 4_Vehicle)
wäre eine Verschlechterung, nicht eine Verbesserung. **Empfehlung: Befund 3
als Umbau zurückstellen.** Stattdessen zwei kleine, belegte Aufräumpunkte
mitnehmen (Abschnitt „Was sich stattdessen lohnt").

## Was wirklich in der Datei steht

Grob gezählt, Zeilen der `.cpp` nach Aufgabe:

| Aufgabe | Zeilen | Anteil |
|---|---|---|
| Zustandsmaschine Fahrbefehl (Start, Fortschritt, Ziel, Timeout, Settle) | ~380 | 67 % |
| Ablauf über die Befehlsliste (parsen, weiterschalten, Ende erkennen) | ~40 | 7 % |
| CMDP-Id und `#`-Ausgaben | ~60 | 11 % |
| Konstruktor, `begin()`, Zustand zurücksetzen | ~55 | 10 % |
| Zugriffsfunktionen | ~30 | 5 % |

Der „Systemteil", den der Header so gleichrangig aussehen ließ, ist
in Wirklichkeit die kleinste Portion: die Schleife in `update()`
(Zeile 367–398) ist rund 30 Zeilen lang. Alles andere ist Fahrbefehl.

Ein 40/380-Schnitt rechtfertigt keine zweite Klasse. Das ist kein
gemischtes Modul, sondern eine Zustandsmaschine für Fahrbefehle mit
einer kleinen Schleife davor.

## Warum der Umzug nach 4_Vehicle ausscheidet

Der naheliegende Gedanke wäre, die Zustandsmaschine dorthin zu legen, wo
`VehicleController` und `MecanumOdometer` liegen. Dagegen spricht ein
handfestes Argument:

**Die Schichten 1 bis 4 sind ausgabefrei.** Gesucht wurde nach `Print&`,
`TelemetryPrinterConfig` und `PRINTER_ENABLE` in der gesamten
Projektmappe. Treffer gibt es ausschließlich in `5_System` und `vorne`.
1_Common, 2_Hardware, 3_Control und 4_Vehicle kennen kein Ausgabeziel.

Die Zustandsmaschine dagegen ist voll davon: `#CMDP_BEGIN`,
`#EVENT,startCmdp`, `#EVENT,pathReached`, `#ERROR,CMDP,timeout` und
sechs Fehlermeldungen bei der Befehlsprüfung. Sie mitzunehmen hieße,
`Print&` und `TelemetryPrinterConfig.h` — eine 5_System-Datei — nach
4_Vehicle zu ziehen. Damit hinge die untere Schicht an der oberen.

Man könnte die Ausgabe vorher entkoppeln (Callback oder Ereignisliste
statt direktem `print`). Das ist auf einem Nano möglich, kostet aber
Indirektion und Flash für einen rein strukturellen Gewinn. Aus meiner
Sicht nicht lohnend — das ist eine Einschätzung, kein Messergebnis.

## Die beiden Teile hängen zusammen

Ein mechanischer Schnitt wäre auch deshalb nicht sauber:

- Die CMDP-Id entsteht beim Start eines Fahrbefehls
  (`startCmdpProtocol`) und wird beim Beenden gelöscht
  (`clearCmdpProtocol`). Ihr Lebenszyklus gehört zum Fahrbefehl, nicht
  zur Liste — sie würde bei einer Trennung mitwandern.
- `_startTime` und `_durationMs` werden von zwei Phasen benutzt: erst
  als Timeout des Fahrbefehls, danach als Dauer der Settle-Phase
  (Zeile 101–102 gegen 248). Das funktioniert, weil die Phasen sich
  nicht überlappen, verbindet aber beide Teile über gemeinsame Felder.
- `finishPathCmd` schaltet den Listenindex weiter (`_cmdIndex++`) und
  entscheidet über `_finished`. Der Fahrteil greift also in den
  Listenteil hinein.

## Was sich stattdessen lohnt

Drei Punkte, absteigend nach Nutzen-je-Risiko. Alle drei betreffen nur
den vorderen Nano.

### 1. Vier tote öffentliche Zugriffsfunktionen (belegt)

`pathProgressCm()`, `pathTargetCm()`, `angleProgressDeg()` und
`angleTargetDeg()` (Header Zeile 41–45) werden **nirgends im Projekt
aufgerufen**. Gesucht in der gesamten Projektmappe außerhalb von
`chatty` und `claudine`: nur die Definitionen selbst.

Die Felder dahinter werden gebraucht, die öffentlichen Funktionen nicht.
Löschen kostet vier Zeilen im Header und ändert kein Verhalten.

### 2. `getWheelSoll` ist eine reine Durchreichung (belegt)

```cpp
float CommandRunner::getWheelSoll(WheelVehicle w) const
{
    return _vehicle.getWheelSoll(w);
}
```

`FrontApp` hält `vehicle` selbst als Element (`FrontApp.h:35`) und
`TelemetryPrinter` ruft an sechs Stellen bereits direkt
`vehicle.getWheelSoll(...)` auf. Nur `FrontApp` geht an sechs Stellen
den Umweg über den Runner (Zeile 492, 496, 554, 567, 580, 584).

Ersetzen durch `vehicle.getWheelSoll(...)` und die Funktion streichen:
sechs Zeilen ändern, eine Abhängigkeit weniger im Header. Verhalten
identisch, weil dieselbe Funktion desselben Objekts gerufen wird.

### 3. Ausgabeformate nach TelemetryPrinter (optional)

Die `#`-Zeilen aus dem CommandRunner ließen sich als
`printCmdpBegin`, `printPathReached`, `printCmdpTimeout` in den
`TelemetryPrinter` verlegen — dorthin, wo alle anderen Formate schon
stehen. Das nähme rund 60 Zeilen aus dem CommandRunner und wäre die
Fortsetzung dessen, was die Funkintegration begonnen hat (dort steht
im Vorschlag ausdrücklich: „Die Ausgabe ist noch nicht zentral").

Zwei Einschränkungen, bevor das jemand als Gewinn verbucht:

- Im aktuellen Build sind `PRINTER_ENABLE_ERRORS` und
  `PRINTER_ENABLE_EVENTS` auf `0` gesetzt. Der größte Teil dieser
  60 Zeilen wird also gar nicht übersetzt. Der Gewinn ist Lesbarkeit,
  **kein Flash**.
- Der CommandRunner bekäme statt `Print&` eine Abhängigkeit auf
  `TelemetryPrinter` — dieselbe Schicht, aber eine dickere Kopplung.

Deshalb: nur mitnehmen, wenn die Telemetriedateien ohnehin angefasst
werden (Befund 2), nicht als eigener Vorgang.

## Auswirkung auf die empfohlene Reihenfolge

Aus der Bestandsaufnahme bleibt damit:

1. ~~`CommandRunner.cpp` lesen und Befund 3 beurteilen~~ — erledigt,
   Ergebnis: kein Umbau.
2. **Umzug des UltrasonicManager nach 2_Hardware (Befund 1).** Jetzt der
   klar erste Schritt. 639 Zeilen, knapp ein Viertel der Schicht,
   Sensortreiber ohne Anwendungslogik, Gegenstück liegt schon dort.
3. Ordnerstruktur vereinheitlichen (Befund 2), im selben Zug.
4. Statt Befund 3: die beiden kleinen Punkte 1 und 2 oben, wenn `vorne`
   ohnehin neu gebaut wird.

Nach Schritt 2 und 3 wäre 5_System bei rund 2.060 Zeilen, ohne dass
fahrender Code angefasst wurde.

## Was nicht geprüft wurde

- Ob die vier toten Zugriffsfunktionen einmal für eine geplante
  Fortschrittsanzeige gedacht waren. Falls ja, wären sie kein Müll,
  sondern Vorbereitung — das steht nirgends im Code und ist deine
  Entscheidung.
- Flash- und RAM-Verbrauch vor/nach den Vorschlägen. Nicht gemessen.
- Die Telemetriedateien (824 Zeilen) im Detail. Für Befund 2 wäre das
  der nächste Lesevorgang.
