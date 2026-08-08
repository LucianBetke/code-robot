# Arbeitsanweisung Stufe 1: Reset des vorderen Nano bei Verbindungsabbruch

## 0. Ausführungsmodus (WICHTIG – übersteuert den Standard-Workflow)

Für **diese Aufgabe** gilt der in `CLAUDE.md` / `AGENTS.md` beschriebene
`chatty`/`claudine`-Workflow **nicht**. Konkret:

- Die Änderung wird **direkt in die echte Projektdatei** `vorne/src/FrontApp.cpp`
  geschrieben.
- **Nicht** nach `claudine` oder `chatty` schreiben, keine Zwischenfassung,
  kein Warten auf „Endfassung übernehmen“.
- Weiterhin gilt: **keine** Git-Commits, **kein** Push, **kein** Upload auf die
  Arduinos.
- Nur `vorne/src/FrontApp.cpp` anfassen. Keine andere Datei ändern.

Dieser Ausführungsmodus gilt ausschließlich für diese Arbeitsanweisung.
Ab der nächsten Aufgabe gilt wieder der normale Workflow aus `CLAUDE.md`.

## 1. Ziel

Der vordere Nano (`vorne`) soll bei einem Abbruch der UART-Verbindung zum
hinteren Nano **nicht mehr weiterlaufen und nur die Aktorik stoppen**, sondern
sich – wie der hintere Nano bereits – per Watchdog **neu starten**.

Gewünschtes Gesamtverhalten:

- Schalter zwischen den Nanos offen -> Verbindung tot -> vorne stoppt
  Fahrzeug/Regler und startet neu.
- Nach dem Neustart hängt vorne blockierend in `conn.begin(true)` bzw.
  `waitForConnection()` und sendet als Initiator weiter `PING`.
- Schalter wieder geschlossen -> Handshake -> beide Nanos laufen an.

Dies ist **Stufe 1** eines mehrstufigen Umbaus. Nur diese eine Änderung
umsetzen, nichts anderes.

## 2. Betroffene Datei

- `vorne/src/FrontApp.cpp`, Methode `FrontApp::updateConnectionSafety()`.

`resetByWatchdog()` existiert vorne bereits und wird an anderer Stelle
(`updateFrameTimeout()`) schon verwendet – also nicht neu anlegen, sondern
denselben vorhandenen Mechanismus nutzen.

## 3. Durchzuführende Änderungen

### 3.1 Startwert von `prevConnected`

In `updateConnectionSafety()` den statischen Startwert von

    static bool prevConnected = false;

auf

    static bool prevConnected = true;

ändern. Damit verhält sich vorne wie hinten (`RearApp::updateConnectionSafety`
nutzt ebenfalls `true`).

### 3.2 Abriss-Flanke: Stoppen und Reset

Der Block

    if (prevConnected && !nowConnected)
    {
        ... bisheriger Aufräumcode ...
    }

wird ersetzt durch ein minimales Stoppen der Aktorik gefolgt vom Watchdog-
Reset (analog zu `RearApp::updateConnectionSafety`, das `stopRearWheels()`
und dann `resetByWatchdog()` aufruft):

    if (prevConnected && !nowConnected)
    {
        vehicle.stop();
        radControl_stopAll();
        resetByWatchdog();
    }

Der bisherige Aufräumcode in dieser Flanke (PI-States,
`wheelMeasurement_reset_all`, `rearFrameClient.clearWaiting/clearFrame/
cancelStopSequence`, `frameScheduler.stop`, `_odomResetPending`,
Ultraschall-Cache) entfällt, weil der anschließende Neustart über
`FrontApp::begin()` diese Initialisierung ohnehin vollständig ausführt.

### 3.3 Wiederverbindungs-Flanke entfernen

Der komplette Block

    if (!prevConnected && nowConnected)
    {
        ...
    }

wird **ersatzlos entfernt**. Nach dem Watchdog-Neustart durchläuft
`FrontApp::begin()` bereits `commandRunner.begin()`, `rearFrameClient.begin()`
und `frameScheduler.begin(VEHICLE_DT_MS)`. Dieser Block ist damit toter Code.

### 3.4 Zeile `prevConnected = nowConnected;`

Die abschließende Zuweisung `prevConnected = nowConnected;` am Ende der
Methode bleibt erhalten.

## 4. Was ausdrücklich NICHT geändert wird

- Kein Umbau am `ConnectionMonitor`, keine Änderung an der D13-LED
  (spätere Stufe).
- Keine Änderung an Ultraschall, Funk, IMU oder Pin-Belegungen.
- `resetByWatchdog()` nicht umbauen; vorhandene Implementierung
  (`wdt_enable(WDTO_15MS); while(1){}`) unverändert nutzen.

## 5. Kompilier-Check (Sicherheitsnetz, kein Workflow-Umweg)

1. Vor der Änderung `vorne` einmal kompilieren und die Program-size-Zeile
   notieren (Ausgangswert, aktuell ca. 20572 Bytes).
2. Änderung umsetzen.
3. `vorne` erneut kompilieren (Board: Arduino Nano, ATmega328P, Old
   Bootloader). Es muss **fehlerfrei** durchlaufen. Die Größe sollte etwa
   gleich bleiben oder leicht sinken (entfernter Code).
4. Bei Kompilierfehler: Änderung zurücknehmen und melden, nicht raten.

## 6. Prüfpunkte für den späteren Hardware-Test

- Erststart bei offenem Schalter: vorne rebootet, landet blockierend in
  `waitForConnection()` und sendet `PING`. Kein unerwünschtes schnelles
  Reboot-Flackern, das Funk/Telemetrie stört.
- Schalter im Betrieb öffnen: vorne stoppt Aktorik und rebootet einmal.
- Schalter schließen: Handshake (`#WAIT` -> `#CON`), beide Nanos fahren an.
- Der vorhandene Reset in `updateFrameTimeout()` bleibt funktional und wird
  nicht doppelt/konkurrierend ausgelöst.
