# Aufgabe von Claude

- Lies die zu prüfenden Dateien aus dem Ordner chatty.
- Vergleiche sie mit dem bestehenden Roboterprojekt.
- Schreibe korrigierte vollständige Dateien in den Ordner claudine.
- Verändere die echten Projektdateien nur, wenn ich ausdrücklich „Endfassung übernehmen“ sage.
- Keine Git-Commits, kein Push und kein Hochladen auf die Arduinos ohne meine ausdrückliche Erlaubnis.

# Hardware-Tests und Fehlersuche

Wenn ich Messwerte oder Diagramme schicke, fehlt oft der Zusammenhang, ohne den
sie sich nicht deuten lassen. Frag danach, statt zu raten:

- **Aufbau.** Steht der Roboter aufgebockt oder auf dem Boden? Was steht in
  Blickrichtung der Sensoren? Ein Gegenstand, von dem du nichts weißt, sieht in
  den Daten aus wie ein Softwarefehler.
- **Gemessen oder angenommen.** Nimm meine Abstandsangaben nur dann als
  Referenz, wenn ich sie ausdrücklich als nachgemessen bezeichne. Sonst frag
  nach, bevor du darauf eine Diagnose stützt.
- **Was sich geändert hat.** Umgesteckt, neu geflasht, Roboter verschoben. Frag
  im Zweifel, welcher Firmwarestand tatsächlich auf welchem Nano liegt.

Kennzeichne Vermutungen ausdrücklich als Vermutungen, auch wenn sie gut
begründet sind. Schlag den Test vor, der zwischen zwei möglichen Erklärungen
entscheidet, bevor du eine davon ausbaust. Sprich eine Empfehlung erst aus,
wenn geklärt ist, ob ein Effekt reproduzierbar ist oder einmalig war.

Ich darf dich jederzeit fragen: „Ist das gemessen oder vermutet?“
