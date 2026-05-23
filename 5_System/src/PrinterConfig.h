// PrinterConfig.h
#ifndef PRINTER_CONFIG_H
#define PRINTER_CONFIG_H

// ------------------------------------------------------------
// Ausgabe-Modus
// ------------------------------------------------------------
// Nur einen Modus aktivieren.

// #define PRINTER_MODE_CHASSIS
#define PRINTER_MODE_RAEDER

// ------------------------------------------------------------
// Diagnose-/Zusatzausgaben
// ------------------------------------------------------------
// 1 = Ausgabe aktiv
// 0 = Ausgabe wird beim Kompilieren entfernt

#define PRINTER_ENABLE_INFO     1
#define PRINTER_ENABLE_ERRORS   0
#define PRINTER_ENABLE_EVENTS   0
#define PRINTER_ENABLE_ODOM     1
#define PRINTER_ENABLE_COUNTS   0

#define PRINTER_ENABLE_WHEELS   1
#define PRINTER_ENABLE_CHASSIS  0

#endif // PRINTER_CONFIG_H