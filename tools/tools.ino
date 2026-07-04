// ============================================================
// File: tools.ino (Vollversion ohne Test_Achse)  – EXTERN-Variante
// Zweck:
//  - Modi: HINTERACHSE, DEADBAND, PI_SPRUNG, REGEL, HI_KOPPEL
//  - AutoTuner integriert (ohne beginIMCOpenStep())
//  - Gemeinsame Laufzeitbegrenzung für zeitlimitierte Tests (REGEL/SPEED/HI_KOPPEL)
//
// WICHTIG (Option A + extern):
//  - Encoder/Motor-Hardware wird NUR über hardware_begin(...) initialisiert.
//  - KEIN enc[Re].begin()/enc[Li].begin() mehr im Sketch.
//  - Alle Hardware-Objekte liegen in hardware.cpp und werden via extern genutzt.
// ============================================================

#include <Arduino.h>
#include "globals.h"
#include "ControlParams.h"
#include "VehicleController.h"

#include "Hardware.h"
#include "Control.h"
#include "Tests.h"
#include "Utils.h"
#include "Printer.h"

// >>> TestMode hier wählen:
// DEADBAND (R,L), PI_SPRUNG (R,L), ENC_HAND(B)
// REGEL(R,L,B)
// RE, LI, BOTH
constexpr TestMode MODE = TestMode::REGEL;
constexpr Side     SIDE = Side::BOTH;

// >>> Normierter Fahrbefehl (ux, uy, uOmega)
// Wertebereich typ. -1 .. +1c
constexpr float CMD_UX = 1.0f;      // vor/zurück
constexpr float CMD_UY = 0.0f;      // seitlich (2-Rad: ignoriert)
constexpr float CMD_UOMEGA = 0.0f;  // drehen

// ============================================================
// --- Zeitbegrenzung für zeitlimitierte Tests (REGEL/SPEED/HI_KOPPEL) ---
// ============================================================
constexpr uint32_t REGELTEST_DAUER_MS = 5000;  // x Sekunden
uint32_t regeltest_start_ms = 0;
uint32_t enc_hand_start_ms = 0;
bool     time_limited = false;

// ============================================================
// --- ENC_HAND state (file-scope) ---
// ============================================================
static bool     enc_hand_init = false;
static long     enc_lastR = 0;
static long     enc_lastL = 0;
static uint32_t enc_t_last_ms = 0;

// Pi Sprung
static bool pi_printedDone = false;

// --- Printer ---
Printer prn;

// Einmalige Bindung der aktiven Seite (SIDE) auf konkrete Objekte.
// Vermeidet doppelte SIDE-Auswahl in setup()/loop() und hält die Zuordnung konsistent.
static SideBinding g_side;
static Wheel g_wheel;

// --- Setup ---
void setup() {
    Serial.begin(115200); while (!Serial) {}
    // Fehleingabe: MODE/Side-Konsistenz
    if (!modeAllowsBoth(MODE) && SIDE == Side::BOTH) {
        Serial.println(F("FEHLER: SIDE muss RE oder LI sein."));
        while (1) {}
    }

    // Hardware-Init (Option A): NUR über hardware_begin(...)
    hardware_begin(true);
    control_begin(true);
    tests_begin();

    g_side = { nullptr, nullptr, nullptr, nullptr };
    if (SIDE != Side::BOTH) {
        g_side = bindSide(SIDE);
        g_wheel = wheelFromSide(SIDE);
    }
    rad[Re].setTrim(0);
    rad[Li].setTrim(0);
    // kann raus?

    digitalWrite(STBY_PIN, HIGH);  // Treiber an

    const int8_t cmd_dir = cmd_dir_from_ux(CMD_UX);

    switch (MODE)
    {

    case TestMode::REGEL:
    {
        start_time_limited(regeltest_start_ms, time_limited);
        vehicle.setCmdNorm(CMD_UX, CMD_UY, CMD_UOMEGA);

        if (SIDE == Side::BOTH)
        {
            const float vSollLi = vehicle.getWheelSollMps(Side::LI);
            const float vSollRe = vehicle.getWheelSollMps(Side::RE);

            rad[Li].reset();
            rad[Re].reset();

            rad[Li].setSoll(vSollLi);
            rad[Re].setSoll(vSollRe);

            prn.regel_header(F("HiBeide"), Side::BOTH, rad[Re], rad[Li]);
        }
        else
        {
            const float vSoll = vehicle.getWheelSollMps(SIDE);
            Rad& r = *g_side.r;
            r.reset();
            r.setSoll(vSoll);

            if (SIDE == Side::RE) {
                prn.regel_header(F("Re"), Side::RE, rad[Re], rad[Li]);
            }
            else {
                prn.regel_header(F("Li"), Side::LI, rad[Re], rad[Li]);
            }
        }

        break;
    }

    case TestMode::DEADBAND:
    {
        constexpr uint8_t  startPWM = 40;
        constexpr uint8_t  endPWM = 200;
        constexpr uint8_t  step = 10;
        constexpr uint16_t measure_ms = 400;  // Messfenster pro PWM
        constexpr uint16_t settle_ms = 500;  // Abkling-/Stillstandszeit
        constexpr float    vThresh_mps = 0.0f; // Schwellwert für "läuft"

        prn.deadband_header(SIDE, startPWM, endPWM, step, measure_ms, settle_ms, vThresh_mps);

        if (SIDE == Side::BOTH) {
            Serial.println(F("FEHLER: DEADBAND nur RE oder LI (nicht BOTH)"));
            while (1) {}
        }

        tuner[g_wheel].setDir(cmd_dir);
        (void)tuner[g_wheel].deadbandScan(startPWM, endPWM, step, measure_ms, settle_ms, vThresh_mps);
        prn.deadband_done(SIDE);
        break;
    }

    case TestMode::PI_SPRUNG:
    {
        pi_printedDone = false;
        prn.pi_sprung_header(SIDE);
        AutoTuner& t = tuner[g_wheel];
        t.setPrinter(&prn);
        t.setDir(cmd_dir);
        t.resetIMC();
        break;
    }

    case TestMode::ENC_HAND:
    {
        enc_hand_start_ms = millis();
        // Motoren sicher still + gebremst
        achse_Hi.bremse(true);

        // Trims neutral (falls vorher gesetzt)
        rad[Re].setTrim(0);
        rad[Li].setTrim(0);

        // Messpfad sauber starten
        speed[Re].reset();
        speed[Li].reset();
        enc_hand_init = false;
        enc_lastR = 0;
        enc_lastL = 0;
        enc_t_last_ms = 0;

        // Header
        prn.enc_hand_header();
        break;
    }
    } // switch
}

// --- Loop ---
void loop() {
    const uint32_t now_ms = millis();

    // Zentrales Pollen ausser bei PI_SPRUNG
    if (MODE != TestMode::PI_SPRUNG) {
        speed[Li].pollEncoder(now_ms);
        speed[Re].pollEncoder(now_ms);
    }

    // Gemeinsame Zeitbegrenzung nur für zeitlimitierte Modi:
    //   REGEL / HI_KOPPEL
    if (time_limited && (now_ms - regeltest_start_ms >= REGELTEST_DAUER_MS)) {
        achse_Hi.bremse(true);

        time_limited = false;

        rad[Re].setTrim(0);
        rad[Li].setTrim(0);

        if (MODE == TestMode::REGEL) { prn.regel_done(); }
    }

    // MODE-Dispatch (Loop)
    switch (MODE) {

    case TestMode::REGEL:
        if (time_limited) {

            if (SIDE == Side::BOTH)
            {
                rad[Li].update(now_ms);
                rad[Re].update(now_ms);

                static uint32_t dbg_last_ms = 0;
                if (now_ms - dbg_last_ms >= DBG_INTERVAL_MS) {
                    dbg_last_ms = now_ms;

                    prn.regel_line_both(
                        (now_ms - regeltest_start_ms) * 0.001f,

                        rad[Re].soll(),
                        speed[Re].mps(),
                        rad[Re].lastPwm(),

                        rad[Li].soll(),
                        speed[Li].mps(),
                        rad[Li].lastPwm()
                    );
                }
            }
            else
            {
                SpeedWeg& s = *g_side.s;
                Rad& r = *g_side.r;
                r.update(now_ms);

                static uint32_t dbg_last_ms = 0;
                if (now_ms - dbg_last_ms >= DBG_INTERVAL_MS) {
                    dbg_last_ms = now_ms;

                    prn.regel_line(
                        (now_ms - regeltest_start_ms) * 0.001f,
                        r.soll(),
                        s.mps(),
                        r.lastPwm()
                    );
                }
            }
        }
        break;

    case TestMode::PI_SPRUNG: {
        // Polling für beide Räder

        const uint32_t now_us = micros();
        tuner[g_wheel].runIMCOpenStepWithPrints(now_us);

        if (!pi_printedDone) {
            if (tuner[g_wheel].isIMCOpenStepDone()) {
                prn.pi_sprung_done(SIDE);
                pi_printedDone = true;
            }
        }
    } break;

    case TestMode::ENC_HAND:
    {
        const long curR = speed[Re].counts_total();
        const long curL = speed[Li].counts_total();

        if (!enc_hand_init) {
            enc_lastR = curR;
            enc_lastL = curL;
            enc_hand_init = true;
            enc_t_last_ms = now_ms;   // Zeitanker setzen
            break;
        }

        if (now_ms - enc_t_last_ms >= 100) {
            enc_t_last_ms = now_ms;

            const long dR = curR - enc_lastR;
            const long dL = curL - enc_lastL;

            if (dR != 0 || dL != 0) {
                prn.enc_hand_line((now_ms - enc_hand_start_ms) * 0.001f,
                    dR, dL, curR, curL, speed[Re].mps(), speed[Li].mps());
            }

            enc_lastR = curR;
            enc_lastL = curL;
        }
    } break;

    case TestMode::DEADBAND:
        // Deadband läuft komplett in setup() (blocking scan) -> loop bleibt leer.
        break;
    }
}
