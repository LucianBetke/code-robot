// ============================================================
// File: TelemetryPrinter.cpp
// Zweck:
//  - Serielle Diagnoseausgabe fuer Front-Nano
//  - WHEELS/CHASSIS/ODOM-Ausgaben
//  - #CHASSISDBG bleibt aus Kompatibilitaetsgruenden erhalten.
//    Die frueheren Phi-Regelungsfelder werden jetzt immer als 0 ausgegeben,
//    weil die Vehicle-/Chassis-Regelung entfernt ist.
//
// Aenderung fuer die Funkintegration:
//  - Ausgabeziel ist ein Print&, nicht mehr fest Serial.
//    Der Inhalt der Zeilen ist unveraendert.
//
// Wichtige Diagnosezeile:
//
// #CHASSISDBG,t,phiDeg100,wzPhi1000,dvPhi100,hiLiSoll100,hiReSoll100,hiLiSend,hiReSend
//
// Bedeutung jetzt:
//  - t              : Frame-Zeit [ms]
//  - phiDeg100      : immer 0, keine Phi-Regelung mehr
//  - wzPhi1000      : immer 0, kein Phi-Reglerausgang mehr
//  - dvPhi100       : immer 0, kein Radbeitrag der Phi-Korrektur mehr
//  - hiLiSoll100    : ungerundeter Sollwert HiLi [cm/s] * 100
//  - hiReSoll100    : ungerundeter Sollwert HiRe [cm/s] * 100
//  - hiLiSend       : tatsaechlich per VSOL gesendeter HiLi-Wert [cm/s]
//  - hiReSend       : tatsaechlich per VSOL gesendeter HiRe-Wert [cm/s]
// ============================================================

#include "TelemetryPrinter.h"

#include "src/MecanumOdometer.h"
#include "src/ScaleUtils.h"

namespace
{
    void printSpeedCms(Print& out, int16_t valueCms)
    {
        out.print((int)valueCms);
    }

    void printValue100(Print& out, float value)
    {
        out.print(scaleFloatToInt100(value));
    }

    void printCountsFrame(
        Print& out,
        uint32_t t_ms,
        int32_t voLiCnt,
        int32_t voReCnt,
        int32_t hiLiCnt,
        int32_t hiReCnt)
    {
#if PRINTER_ENABLE_COUNTS
        out.print(F("#CNTF,"));
        out.print(t_ms);          out.print(',');
        out.print((long)voLiCnt); out.print(',');
        out.print((long)voReCnt); out.print(',');
        out.print((long)hiLiCnt); out.print(',');
        out.println((long)hiReCnt);
#else
        (void)out;
        (void)t_ms;
        (void)voLiCnt;
        (void)voReCnt;
        (void)hiLiCnt;
        (void)hiReCnt;
#endif
    }
}

TelemetryPrinter::TelemetryPrinter(Print& out)
    : _out(out)
{
}

void TelemetryPrinter::printInfo(VehicleController& vehicle, const RadControlConfig& cfg)
{
#if PRINTER_ENABLE_INFO
    _out.print(F("#INFO,Raeder,Li,Kp100="));  _out.print(scaleFloatToInt100(cfg.pi[Li].Kp));
    _out.print(F(",Ki100="));                 _out.print(scaleFloatToInt100(cfg.pi[Li].Ki));
    _out.print(F(",dead="));                  _out.print(cfg.deadPwm[Li]);
    _out.print(F(",Re,Kp100="));              _out.print(scaleFloatToInt100(cfg.pi[Re].Kp));
    _out.print(F(",Ki100="));                 _out.print(scaleFloatToInt100(cfg.pi[Re].Ki));
    _out.print(F(",dead="));                  _out.println(cfg.deadPwm[Re]);

    (void)vehicle;

    _out.println(F("#INFO,VehicleRegelung,aus"));
#else
    (void)vehicle;
    (void)cfg;
#endif
}

void TelemetryPrinter::printCompletedFrame(
    VehicleController& vehicle,
    const RearPendingFrame& frame,
    int16_t hiLi_i_cms,
    int16_t hiRe_i_cms,
    int16_t hiLi_pwm,
    int16_t hiRe_pwm)
{
#if defined(PRINTER_MODE_CHASSIS) && PRINTER_ENABLE_CHASSIS
    printFrame(
        vehicle,
        frame.t,
        frame.voLi_i_cms,
        frame.voRe_i_cms,
        hiLi_i_cms,
        hiRe_i_cms
    );
#else
    (void)vehicle;
#endif

#if defined(PRINTER_MODE_RAEDER) && PRINTER_ENABLE_WHEELS
    printFrame(
        frame.t,

        frame.voLi_s_cms,
        frame.voLi_i_cms,
        frame.voLi_pwm,

        frame.voRe_s_cms,
        frame.voRe_i_cms,
        frame.voRe_pwm,

        frame.hiLi_s_cms,
        hiLi_i_cms,
        hiLi_pwm,

        frame.hiRe_s_cms,
        hiRe_i_cms,
        hiRe_pwm
    );
#else
    (void)hiLi_i_cms;
    (void)hiRe_i_cms;
    (void)hiLi_pwm;
    (void)hiRe_pwm;
#endif

    printCountsFrame(
        _out,
        frame.t,
        frame.voLiCnt,
        frame.voReCnt,
        frame.hiLiCnt,
        frame.hiReCnt
    );
}

void TelemetryPrinter::printOdom(
    uint16_t cmdpId,
    uint32_t t_ms,
    const MecanumOdometer& odom)
{
#if PRINTER_ENABLE_ODOM
    if (cmdpId == 0)
    {
        return;
    }

    _out.print(F("#ODOM,"));
    _out.print((unsigned int)cmdpId);              _out.print(',');
    _out.print((unsigned long)t_ms);               _out.print(',');
    _out.print(scaleFloatToInt100(odom.absCm()));  _out.print(',');
    _out.print(scaleFloatToInt100(odom.xCm()));    _out.print(',');
    _out.print(scaleFloatToInt100(odom.yCm()));    _out.print(',');
    _out.println(scaleFloatToInt100(odom.phiDeg()));
#else
    (void)cmdpId;
    (void)t_ms;
    (void)odom;
#endif
}

#ifdef PRINTER_MODE_CHASSIS

void TelemetryPrinter::printChassisDebug(
    VehicleController& vehicle,
    uint32_t t_ms,
    int16_t hiLi_send_cms,
    int16_t hiRe_send_cms)
{
#if PRINTER_ENABLE_CHASSIS
    _out.print(F("#CHASSISDBG,"));
    _out.print(t_ms);                                                     _out.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Keine Phi-Regelung mehr, deshalb immer 0.
    _out.print(0);                                                        _out.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Kein Phi-Reglerausgang mehr, deshalb immer 0.
    _out.print(0);                                                        _out.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Kein Radbeitrag aus Phi-Korrektur mehr, deshalb immer 0.
    _out.print(0);                                                        _out.print(',');

    // ungerundete hintere Rad-Sollwerte [cm/s] * 100
    _out.print(scaleFloatToInt100(vehicle.getWheelSoll(HiLi)));           _out.print(',');
    _out.print(scaleFloatToInt100(vehicle.getWheelSoll(HiRe)));           _out.print(',');

    // tatsaechlich per VSOL gesendete hintere Sollwerte [cm/s]
    printSpeedCms(_out, hiLi_send_cms);                                   _out.print(',');
    printSpeedCms(_out, hiRe_send_cms);
    _out.println();
#else
    (void)vehicle;
    (void)t_ms;
    (void)hiLi_send_cms;
    (void)hiRe_send_cms;
#endif
}

void TelemetryPrinter::printWheels(
    VehicleController& vehicle,
    int16_t v2_ist_cms,
    int16_t v3_ist_cms,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_CHASSIS
    _out.print(F("#CHASSIS,"));
    _out.print(t_ms);                                              _out.print(',');

    printSpeedCms(_out, wheelMeasurements[Li].cmsInt());           _out.print(',');
    printSpeedCms(_out, wheelMeasurements[Re].cmsInt());           _out.print(',');
    printSpeedCms(_out, v2_ist_cms);                               _out.print(',');
    printSpeedCms(_out, v3_ist_cms);                               _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.vxIst()));       _out.print(',');
    printSpeedCms(_out, scaleRoundToInt16(vehicle.vyIst()));       _out.print(',');
    printValue100(_out, vehicle.wzIst());
    _out.println();
#else
    (void)vehicle;
    (void)v2_ist_cms;
    (void)v3_ist_cms;
    (void)t_ms;
#endif
}

void TelemetryPrinter::printFrame(
    VehicleController& vehicle,
    uint32_t t_ms,
    int16_t voLi_i_cms,
    int16_t voRe_i_cms,
    int16_t hiLi_i_cms,
    int16_t hiRe_i_cms)
{
#if PRINTER_ENABLE_CHASSIS
    _out.print(F("#CHASSIS,"));
    _out.print(t_ms);                                              _out.print(',');

    printSpeedCms(_out, voLi_i_cms);                               _out.print(',');
    printSpeedCms(_out, voRe_i_cms);                               _out.print(',');
    printSpeedCms(_out, hiLi_i_cms);                               _out.print(',');
    printSpeedCms(_out, hiRe_i_cms);                               _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.vxIst()));       _out.print(',');
    printSpeedCms(_out, scaleRoundToInt16(vehicle.vyIst()));       _out.print(',');
    printValue100(_out, vehicle.wzIst());
    _out.println();
#else
    (void)vehicle;
    (void)t_ms;
    (void)voLi_i_cms;
    (void)voRe_i_cms;
    (void)hiLi_i_cms;
    (void)hiRe_i_cms;
#endif
}

#endif // PRINTER_MODE_CHASSIS

#ifdef PRINTER_MODE_RAEDER

void TelemetryPrinter::printWheels(
    VehicleController& vehicle,
    int16_t v2_ist_cms,
    int16_t v3_ist_cms,
    int16_t pwm2,
    int16_t pwm3,
    uint32_t t_ms)
{
#if PRINTER_ENABLE_WHEELS
    _out.print(F("#WHEELS,"));
    _out.print(t_ms);                                                     _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.getWheelSoll(VoLi)));   _out.print(',');
    printSpeedCms(_out, wheelMeasurements[Li].cmsInt());                  _out.print(',');
    _out.print(rad[Li].lastPwm());                                        _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.getWheelSoll(VoRe)));   _out.print(',');
    printSpeedCms(_out, wheelMeasurements[Re].cmsInt());                  _out.print(',');
    _out.print(rad[Re].lastPwm());                                        _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.getWheelSoll(HiLi)));   _out.print(',');
    printSpeedCms(_out, v2_ist_cms);                                      _out.print(',');
    _out.print(pwm2);                                                     _out.print(',');

    printSpeedCms(_out, scaleRoundToInt16(vehicle.getWheelSoll(HiRe)));   _out.print(',');
    printSpeedCms(_out, v3_ist_cms);                                      _out.print(',');
    _out.println(pwm3);
#else
    (void)vehicle;
    (void)v2_ist_cms;
    (void)v3_ist_cms;
    (void)pwm2;
    (void)pwm3;
    (void)t_ms;
#endif
}

void TelemetryPrinter::printFrame(
    uint32_t t_ms,
    int16_t voLi_s_cms,
    int16_t voLi_i_cms,
    int16_t voLi_pwm,
    int16_t voRe_s_cms,
    int16_t voRe_i_cms,
    int16_t voRe_pwm,
    int16_t hiLi_s_cms,
    int16_t hiLi_i_cms,
    int16_t hiLi_pwm,
    int16_t hiRe_s_cms,
    int16_t hiRe_i_cms,
    int16_t hiRe_pwm)
{
#if PRINTER_ENABLE_WHEELS
    _out.print(F("#WHEELS,"));
    _out.print(t_ms);                _out.print(',');

    printSpeedCms(_out, voLi_s_cms); _out.print(',');
    printSpeedCms(_out, voLi_i_cms); _out.print(',');
    _out.print(voLi_pwm);            _out.print(',');

    printSpeedCms(_out, voRe_s_cms); _out.print(',');
    printSpeedCms(_out, voRe_i_cms); _out.print(',');
    _out.print(voRe_pwm);            _out.print(',');

    printSpeedCms(_out, hiLi_s_cms); _out.print(',');
    printSpeedCms(_out, hiLi_i_cms); _out.print(',');
    _out.print(hiLi_pwm);            _out.print(',');

    printSpeedCms(_out, hiRe_s_cms); _out.print(',');
    printSpeedCms(_out, hiRe_i_cms); _out.print(',');
    _out.println(hiRe_pwm);
#else
    (void)t_ms;
    (void)voLi_s_cms;
    (void)voLi_i_cms;
    (void)voLi_pwm;
    (void)voRe_s_cms;
    (void)voRe_i_cms;
    (void)voRe_pwm;
    (void)hiLi_s_cms;
    (void)hiLi_i_cms;
    (void)hiLi_pwm;
    (void)hiRe_s_cms;
    (void)hiRe_i_cms;
    (void)hiRe_pwm;
#endif
}

#endif // PRINTER_MODE_RAEDER
