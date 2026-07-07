// ============================================================
// File: TelemetryPrinter.cpp
// Zweck:
//  - Serielle Diagnoseausgabe fuer Front-Nano
//  - WHEELS/CHASSIS/ODOM2-Ausgaben
//  - #CHASSISDBG bleibt aus Kompatibilitaetsgruenden erhalten.
//    Die frueheren Phi-Regelungsfelder werden jetzt immer als 0 ausgegeben,
//    weil die Vehicle-/Chassis-Regelung entfernt ist.
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
    void printSpeedCms(int16_t valueCms)
    {
        Serial.print((int)valueCms);
    }

    void printValue100(float value)
    {
        Serial.print(scaleFloatToInt100(value));
    }

    void printCountsFrame(
        uint32_t t_ms,
        int32_t voLiCnt,
        int32_t voReCnt,
        int32_t hiLiCnt,
        int32_t hiReCnt)
    {
#if PRINTER_ENABLE_COUNTS
        Serial.print(F("#CNTF,"));
        Serial.print(t_ms);          Serial.print(',');
        Serial.print((long)voLiCnt); Serial.print(',');
        Serial.print((long)voReCnt); Serial.print(',');
        Serial.print((long)hiLiCnt); Serial.print(',');
        Serial.println((long)hiReCnt);
#else
        (void)t_ms;
        (void)voLiCnt;
        (void)voReCnt;
        (void)hiLiCnt;
        (void)hiReCnt;
#endif
    }
}

void TelemetryPrinter::printInfo(VehicleController& vehicle, const RadControlConfig& cfg)
{
#if PRINTER_ENABLE_INFO
    Serial.print(F("#INFO,Raeder,Li,Kp100="));  Serial.print(scaleFloatToInt100(cfg.pi[Li].Kp));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(cfg.pi[Li].Ki));
    Serial.print(F(",dead="));                  Serial.print(cfg.deadPwm[Li]);
    Serial.print(F(",Re,Kp100="));              Serial.print(scaleFloatToInt100(cfg.pi[Re].Kp));
    Serial.print(F(",Ki100="));                 Serial.print(scaleFloatToInt100(cfg.pi[Re].Ki));
    Serial.print(F(",dead="));                  Serial.println(cfg.deadPwm[Re]);

    (void)vehicle;

    Serial.println(F("#INFO,VehicleRegelung,aus"));
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
        frame.t,
        frame.voLiCnt,
        frame.voReCnt,
        frame.hiLiCnt,
        frame.hiReCnt
    );
}

void TelemetryPrinter::printOdom2(
    uint16_t cmdpId,
    uint32_t t_ms,
    const MecanumOdometer& odom)
{
#if PRINTER_ENABLE_ODOM
    if (cmdpId == 0)
    {
        return;
    }

    Serial.print(F("#ODOM2,"));
    Serial.print((unsigned int)cmdpId);              Serial.print(',');
    Serial.print((unsigned long)t_ms);               Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.absCm()));  Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.xCm()));    Serial.print(',');
    Serial.print(scaleFloatToInt100(odom.yCm()));    Serial.print(',');
    Serial.println(scaleFloatToInt100(odom.phiDeg()));
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
    Serial.print(F("#CHASSISDBG,"));
    Serial.print(t_ms);                                                   Serial.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Keine Phi-Regelung mehr, deshalb immer 0.
    Serial.print(0);                                                      Serial.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Kein Phi-Reglerausgang mehr, deshalb immer 0.
    Serial.print(0);                                                      Serial.print(',');

    // Feld bleibt wegen Python-/Log-Kompatibilitaet erhalten.
    // Kein Radbeitrag aus Phi-Korrektur mehr, deshalb immer 0.
    Serial.print(0);                                                      Serial.print(',');

    // ungerundete hintere Rad-Sollwerte [cm/s] * 100
    Serial.print(scaleFloatToInt100(vehicle.getWheelSoll(HiLi)));         Serial.print(',');
    Serial.print(scaleFloatToInt100(vehicle.getWheelSoll(HiRe)));         Serial.print(',');

    // tatsaechlich per VSOL gesendete hintere Sollwerte [cm/s]
    printSpeedCms(hiLi_send_cms);                                         Serial.print(',');
    printSpeedCms(hiRe_send_cms);
    Serial.println();
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
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);                                            Serial.print(',');

    printSpeedCms(wheelMeasurements[Li].cmsInt());                 Serial.print(',');
    printSpeedCms(wheelMeasurements[Re].cmsInt());                 Serial.print(',');
    printSpeedCms(v2_ist_cms);                                     Serial.print(',');
    printSpeedCms(v3_ist_cms);                                     Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.vxIst()));             Serial.print(',');
    printSpeedCms(scaleRoundToInt16(vehicle.vyIst()));             Serial.print(',');
    printValue100(vehicle.wzIst());
    Serial.println();
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
    Serial.print(F("#CHASSIS,"));
    Serial.print(t_ms);                                            Serial.print(',');

    printSpeedCms(voLi_i_cms);                                     Serial.print(',');
    printSpeedCms(voRe_i_cms);                                     Serial.print(',');
    printSpeedCms(hiLi_i_cms);                                     Serial.print(',');
    printSpeedCms(hiRe_i_cms);                                     Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.vxIst()));             Serial.print(',');
    printSpeedCms(scaleRoundToInt16(vehicle.vyIst()));             Serial.print(',');
    printValue100(vehicle.wzIst());
    Serial.println();
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
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);                                             Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.getWheelSoll(VoLi)));   Serial.print(',');
    printSpeedCms(wheelMeasurements[Li].cmsInt());                  Serial.print(',');
    Serial.print(rad[Li].lastPwm());                                Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.getWheelSoll(VoRe)));   Serial.print(',');
    printSpeedCms(wheelMeasurements[Re].cmsInt());                  Serial.print(',');
    Serial.print(rad[Re].lastPwm());                                Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.getWheelSoll(HiLi)));   Serial.print(',');
    printSpeedCms(v2_ist_cms);                                      Serial.print(',');
    Serial.print(pwm2);                                             Serial.print(',');

    printSpeedCms(scaleRoundToInt16(vehicle.getWheelSoll(HiRe)));   Serial.print(',');
    printSpeedCms(v3_ist_cms);                                      Serial.print(',');
    Serial.println(pwm3);
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
    Serial.print(F("#WHEELS,"));
    Serial.print(t_ms);        Serial.print(',');

    printSpeedCms(voLi_s_cms); Serial.print(',');
    printSpeedCms(voLi_i_cms); Serial.print(',');
    Serial.print(voLi_pwm);    Serial.print(',');

    printSpeedCms(voRe_s_cms); Serial.print(',');
    printSpeedCms(voRe_i_cms); Serial.print(',');
    Serial.print(voRe_pwm);    Serial.print(',');

    printSpeedCms(hiLi_s_cms); Serial.print(',');
    printSpeedCms(hiLi_i_cms); Serial.print(',');
    Serial.print(hiLi_pwm);    Serial.print(',');

    printSpeedCms(hiRe_s_cms); Serial.print(',');
    printSpeedCms(hiRe_i_cms); Serial.print(',');
    Serial.println(hiRe_pwm);
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