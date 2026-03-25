// Printer.h
#pragma once
#include <Arduino.h>
#include "TestTypes.h"   // für enum class Side
#include "Rad.h"
#include "ControlParams.h"

namespace AutoTunerTypes {
    struct StepDataView { float y_start, y_end, dy; uint32_t t63_ms; };
    struct IMCResultView {
        float K1, K2, Kavg, T1, T2, Tavg, Kp, Ti, Ki; bool valid;
    };
}

class Printer {
public:
    void regel_line_both(float t_s,
        float vSollRe, float vIstRe, int16_t pwmRe,
        float vSollLi, float vIstLi, int16_t pwmLi);

    // PI Sprung
    void pi_sprung_header(Side side);
    void pi_sprung_done(Side side);

    // Regeltest (CSV, Komma)
    void regel_header( const __FlashStringHelper* name, Side side, 
        const Rad& re, const Rad& li);

    void regel_line(float t_s, float soll, float ist, int16_t pwm);
    void regel_done();

    // Deadband (CSV, Komma)
    void deadband_header(Side side,uint8_t startPWM, uint8_t endPWM, uint8_t step,
        uint16_t measure_ms, uint16_t settle_ms, float vThresh_mps);
    void deadband_line(uint16_t pwm, float v_mps);
    void deadband_result(Side side, uint8_t u_dead);
    void deadband_done(Side side);

    // IMC / Schrittantwort (CSV, Komma)
    void imc_header();
    void imc_step_start(uint8_t step, uint8_t u_dead, uint8_t du);
    void imc_progress(uint8_t step, uint32_t dt_ms, float v_mps);
    void imc_t63_hit(uint8_t step, uint32_t t63_ms, float v_at, float y63_est);
    void imc_step(uint8_t step, const AutoTunerTypes::StepDataView& s, float K);
    void imc_summary(const AutoTunerTypes::IMCResultView& r);

    void enc_hand_header();
    void enc_hand_line(float t_s,
        long dR, long dL,
        long ticksR, long ticksL,
        float vR_mps, float vL_mps);
};
