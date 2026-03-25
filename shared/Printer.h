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
};
