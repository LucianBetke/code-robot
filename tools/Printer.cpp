// Printer.cpp
#include "Printer.h"

// ============================================================
// Regeltest (CSV, Komma)
// ============================================================

void Printer::regel_header(
    const __FlashStringHelper* name,
    Side side,
    const Rad& re,
    const Rad& li)
{
    const PIParam pRe = re.getPI();
    const PIParam pLi = li.getPI();

    Serial.print(F("REGEL ("));
    Serial.print(name);
    Serial.println(F(")"));

    if (side == Side::BOTH)
    {
        Serial.print(F("KpRe=")); Serial.print(pRe.Kp, 2);
        Serial.print(F(",KiRe=")); Serial.print(pRe.Ki, 2);

        Serial.print(F(",KpLi=")); Serial.print(pLi.Kp, 2);
        Serial.print(F(",KiLi=")); Serial.println(pLi.Ki, 2);

        Serial.println(F("t_s,vSollRe,vIstRe,pwmRe,vSollLi,vIstLi,pwmLi"));
    }
    else
    {
        const PIParam p = (side == Side::RE) ? pRe : pLi;

        Serial.print(F("Kp=")); Serial.print(p.Kp, 2);
        Serial.print(F(",Ki=")); Serial.println(p.Ki, 2);

        Serial.println(F("t_s,vSoll,vIst,pwm"));
    }
}


void Printer::regel_line(float t_s, float soll, float ist, int16_t pwm) {
    Serial.print(t_s, 3);    Serial.print(',');
    Serial.print(soll, 3);   Serial.print(',');
    Serial.print(ist, 3);    Serial.print(',');
    Serial.println(pwm);
}

void Printer::regel_done() {
    Serial.println(F("REGEL_DONE"));
}

void Printer::regel_line_both(float t_s,
    float vSollRe, float vIstRe, int16_t pwmRe,
    float vSollLi, float vIstLi, int16_t pwmLi)
{
    Serial.print(t_s, 3);
    Serial.print(',');

    Serial.print(vSollRe, 3);
    Serial.print(',');
    Serial.print(vIstRe, 3);
    Serial.print(',');
    Serial.print(pwmRe);
    Serial.print(',');

    Serial.print(vSollLi, 3);
    Serial.print(',');
    Serial.print(vIstLi, 3);
    Serial.print(',');
    Serial.println(pwmLi);
}
// ============================================================
// Deadband Finder (CSV, Komma)
// ============================================================

void Printer::deadband_header(Side side,
    uint8_t startPWM, uint8_t endPWM, uint8_t step,
    uint16_t measure_ms, uint16_t settle_ms, float vThresh_mps)
{
    Serial.println();
    Serial.print(F("DEADBAND "));
    if (side == Side::RE) Serial.println(F("(RE)"));
    else if (side == Side::LI) Serial.println(F("(LI)"));
    else Serial.println(F("(BOTH)"));

    Serial.print(F("params,startPWM=")); Serial.print(startPWM);
    Serial.print(F(",endPWM="));         Serial.print(endPWM);
    Serial.print(F(",step="));           Serial.print(step);
    Serial.print(F(",measure_ms="));     Serial.print(measure_ms);
    Serial.print(F(",settle_ms="));      Serial.print(settle_ms);
    Serial.print(F(",vThresh_mps="));    Serial.println(vThresh_mps, 3);

    // optional: CSV-Spaltenkopf für Lines (nur falls du deadband_line nutzt)
    Serial.println(F("PWM,v_mps"));
}


void Printer::deadband_line(uint16_t pwm, float v_mps) {
    Serial.print(pwm);         Serial.print(',');
    Serial.println(v_mps, 3);
}

void Printer::deadband_result(Side side, uint8_t u_dead)
{
    Serial.print(F("DEADBAND_RESULT "));
    if (side == Side::RE) Serial.print(F("(RE) "));
    else if (side == Side::LI) Serial.print(F("(LI) "));
    else Serial.print(F("(BOTH) "));

    Serial.print(F("u_dead="));
    Serial.println(u_dead);
}
void Printer::deadband_done(Side side)
{
    Serial.print(F("DEADBAND_DONE "));
    if (side == Side::RE) Serial.println(F("(RE)"));
    else if (side == Side::LI) Serial.println(F("(LI)"));
    else Serial.println(F("(BOTH)"));
}

// ============================================================
// IMC / Schrittantwort (MINIMAL: nur KP, KI, T)
// ============================================================

void Printer::imc_header() {
    // absichtlich keine Ausgabe
}

void Printer::imc_step_start(uint8_t /*step*/, uint8_t /*u_dead*/, uint8_t /*du*/) {
    // absichtlich keine Ausgabe
}

void Printer::imc_progress(uint8_t /*step*/, uint32_t /*dt_ms*/, float /*v_mps*/) {
    // absichtlich keine Ausgabe
}

void Printer::imc_t63_hit(uint8_t /*step*/, uint32_t /*t63_ms*/, float /*v_at*/, float /*y63_est*/) {
    // absichtlich keine Ausgabe
}

void Printer::imc_step(uint8_t /*step*/, const AutoTunerTypes::StepDataView& /*s*/, float /*K*/) {
    // absichtlich keine Ausgabe
}

void Printer::imc_summary(const AutoTunerTypes::IMCResultView& r) {
    // Für DEINEN PIRegler: r.Kp und r.Ki sind normiert (u_norm 0..1), passend zu PIRegler.cpp
    // r.Tavg ist die gemittelte Zeitkonstante (aus 63%-Punkt)

    Serial.print(F("PI_TUNE,KP,"));
    Serial.print(r.Kp, 6);

    Serial.print(F(",KI,"));
    Serial.print(r.Ki, 6);

    Serial.print(F(",T_S,"));
    Serial.println(r.Tavg, 4);
}

void Printer::enc_hand_header() {
    Serial.println();
    Serial.println(F("== ENC_HAND: Encoder Handtest (Motor AUS) =="));
    Serial.println(F("# t_s, dTicksR, dTicksL, ticksR, ticksL, vR_mps, vL_mps"));
}

void Printer::enc_hand_line(float t_s,
    long dR, long dL,
    long ticksR, long ticksL,
    float vR_mps, float vL_mps) {
    Serial.print(t_s, 3);   Serial.print(F(","));
    Serial.print(dR);       Serial.print(F(","));
    Serial.print(dL);       Serial.print(F(","));
    Serial.print(ticksR);   Serial.print(F(","));
    Serial.print(ticksL);   Serial.print(F(","));
    Serial.print(vR_mps, 6); Serial.print(F(","));
    Serial.println(vL_mps, 6);
}

void Printer::pi_sprung_header(Side side)
{
    Serial.println();
    Serial.print(F("PI_SPRUNG "));
    if (side == Side::RE)        Serial.println(F("(RE)"));
    else if (side == Side::LI)   Serial.println(F("(LI)"));
    else                         Serial.println(F("(BOTH)"));

    // Wenn dein IMC/Step-Print schon eigene Header hat, lass es minimal.
    // Optional: eine kurze Spaltenzeile, wenn du später einheitlich parsen willst.
}

void Printer::pi_sprung_done(Side side)
{
    Serial.print(F("PI_SPRUNG_DONE "));
    if (side == Side::RE)        Serial.println(F("(RE)"));
    else if (side == Side::LI)   Serial.println(F("(LI)"));
    else                         Serial.println(F("(BOTH)"));
}