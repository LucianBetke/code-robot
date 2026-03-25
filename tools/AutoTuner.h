// AutoTuner.h
#pragma once
#include <Arduino.h>
#include "Motor.h"
#include "SpeedWeg.h"
#include "globals.h"
#include "ControlParams.h"

class Printer; // forward declaration

class AutoTuner {
public:
	struct IMCParams {
		uint8_t  u_dead;
		uint8_t  du1;
		uint8_t  du2;
		uint16_t sample_dt_ms;
		uint32_t hold_ms;
		uint32_t avg_last_ms;
		uint32_t settle_ms;

		// ACHTUNG:
		// Früher: rpm_still (rpm). Jetzt verwenden wir m/s.
		// Interpretation: "still" wenn |v| <= v_still_mps.
		float    v_still_mps;

		IMCParams()
			: u_dead(0), du1(60), du2(120),
			sample_dt_ms(20), hold_ms(2000),
			avg_last_ms(500), settle_ms(500),
			v_still_mps(0.02f) {   // 0.02 m/s als Startwert, ggf. anpassen
		}
	};

	struct IMCResult {
		float K[2];     // K1, K2  (Einheit: (m/s) pro PWM-Delta)
		float T[2];     // T1, T2  (s)
		float Kavg;
		float Tavg;
		float Kp;       // IMC-PI Kp (Normierung siehe Code; entspricht PWM/(m/s) "normiert" unten)
		float Ti;       // s
		float Ki;       // 1/s (für normierten Reglerausgang u in 0..1)
		bool  valid;

		IMCResult()
			: K{ 0.0f, 0.0f }, T{ 0.0f, 0.0f }, Kavg(0.0f), Tavg(0.0f),
			Kp(0.0f), Ti(0.0f), Ki(0.0f), valid(false) {
		}
	};

	AutoTuner(Motor& motor, SpeedWeg& speed, Wheel wheelIndex);
	AutoTuner(Motor& motor, SpeedWeg& speed, Wheel wheelIndex, bool useDefaultPrinter);

	void setPrinter(Printer* p) { _prn = p; }

	// Hinweis: now_us wird intern NICHT mehr als Zeitbasis benutzt.
	// SpeedWeg läuft in ms. Wir verwenden millis() konsequent.
	void runIMCOpenStepWithPrints(uint32_t now_us);
	void runIMCOpenStepWithPrints(const IMCParams& p, uint32_t now_us);

	bool isIMCOpenStepDone() const;
	const IMCResult& result() const;

	uint8_t deadbandScan(uint8_t startPWM = 40,
		uint8_t endPWM = 200,
		uint8_t step = 10,
		uint16_t measure_ms = 400,
		uint16_t settle_ms = 500,
		float vThresh_mps = 0.05f); // früher rpmThresh -> jetzt m/s

	void brakeToStill(uint16_t hold_ms = 300);
	void setDir(int8_t dir);
	void resetIMC();

private:
	Wheel _wheelIndex;
	static Printer _defaultPrinter;
	Printer* _prn;
	Motor& _motor;
	SpeedWeg& _speed;
	int8_t _dir;   // +1 vorwärts, -1 rückwärts

	struct StepData {
		static constexpr uint8_t MAX_SAMPLES = 25;

		uint8_t  du;
		float    y_start, y_end, dy;   // y = v [m/s]
		uint32_t t0_ms;
		float    avg_sum; uint32_t avg_n;

		uint16_t n;
		struct Sample { uint16_t y_x1000; } buf[MAX_SAMPLES]; // v * 1000
		uint32_t last_sample_ms;
		uint16_t sample_dt_ms;
		uint32_t t63_ms;
		uint16_t t63_idx;

		StepData()
			: du(0), y_start(0.0f), y_end(0.0f), dy(0.0f),
			t0_ms(0), avg_sum(0.0f), avg_n(0),
			n(0), last_sample_ms(0), sample_dt_ms(20),
			t63_ms(0), t63_idx(0xFFFF) {
		}
	};

	enum class ST { Idle, PreBrake, PreWait, StepStart, StepHold, Done };

	IMCParams _p;
	IMCResult _r;
	StepData  _steps[2];
	uint8_t   _stepIndex;
	ST        _st;
	uint32_t  _t_step_ms;
	uint32_t  _t_last_print;
	bool      _initialized;

	// Helfer
	static uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi);
	static uint8_t  pwm_from_delta(uint8_t u_dead, uint8_t du);
	void motor_apply_delta(uint8_t du);

	// y ist jetzt v [m/s]
	void hold_sample(StepData& s, uint32_t now_ms, float v_mps);

	static void compute_t63(StepData& s);

	// now_us bleibt als Parameter wegen deiner bestehenden Aufrufer,
	// intern wird aber millis() als Zeitbasis verwendet.
	void updateIMC(uint32_t now_us);

	void brakeToStill(uint16_t settle_ms, float v_eps_mps);
};
