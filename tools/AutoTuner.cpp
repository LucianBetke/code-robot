// AutoTuner.cpp
#include "AutoTuner.h"
#include <math.h>
#include "Printer.h"

// 1.0f = langsam, 0.5f ≈ doppelt so schnell, 0.3f = aggressiver
constexpr float IMC_LAMBDA_FACTOR = 0.45f;

Printer AutoTuner::_defaultPrinter;

AutoTuner::AutoTuner(Motor& m, SpeedWeg& speed, Wheel wheelIndex)
	: _wheelIndex(wheelIndex), _prn(0), _motor(m), _speed(speed),
	_stepIndex(0), _st(ST::Idle),
	_t_step_ms(0), _t_last_print(0),
	_initialized(false), _dir(+1) 
{
	_p.u_dead = (uint8_t)DEAD_PWM[_wheelIndex];
}

AutoTuner::AutoTuner(Motor& m, SpeedWeg& speed, Wheel wheelIndex, bool useDefaultPrinter)
	: _wheelIndex(wheelIndex), _prn(0), _motor(m), _speed(speed),
	_stepIndex(0), _st(ST::Idle),
	_t_step_ms(0), _t_last_print(0),
	_initialized(false), _dir(+1) 
{
	_p.u_dead = (uint8_t)DEAD_PWM[_wheelIndex];
	if (useDefaultPrinter) _prn = &_defaultPrinter;
}

void AutoTuner::setDir(int8_t dir)
{
	_dir = (dir >= 0) ? (int8_t)+1 : (int8_t)-1;
}

uint32_t AutoTuner::clamp_u32(uint32_t x, uint32_t lo, uint32_t hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}

uint8_t AutoTuner::pwm_from_delta(uint8_t u_dead, uint8_t du) {
	if (du == 0) return 0;
	const uint16_t u = static_cast<uint16_t>(u_dead) + du;
	return (u > 255) ? 255 : static_cast<uint8_t>(u);
}

void AutoTuner::motor_apply_delta(uint8_t du) {
	if (du == 0) {
		_motor.bremse(HIGH);
		return;
	}

	const uint8_t pwm = pwm_from_delta(_p.u_dead, du);

	if (_dir > 0) _motor.vor(pwm);
	else          _motor.rueck(pwm);
}

void AutoTuner::hold_sample(StepData& s, uint32_t now_ms, float v_mps) {
	if (s.n < StepData::MAX_SAMPLES && now_ms - s.last_sample_ms >= s.sample_dt_ms) {
		s.last_sample_ms = now_ms;
		const uint32_t dt = now_ms - s.t0_ms;

		// v * 1000 (m/s -> milli-m/s)
		const uint32_t y_x1000_u32 =
			clamp_u32(static_cast<uint32_t>(v_mps * 1000.0f + 0.5f), 0, 65535);

		s.buf[s.n++] = StepData::Sample{
			static_cast<uint16_t>(y_x1000_u32)
		};
	}
}

void AutoTuner::compute_t63(StepData& s) {
	constexpr float FACT = 0.632f;
	const float y63 = s.y_start + FACT * (s.y_end - s.y_start);
	const uint16_t y63_x1000 = static_cast<uint16_t>(y63 * 1000.0f + 0.5f);

	s.t63_ms = 0;
	s.t63_idx = 0xFFFF;

	
	for (uint16_t i = 0; i < s.n; ++i) {
		if (s.buf[i].y_x1000 >= y63_x1000) {
			s.t63_ms = i * s.sample_dt_ms;
			s.t63_idx = i;
			break;
		}
	}
}

void AutoTuner::runIMCOpenStepWithPrints(uint32_t now_us) {
	if (!_initialized && _st == ST::Idle) {
		_p = IMCParams(); // Defaults
		_p.u_dead = (uint8_t)DEAD_PWM[_wheelIndex];
	}
	runIMCOpenStepWithPrints(_p, now_us);
}

void AutoTuner::runIMCOpenStepWithPrints(const IMCParams& p, uint32_t now_us) {
	if (!_initialized) {
		_p = p;

		if (_p.avg_last_ms > _p.hold_ms) _p.avg_last_ms = _p.hold_ms;
		if (_p.du2 <= _p.du1) {
			Serial.println(F("[Warn] IMC: du2 <= du1 -> K2 wird 0. Bitte du2 > du1 wählen."));
		}

		for (int i = 0; i < 2; ++i) {
			_steps[i] = StepData();
			_steps[i].sample_dt_ms = _p.sample_dt_ms;
		}

		_steps[0].du = _p.du1;
		_steps[1].du = _p.du2;

		motor_apply_delta(0);
		_stepIndex = 0;
		_t_last_print = 0;
		_t_step_ms = millis();
		_st = ST::PreBrake;
		_initialized = true;
	}

	updateIMC(now_us);
}

void AutoTuner::updateIMC(uint32_t /*now_us*/) {

	// WICHTIG: SpeedWeg läuft in ms. Daher konsequent millis() nutzen.
	const uint32_t now = millis();
	_speed.pollEncoder(now);

	// Jetzt verwenden wir v [m/s] als Ausgang
	const float v = _speed.mps();

	StepData& s = _steps[_stepIndex];
	

	switch (_st) {
	case ST::Idle:
		return;

	case ST::PreBrake:
		_t_step_ms = now;
		_st = ST::PreWait;
		break;

	case ST::PreWait:
		// Nur VOR Step 1 warten wir auf Stillstand.
		if (_stepIndex == 0) {
			if (fabsf(v) > _p.v_still_mps) { _t_step_ms = now; break; }
			if (now - _t_step_ms >= _p.settle_ms) {
				s.y_start = v;            // Start aus dem Stillstand
				_st = ST::StepStart;
			}
		}
		else {
			// Für Step 2 KEIN Stillstand: kontinuierlicher Test
			// y_start(2) wird aus y_end(1) gesetzt, siehe unten.
			_st = ST::StepStart;
		}
		break;

	case ST::StepStart:
		if (_stepIndex == 1) {
			// Kontinuierlich: Step 2 startet vom Endwert von Step 1
			s.y_start = _steps[0].y_end;
		}
		motor_apply_delta(s.du);
		s.t0_ms = now;
		s.avg_sum = 0.0f; s.avg_n = 0;

		// Printer-Signaturen bleiben unverändert.
		// Inhaltlich sind die Werte jetzt v [m/s].
		if (_prn) _prn->imc_step_start(_stepIndex + 1, _p.u_dead, s.du);

		_st = ST::StepHold;
		break;

	case ST::StepHold:
		hold_sample(s, now, v);

		if (now - _t_last_print >= 100) {
			_t_last_print = now;
			if (_prn) _prn->imc_progress(_stepIndex + 1, now - s.t0_ms, v);
		}

		if (now - s.t0_ms >= _p.hold_ms - _p.avg_last_ms) {
			s.avg_sum += v; s.avg_n++;
		}

		if (now - s.t0_ms >= _p.hold_ms) {
			s.y_end = (s.avg_n ? (s.avg_sum / s.avg_n) : v);
			s.dy = s.y_end - s.y_start;

			

			compute_t63(s);
			Serial.print(F("t63_idx="));
			Serial.println(s.t63_idx);

			if (s.t63_idx == 0xFFFF) {
				Serial.println(F("[Warn] 63%-Marke nicht erreicht -> T ~ 0."));
			}
			else if (_prn) {
				const float y_at = s.buf[s.t63_idx].y_x1000 / 1000.0f;
				const float y63 = s.y_start + 0.632f * (s.y_end - s.y_start);
				_prn->imc_t63_hit(_stepIndex + 1, s.t63_ms, y_at, y63);
			}

			if (_stepIndex == 0) {
				// >>> WICHTIG: KEIN Bremsen/Absetzen zwischen Step 1 und 2
				_stepIndex = 1;
				_st = ST::StepStart;   // direkt in Step 2 übergehen
				// y_start(2) setzen wir in StepStart aus y_end(1)
			}
			else {
				// beide Steps fertig -> Kennwerte
				motor_apply_delta(0);

				// K: (m/s) pro PWM-Delta
				_r.K[0] = (_p.du1 > 0) ? (_steps[0].dy / static_cast<float>(_p.du1)) : 0.0f;

				const float du2d = (_p.du2 > _p.du1) ? static_cast<float>(_p.du2 - _p.du1) : 0.0f;
				_r.K[1] = (du2d > 0.0f) ? (_steps[1].dy / du2d) : 0.0f;

				_r.Kavg = 0.5f * (_r.K[0] + _r.K[1]);

				_r.T[0] = _steps[0].t63_ms * 0.001f;
				_r.T[1] = _steps[1].t63_ms * 0.001f;
				_r.Tavg = 0.5f * (_r.T[0] + _r.T[1]);

				// IMC: lambda = factor * T
				const float lambda = IMC_LAMBDA_FACTOR * _r.Tavg;

				// Hier berechnen wir Kp,Ki passend zu DEINEM PIRegler,
				// der intern u in 0..1 nutzt und danach PWM = u*255 macht.
				//
				// IMC-Formel liefert zunächst Kp in PWM/(m/s).
				// Danach normieren wir durch 255 -> Kp_norm für u∈[0..1].
				float Kp_pwm = 0.0f;
				if (_r.Kavg > 0.0f && lambda > 0.0f) {
					Kp_pwm = (_r.Tavg / (_r.Kavg * lambda));
				}

				_r.Ti = _r.Tavg;

				float Ki_pwm = 0.0f;
				if (_r.Ti > 0.0f) {
					Ki_pwm = (Kp_pwm / _r.Ti);
				}

				// Normierung für deinen PIRegler (u_norm -> PWM später):
				_r.Kp = Kp_pwm / 255.0f;
				_r.Ki = Ki_pwm / 255.0f;

				_r.valid = true;

				if (_prn) {
					_prn->imc_header();
					for (int i = 0; i < 2; ++i) {
						_prn->imc_step(
							i + 1,
							AutoTunerTypes::StepDataView{
								_steps[i].y_start,
								_steps[i].y_end,
								_steps[i].dy,
								_steps[i].t63_ms
							},
							_r.K[i]
						);
					}
					_prn->imc_summary(
						AutoTunerTypes::IMCResultView{
							_r.K[0], _r.K[1], _r.Kavg,
							_r.T[0], _r.T[1], _r.Tavg,
							_r.Kp, _r.Ti, _r.Ki, _r.valid
						}
					);
				}

				_st = ST::Done;
			}
		}
		break;

	case ST::Done:
		break;
	}
}

bool AutoTuner::isIMCOpenStepDone() const { return _st == ST::Done; }
const AutoTuner::IMCResult& AutoTuner::result() const { return _r; }

uint8_t AutoTuner::deadbandScan(uint8_t startPWM,
	uint8_t endPWM,
	uint8_t step,
	uint16_t measure_ms,
	uint16_t settle_ms,
	float vThresh_mps)
{
	uint8_t u_dead = 0;

	const uint32_t to_ms32 = static_cast<uint32_t>(measure_ms)
		+ static_cast<uint32_t>(settle_ms);
	const uint16_t to_ms = (to_ms32 > 500U)	? static_cast<uint16_t>(to_ms32): 500U;
	_speed.setTimeoutMs(to_ms);

	for (uint8_t pwm = startPWM; pwm <= endPWM; pwm += step) {
		if (_dir > 0) _motor.vor(pwm);
		else          _motor.rueck(pwm);

		const uint32_t t0 = millis();
		while (millis() - t0 < measure_ms) {
			_speed.pollEncoder(millis());
			delay(5);
		}

		const float v = _speed.mps();

		if (_prn) _prn->deadband_line(pwm, v);   // ✔ Rohdaten: erlaubt
		if (u_dead == 0 && v >= vThresh_mps) u_dead = pwm;

		_motor.bremse(HIGH);
		brakeToStill(settle_ms, 0.01f);
	}

	brakeToStill(300);
	_motor.bremse(HIGH);

	return u_dead;
}


void AutoTuner::brakeToStill(uint16_t hold_ms) {
	_motor.bremse(HIGH);
	delay(hold_ms);
	_motor.bremse(LOW);
}

void AutoTuner::brakeToStill(uint16_t settle_ms, float v_eps_mps) {
	const uint32_t tStop = millis();
	while (millis() - tStop < settle_ms) {
		_speed.pollEncoder(millis());  // WICHTIG: ms, nicht micros()
		if (fabsf(_speed.mps()) < v_eps_mps) break;
		delay(10);
	}
}
void AutoTuner::resetIMC()
{
	// Motor sicher stoppen
	motor_apply_delta(0);

	// IMC State komplett zurücksetzen
	_r = IMCResult();
	_p = IMCParams();
	_p.u_dead = (uint8_t)DEAD_PWM[_wheelIndex];

	for (int i = 0; i < 2; ++i) {
		_steps[i] = StepData();
		_steps[i].sample_dt_ms = _p.sample_dt_ms;
	}
	_steps[0].du = _p.du1;
	_steps[1].du = _p.du2;

	_stepIndex = 0;
	_st = ST::Idle;
	_t_step_ms = 0;
	_t_last_print = 0;
	_initialized = false;

	// SpeedPfad sauber
	_speed.reset();
}

