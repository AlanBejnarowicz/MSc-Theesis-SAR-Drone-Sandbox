#pragma once
// ================================================================
// HeadingPID.h  —  Simple PD heading controller for surface vessel
//
// Input:  current heading (deg), desired heading (deg)
// Output: steer command  (-1.0 .. +1.0)
//
// Notes:
//   - Integral is intentionally omitted for now.
//     A surface vessel with a good D term rarely needs I.
//     Add it once P and D are tuned.
//   - "Heading factor" scales throttle down when far off-course
//     so the vessel turns before accelerating.
// ================================================================

#include <cmath>
#include <algorithm>

class HeadingPID
{
public:
    // ── Gains — tune these ────────────────────────────────────────
    float Kp = 0.008f;   // proportional  (180° err → steer=1.44, clipped to 1)
    float Kd = 0.003f;   // derivative    (damps oscillation)

    // ── Speed params ──────────────────────────────────────────────
    float cruiseSpeed   = 10.0f;   // m/s target at full alignment
    float minSpeedFactor= 0.25f;  // fraction of cruise when heading=180° off

    // ── Update — call every control tick ─────────────────────────
    // Returns steer (-1..1) and writes desiredSpeed_ms
    float update(float currentHdg, float targetHdg,
                 float dt, float& desiredSpeed_ms)
    {
        float err = _deltaAngle(currentHdg, targetHdg);

        // Derivative (raw, then low-pass filtered)
        float rawD = (dt > 0.f) ? (err - _lastErr) / dt : 0.f;
        _filtD     = 0.1f * rawD + 0.9f * _filtD;
        _lastErr   = err;

        float steer = Kp * err + Kd * _filtD;

        // Speed: full cruise when aligned, minSpeedFactor when 180° off
        float alignFactor = 1.f - (1.f - minSpeedFactor)
                            * std::min(std::abs(err) / 180.f, 1.f);
        desiredSpeed_ms = cruiseSpeed * alignFactor;

        return std::clamp(steer, -1.f, 1.f);
    }

    void reset() { _lastErr = 0.f; _filtD = 0.f; }

    float lastError() const { return _lastErr; }

private:
    float _lastErr = 0.f;
    float _filtD   = 0.f;

    static float _deltaAngle(float from, float to)
    {
        float d = to - from;
        while (d >  180.f) d -= 360.f;
        while (d < -180.f) d += 360.f;
        return d;
    }
};
