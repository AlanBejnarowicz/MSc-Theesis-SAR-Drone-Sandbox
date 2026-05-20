#pragma once
// ================================================================
// SpeedP.h  —  Simple P controller for throttle
//
// Input:  current speed (m/s), desired speed (m/s)
// Output: throttle (0.0 .. 1.0)  — forward only
// ================================================================

#include <algorithm>

class SpeedP
{
public:
    float Kp = 0.4f;   // tune: higher = more aggressive throttle response

    // Returns throttle 0..1 (forward only — no reverse)
    float update(float currentSpeed_ms, float desiredSpeed_ms)
    {
        float err = desiredSpeed_ms - currentSpeed_ms;
        return std::clamp(err * Kp, 0.f, 1.f);
    }
};
