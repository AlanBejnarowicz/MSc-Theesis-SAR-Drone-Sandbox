#include "DroneController.h"
#include <iostream>

// ================================================================
// DroneController.cpp
// ================================================================

void DroneController::init(int id)
{
    droneId = id;
}

void DroneController::onStateReceived(const DroneStatePacket& newState)
{
    state            = newState;
    hasState         = true;
    isLost           = false;
    _lastReceiveTime = nowMs();
}

void DroneController::updateLostStatus()
{
    if (hasState && (nowMs() - _lastReceiveTime > STATE_TIMEOUT_MS))
    {
        isLost = true;
        std::cout << "[Drone " << droneId << "] LOST — no state for "
                  << STATE_TIMEOUT_MS << "ms\n";
    }
}






DroneCommandPacket DroneController::tick()
{
    DroneCommandPacket cmd;
    cmd.droneId   = droneId;
    cmd.packetSeq = _cmdSeq++;

    if (!hasState || isLost)
    {
        cmd.throttle = 0.0f;
        cmd.steer    = 0.0f;
        return cmd;
    }

    cmd.throttle = std::clamp(computeThrottle(), -MAX_THROTTLE, MAX_THROTTLE);
    cmd.steer    = std::clamp(computeSteer(),    -MAX_STEER,    MAX_STEER);

    return cmd;
}













// ── Speed: P controller on speed error ───────────────────────────
float DroneController::computeThrottle()
{
    float speedError = targetSpeed - state.velocity.speed_ms;
    return speedError * KP_SPEED;
}

float DroneController::computeSteer()
{
    float error = deltaAngle(state.compass.heading, targetHeading);

    // Raw derivative
    float rawDError = error - _lastHeadingError;
    _lastHeadingError = error;

    // Low-pass filter on derivative — kills noise spikes
    // Alpha: 0.1 = heavy filtering, 0.5 = light filtering
    float alpha       = 0.1f;
    _filteredDError   = alpha * rawDError + (1.0f - alpha) * _filteredDError;

    // Integral with anti-windup
    _integralError   += error * 0.02f;
    _integralError    = std::clamp(_integralError, -30.0f, 30.0f);

    float output = (error          * KP_HEADING)
                 + (_integralError * KI_HEADING)
                 + (_filteredDError * KD_HEADING);

    return output;
}



float DroneController::deltaAngle(float from, float to)
{
    float diff = to - from;
    // Wrap to -180 to 180
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

long long DroneController::nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}
