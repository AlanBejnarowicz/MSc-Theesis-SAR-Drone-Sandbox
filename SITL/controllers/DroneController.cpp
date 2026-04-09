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

// ── Heading: PD controller with AIS avoidance override ───────────
float DroneController::computeSteer()
{
    float avoidance = computeAvoidance();
    if (std::abs(avoidance) > 0.01f)
        return avoidance;

    float error  = deltaAngle(state.compass.heading, targetHeading);
    float dError = error - _lastHeadingError;
    _lastHeadingError = error;

    return (error * KP_HEADING) + (dError * KD_HEADING);
}

// ── Steer away from nearest contact inside safety radius ─────────
float DroneController::computeAvoidance()
{
    for (const auto& contact : state.ais.contacts)
    {
        if (contact.distance < MIN_AIS_DISTANCE)
        {
            float direction = (contact.bearing > 0.0f) ? -1.0f : 1.0f;
            float strength  = 1.0f - (contact.distance / MIN_AIS_DISTANCE);
            return direction * strength;
        }
    }
    return 0.0f;
}

// ── Shortest angular difference (-180 to 180) ────────────────────
float DroneController::deltaAngle(float from, float to)
{
    return std::fmod(to - from + 540.0f, 360.0f) - 180.0f;
}

long long DroneController::nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}
