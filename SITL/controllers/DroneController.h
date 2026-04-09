#pragma once
#include "packets.h"
#include "config.h"
#include <chrono>
#include <cmath>
#include <algorithm>

// ================================================================
// DroneController  —  One instance per drone
// Replace computeThrottle() and computeSteer() with your INDI logic
// ================================================================

// ── Controller gains (simple heading+speed PID for now) ──────────
#define KP_HEADING          0.15f
#define KD_HEADING          0.05f
#define KP_SPEED            0.5f

// ── Safety ───────────────────────────────────────────────────────
#define MAX_THROTTLE        1.0f
#define MAX_STEER           1.0f
#define MIN_AIS_DISTANCE    20.0f       // Metres — trigger avoidance below this


class DroneController
{
public:
    // ── Identity ─────────────────────────────────────────────────
    int droneId = -1;

    // ── Last received state from Unity ───────────────────────────
    DroneStatePacket state;
    bool hasState        = false;
    bool isLost          = false;

    // ── Mission target ───────────────────────────────────────────
    float targetHeading  = 90.0f;    // Degrees 0-360
    float targetSpeed    = 8.0f;    // m/s

    // ── Public interface ─────────────────────────────────────────
    void init(int id);
    void onStateReceived(const DroneStatePacket& newState);
    void updateLostStatus();
    DroneCommandPacket tick();

private:
    int       _cmdSeq             = 0;
    float     _lastHeadingError   = 0.0f;
    long long _lastReceiveTime    = 0;

    float computeThrottle();
    float computeSteer();
    float computeAvoidance();
    float deltaAngle(float from, float to);
    long long nowMs();
};
