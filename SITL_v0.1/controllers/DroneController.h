#pragma once
#include "packets.h"
#include "config.h"
#include <chrono>
#include <cmath>
#include <algorithm>

// ================================================================
// DroneController  —  One instance per drone
// ================================================================

// ── Controller gains (simple heading+speed PID for now) ──────────
#define KP_HEADING   0.15f
#define KI_HEADING   0.001f
#define KD_HEADING   0.001f  

#define KP_SPEED            0.8f

// ── Safety ───────────────────────────────────────────────────────
#define MAX_THROTTLE        1.0f
#define MAX_STEER           1.0f



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
    float targetHeading  = 30.0f;    // Degrees 0-360
    float targetSpeed    = 3.0f;    // m/s

    // ── Public interface ─────────────────────────────────────────
    void init(int id);
    void onStateReceived(const DroneStatePacket& newState);
    void updateLostStatus();
    DroneCommandPacket tick();

private:
    int       _cmdSeq             = 0;
    long long _lastReceiveTime    = 0;


    float _integralError  = 0.0f;
    float _lastHeadingError = 0.0f;
    float _filteredDError = 0.0f;


    float computeThrottle();
    float computeSteer();
    float computeAvoidance();
    float deltaAngle(float from, float to);
    long long nowMs();
};
