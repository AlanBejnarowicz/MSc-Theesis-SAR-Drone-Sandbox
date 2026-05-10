#pragma once
// ================================================================
// SwarmDroneController.h  —  Full swarm agent: one instance per drone
//
// Layers (bottom → top):
//   1. KalmanFilter      — sensor fusion (GPS/IMU/Compass/LoRa ToF)
//   2. DroneController   — PID speed + heading (unchanged from yours)
//   3. TrajectoryController — target heading/speed with avoidance
//   4. GridPatrol        — patrol cell assignment (shared object)
//   5. LoraSwarm         — inter-drone communication
//
// Only GridPatrol is shared (pointer) — all others are per-drone.
// ================================================================

#include "packets.h"
#include "config.h"
#include "KalmanFilter.h"
#include "TrajectoryController.h"
#include "LoraSwarm.h"
#include "GridPatrol.h"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

// ── PID gains — same as your original DroneController ────────────
#define SC_KP_HEADING   0.15f
#define SC_KI_HEADING   0.001f
#define SC_KD_HEADING   0.08f    // increased from 0.001 for less overshoot
#define SC_KP_SPEED     0.8f
#define SC_MAX_THROTTLE 1.0f
#define SC_MAX_STEER    1.0f

class SwarmDroneController
{
public:
    // ── Identity ─────────────────────────────────────────────────
    int droneId = -1;

    // ── Shared patrol grid (set once before first tick) ───────────
    GridPatrol* patrol = nullptr;

    // ── Public state ─────────────────────────────────────────────
    DroneStatePacket state;
    bool hasState = false;
    bool isLost   = false;

    KalmanFilter        kalman;
    LoraSwarm           loraSwarm;
    TrajectoryController trajectory;

    // ── Init ─────────────────────────────────────────────────────
    void init(int id)
    {
        droneId = id;
    }

    // ── Called when Unity sends a state packet ───────────────────
    void onStateReceived(const DroneStatePacket& newState)
    {
        state    = newState;
        hasState = true;
        isLost   = false;
        _lastRxMs = nowMs();

        float dt = (float)(nowMs() - _lastKalmanMs) / 1000.0f;
        _lastKalmanMs = nowMs();

        // ── Kalman: predict with IMU ──────────────────────────────
        if (!_kalmanInited && state.gps.lat != 0.0)
        {
            kalman.init(state.gps.lat, state.gps.lon, state.compass.heading);
            _kalmanInited = true;
        }

        if (_kalmanInited)
        {
            kalman.predict(state.imu.accel_x, state.imu.accel_z, dt);

            // GPS update
            if (state.gps.lat != 0.0)
                kalman.updateGPS(state.gps.lat, state.gps.lon,
                                 state.gps.accuracy);

            // Compass update
            kalman.updateCompass(state.compass.heading,
                                 state.compass.noise_sigma + 1.0f);

            // LoRa ToF range updates
            for (auto& n : state.lora.neighbours)
            {
                if (n.distanceMetres < 1.0f) continue;
                auto& nb = loraSwarm.allNeighbours();
                auto it = nb.find(n.nodeId);
                if (it != nb.end() && it->second.lat != 0.0)
                {
                    kalman.updateLoRaRange(it->second.lat, it->second.lon,
                                          n.distanceMetres, 3.0f);
                }
            }
        }

        // ── Process LoRa swarm messages ───────────────────────────
        loraSwarm.processLoraBlock(state.lora, nowMs());
    }

    // ── Check lost status ─────────────────────────────────────────
    void updateLostStatus()
    {
        if (hasState && (nowMs() - _lastRxMs > STATE_TIMEOUT_MS))
        {
            isLost = true;
            if (patrol) patrol->releaseTarget(droneId);
        }
    }

    // ── Main tick — returns command for Unity ─────────────────────
    DroneCommandPacket tick()
    {
        DroneCommandPacket cmd;
        cmd.droneId   = droneId;
        cmd.packetSeq = _cmdSeq++;

        if (!hasState || isLost || !_kalmanInited)
        {
            cmd.throttle = 0.0f;
            cmd.steer    = 0.0f;
            return cmd;
        }

        double myLat = kalman.lat;
        double myLon = kalman.lon;
        float  myHdg = kalman.heading;

        // ── Patrol: get/update target cell ────────────────────────
        if (patrol)
        {
            double tLat, tLon;
            if (patrol->getTarget(droneId, myLat, myLon, tLat, tLon))
                trajectory.setTarget(tLat, tLon);
            else
                trajectory.clearTarget();
        }

        // ── Trajectory + avoidance → desired heading & speed ─────
        float desiredHeading, desiredSpeed;
        trajectory.update(myLat, myLon, myHdg,
                          state.ais, loraSwarm,
                          desiredHeading, desiredSpeed);

        // ── PID heading controller ────────────────────────────────
        float headingError = deltaAngle(myHdg, desiredHeading);
        float rawDError    = headingError - _lastHdgError;
        _lastHdgError      = headingError;

        float alpha        = 0.1f;
        _filteredDError    = alpha * rawDError + (1.0f - alpha) * _filteredDError;

        _integralError    += headingError * 0.02f;
        _integralError     = std::clamp(_integralError, -30.0f, 30.0f);

        float steer = (headingError    * SC_KP_HEADING)
                    + (_integralError  * SC_KI_HEADING)
                    + (_filteredDError * SC_KD_HEADING);

        // ── PID speed controller ──────────────────────────────────
        float speedError = desiredSpeed - state.velocity.speed_ms;
        float throttle   = speedError * SC_KP_SPEED;

        cmd.throttle = std::clamp(throttle, -SC_MAX_THROTTLE, SC_MAX_THROTTLE);
        cmd.steer    = std::clamp(steer,    -SC_MAX_STEER,    SC_MAX_STEER);

        // ── LoRa broadcast (every N ticks) ────────────────────────
        if (loraSwarm.shouldBroadcastThisTick())
        {
            int targetCell = -1;
            int cellsVisited = patrol ? patrol->droneProgress(droneId) : 0;

            // Get current target cell ID from trajectory
            // (the GridPatrol holds it internally)

            cmd.loraBroadcast = loraSwarm.encodeBeacon(
                droneId, myLat, myLon, myHdg,
                state.velocity.speed_ms,
                targetCell, cellsVisited,
                kalman.posStdM(), isLost,
                state.gps.lat != 0.0);
        }

        return cmd;
    }

    // ── Status line for console ───────────────────────────────────
    void printStatus() const
    {
        int nb = (int)loraSwarm.allNeighbours().size();
        float cov = patrol ? patrol->coveragePercent() : 0.0f;
        std::cout << "[Drone " << droneId << "]"
                  << "  hdg="   << (int)kalman.heading << "°"
                  << "  spd="   << state.velocity.speed_knots << "kn"
                  << "  pos=("  << kalman.lat << "," << kalman.lon << ")"
                  << "  posStd=" << (int)kalman.posStdM() << "m"
                  << "  cov="   << (int)cov << "%"
                  << "  LoRa_nb=" << nb
                  << "  AIS="   << state.ais.contactCount
                  << (isLost ? "  [LOST]" : "")
                  << "\n";
    }

private:
    int       _cmdSeq          = 0;
    long long _lastRxMs        = 0;
    long long _lastKalmanMs    = 0;
    bool      _kalmanInited    = false;

    // PID state
    float _integralError  = 0.0f;
    float _lastHdgError   = 0.0f;
    float _filteredDError = 0.0f;

    static float deltaAngle(float from, float to)
    {
        float d = to - from;
        while (d >  180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        return d;
    }

    static long long nowMs()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }
};
