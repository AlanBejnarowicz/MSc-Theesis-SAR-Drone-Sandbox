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
#include <iomanip>

// ── PID gains — same as your original DroneController ────────────
// Heading PID — steer output is -1..1
// Kp=0.15 saturates at 6.6° error. Lowered so output is proportional
// across the full turn range (180° error → steer=0.9, not clipped at 1.0)
#define SC_KP_HEADING   0.005f   // 180° error → 0.9 steer (not saturated)
#define SC_KI_HEADING   0.0001f
#define SC_KD_HEADING   0.002f
#define SC_KP_SPEED     0.5f
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
        long long now = nowMs();
        _lastRxMs = now;

        // FIX: first tick _lastKalmanMs==0 -> dt would be ms-since-epoch
        if (_lastKalmanMs == 0) _lastKalmanMs = now;
        float dt = (float)(now - _lastKalmanMs) / 1000.0f;
        _lastKalmanMs = now;

        // ── Kalman: init on first valid GPS fix ───────────────────
        if (!_kalmanInited && std::abs(state.gps.lat) > 0.001)
        {
            kalman.init(state.gps.lat, state.gps.lon, state.compass.heading);
            _kalmanInited = true;
        }

        if (_kalmanInited)
        {
            // dt clamped to 0.2s max inside KalmanFilter::predict
            kalman.predict(state.imu.accel_x, state.imu.accel_z, dt);

            // GPS — only when fix is valid
            if (std::abs(state.gps.lat) > 0.001)
                kalman.updateGPS(state.gps.lat, state.gps.lon,
                                 state.gps.accuracy);

            // Compass — 0 sigma handled inside KalmanFilter (defaults 2 deg)
            kalman.updateCompass(state.compass.heading,
                                 state.compass.noise_sigma);

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

        // Use raw GPS as fallback if Kalman not yet initialised
        // This is the key fix: !_kalmanInited was causing early return
        // with thr=0 even though we have valid GPS position
        double myLat = (_kalmanInited && kalman.lat != 0.0) ? kalman.lat : state.gps.lat;
        double myLon = (_kalmanInited && kalman.lon != 0.0) ? kalman.lon : state.gps.lon;
        float  myHdg = state.compass.heading;   // always raw compass

        if (!hasState || isLost || (myLat == 0.0 && myLon == 0.0))
        {
            cmd.throttle = 0.0f;
            cmd.steer    = 0.0f;
            return cmd;
        }

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

        // ── Heading PID ───────────────────────────────────────────
        float headingError = deltaAngle(myHdg, desiredHeading);
        float rawDError    = headingError - _lastHdgError;
        _lastHdgError      = headingError;

        // Light low-pass on derivative
        _filteredDError = 0.1f * rawDError + 0.9f * _filteredDError;

        // Integral — only accumulate when error is small (avoid windup during
        // large turns where the drone is physically spinning)
        if (std::abs(headingError) < 20.0f)
        {
            _integralError += headingError * 0.02f;
            _integralError  = std::clamp(_integralError, -15.0f, 15.0f);
        }
        else
        {
            _integralError *= 0.95f;  // bleed off integral during large turns
        }

        float steer = (headingError    * SC_KP_HEADING)
                    + (_integralError  * SC_KI_HEADING)
                    + (_filteredDError * SC_KD_HEADING);

        // ── Speed controller ──────────────────────────────────────
        // Always maintain minimum forward speed so rudder has effect.
        // Scale up toward full cruise as heading aligns.
        // headingFactor: 0° error → 1.0, 180° error → MIN_SPEED_FACTOR
        const float MIN_SPEED_FACTOR = 0.3f;   // 30% speed even while turning
        float abErr         = std::abs(headingError);
        float headingFactor = MIN_SPEED_FACTOR
                            + (1.0f - MIN_SPEED_FACTOR)
                            * (1.0f - std::min(abErr / 90.0f, 1.0f));
        float adjustedSpeed = desiredSpeed * headingFactor;
        float speedError    = adjustedSpeed - state.velocity.speed_ms;
        float throttle      = speedError * SC_KP_SPEED;

        cmd.throttle = std::clamp(throttle, 0.0f, SC_MAX_THROTTLE);  // forward only
        cmd.steer    = std::clamp(steer,   -SC_MAX_STEER, SC_MAX_STEER);
        const_cast<SwarmDroneController*>(this)->lastThrottle     = cmd.throttle;
        const_cast<SwarmDroneController*>(this)->lastSteer        = cmd.steer;
        const_cast<SwarmDroneController*>(this)->lastDesiredSpeed = desiredSpeed;

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
    float lastThrottle = 0.0f, lastSteer = 0.0f, lastDesiredSpeed = 0.0f;
    void printStatus() const
    {
        int nb = (int)loraSwarm.allNeighbours().size();
        float cov = patrol ? patrol->coveragePercent() : 0.0f;
        std::cout << "[Drone " << droneId << "]"
                  << "  hdg="   << (int)state.compass.heading << "°(raw)"
                  << "  spd="   << state.velocity.speed_knots << "kn(" << state.velocity.speed_ms << "ms)"
                  << "  pos=("  << kalman.lat << "," << kalman.lon << ")"
                  << "  posStd=" << (int)kalman.posStdM() << "m"
                  << "  hasTgt=" << trajectory.hasTarget()
                  << "  patrol=" << (patrol != nullptr)
                  << "  tgtHdg=" << (int)GridPatrol::bearingDeg(kalman.lat,kalman.lon,trajectory.targetLat(),trajectory.targetLon()) << "°"
                  << "  tgt=("  << std::fixed << std::setprecision(4)
                  << trajectory.targetLat() << "," << trajectory.targetLon() << ")"
                  << "  cov="   << (int)cov << "%"
                  << "  LoRa_nb=" << nb
                  << "  AIS="   << state.ais.contactCount
                  << "  tgtSpd=" << lastDesiredSpeed
                  << "  thr=" << lastThrottle << "  str=" << lastSteer
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