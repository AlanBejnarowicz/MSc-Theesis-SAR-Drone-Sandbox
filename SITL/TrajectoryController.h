#pragma once
// ================================================================
// TrajectoryController.h  —  High-level path + avoidance layer
//
// Responsibilities:
//   1. Accept a geo target (lat/lon) from GridPatrol
//   2. Compute desired heading and speed toward target
//   3. Apply AIS vessel avoidance (pass-ahead / slow-down)
//   4. Apply drone-drone avoidance via LoRa neighbour list
//   5. Output targetHeading + targetSpeed to DroneController
//
// Avoidance strategy:
//   - Ships (AIS): "give way" — if bearing < AVOID_HALF_ANG degrees
//     relative to bow and closing, steer away and reduce speed
//   - Drones: symmetric — both turn right (starboard) if within 
//     DRONE_AVOID_M metres (like maritime colregs rule 15)
// ================================================================

#include "packets.h"
#include "LoraSwarm.h"
#include "GridPatrol.h"
#include <cmath>
#include <vector>
#include <algorithm>

class TrajectoryController
{
public:
    // ── Tuning ────────────────────────────────────────────────────
    static constexpr float CRUISE_SPEED_MS      = 3.0f;   // normal patrol
    static constexpr float APPROACH_SPEED_MS    = 1.5f;   // near cell centre
    static constexpr float APPROACH_DIST_M      = 120.0f; // slow down radius

    // AIS avoidance
    static constexpr float AIS_ALERT_M         = 300.0f;  // start avoiding
    static constexpr float AIS_DANGER_M        = 80.0f;   // hard stop
    static constexpr float AIS_AVOID_STEER_DEG = 45.0f;   // how hard we turn
    static constexpr float AIS_AVOID_HALF_ANG  = 90.0f;   // front cone width/2

    // Drone avoidance
    static constexpr float DRONE_ALERT_M       = 150.0f;
    static constexpr float DRONE_DANGER_M      = 40.0f;
    static constexpr float DRONE_AVOID_DEG     = 60.0f;

    // ── Set patrol target ─────────────────────────────────────────
    void setTarget(double lat, double lon)
    {
        targetLat_ = lat;
        targetLon_ = lon;
        hasTarget_ = true;
    }

    void clearTarget() { hasTarget_ = false; }

    // ── Main update — call every tick ─────────────────────────────
    // Returns desired heading (0-360) and speed (m/s) for DroneController
    void update(double myLat, double myLon, float myHeading,
                const AIS_Block& ais,
                const LoraSwarm& swarm,
                float& outHeading, float& outSpeed)
    {
        // Default: cruise toward patrol target
        if (!hasTarget_) {
            outHeading = myHeading;
            outSpeed   = 0.0f;
            return;
        }

        float distToTarget = GridPatrol::distMetres(myLat, myLon,
                                                    targetLat_, targetLon_);
        float bearingToTarget = GridPatrol::bearingDeg(myLat, myLon,
                                                        targetLat_, targetLon_);
        outHeading = bearingToTarget;
        outSpeed   = (distToTarget < APPROACH_DIST_M)
                     ? APPROACH_SPEED_MS : CRUISE_SPEED_MS;

        // ── AIS avoidance ─────────────────────────────────────────
        float aisSteer = 0.0f;
        float aisSpeedFactor = 1.0f;

        for (auto& contact : ais.contacts)
        {
            if (contact.distance > AIS_ALERT_M) continue;

            // Relative bearing to AIS contact from our heading
            float relBearing = contact.bearing; // already relative in packets.h

            // Only care about contacts roughly ahead
            if (std::abs(relBearing) > AIS_AVOID_HALF_ANG) continue;

            // Closing check: AIS heading toward us?
            // (simple: if contact speed is significant)
            bool closing = (contact.speed_knots > 0.5f);

            float proximity = 1.0f - (contact.distance / AIS_ALERT_M);

            if (contact.distance < AIS_DANGER_M)
            {
                // Emergency: stop and turn hard away
                outSpeed = 0.0f;
                aisSteer += (relBearing >= 0 ? -AIS_AVOID_STEER_DEG
                                             :  AIS_AVOID_STEER_DEG)
                            * 2.0f;
            }
            else
            {
                // Gradual avoidance: steer away proportional to proximity
                aisSteer += (relBearing >= 0 ? -AIS_AVOID_STEER_DEG
                                             :  AIS_AVOID_STEER_DEG)
                            * proximity;
                if (closing)
                    aisSpeedFactor = std::min(aisSpeedFactor, proximity);
            }
        }

        outHeading += std::clamp(aisSteer, -AIS_AVOID_STEER_DEG * 2,
                                            AIS_AVOID_STEER_DEG * 2);
        outSpeed   *= aisSpeedFactor;

        // ── Drone-drone avoidance ─────────────────────────────────
        auto nearby = swarm.nearbyDrones(myLat, myLon, DRONE_ALERT_M);
        float droneSteer = 0.0f;

        for (auto& nd : nearby)
        {
            float proximity = 1.0f - (nd.dist / DRONE_ALERT_M);

            // Relative bearing of neighbour relative to my heading
            float relBear = nd.bearing - myHeading;
            while (relBear >  180.0f) relBear -= 360.0f;
            while (relBear < -180.0f) relBear += 360.0f;

            if (nd.dist < DRONE_DANGER_M)
            {
                outSpeed = 0.0f;
                droneSteer += (relBear >= 0 ? -DRONE_AVOID_DEG
                                            :  DRONE_AVOID_DEG) * 2.0f;
            }
            else
            {
                // Starboard rule: turn right if other drone is on left bow
                droneSteer += (relBear >= 0 ? -DRONE_AVOID_DEG
                                            :  DRONE_AVOID_DEG)
                              * proximity;
            }
        }

        outHeading += std::clamp(droneSteer, -DRONE_AVOID_DEG * 2,
                                              DRONE_AVOID_DEG * 2);

        // ── Normalise heading ─────────────────────────────────────
        while (outHeading <   0.0f) outHeading += 360.0f;
        while (outHeading > 360.0f) outHeading -= 360.0f;

        outSpeed = std::max(0.0f, std::min(outSpeed, CRUISE_SPEED_MS));
    }

    bool hasTarget() const { return hasTarget_; }
    double targetLat() const { return targetLat_; }
    double targetLon() const { return targetLon_; }

private:
    double targetLat_ = 0.0;
    double targetLon_ = 0.0;
    bool   hasTarget_ = false;
};
