#pragma once
// ================================================================
// CollisionAvoidance.h  —  Molecular repulsion between drones
//
// Physics model:
//   F = K / d²   (inverse-square, like electrostatic repulsion)
//   Force cuts off at CUTOFF_M (200m) — zero beyond that.
//   All per-drone forces are summed into a world-frame vector,
//   then converted to a heading correction (degrees) and a
//   speed reduction factor.
//
// Usage — inside SwarmController::tick(), after receiveAndTick():
//
//   CollisionAvoidance::apply(agent, agents);
//
// This modifies agent's headingPID target and speedP target
// by injecting a corrective nudge — it does NOT override them,
// it adds on top of the navigation heading.
// ================================================================

#include "DroneAgent.h"
#include "WaypointNav.h"
#include <cmath>
#include <vector>

class CollisionAvoidance
{
public:
    // ── Tuning ────────────────────────────────────────────────────
    static constexpr float CUTOFF_M     = 200.f;  // force drops to 0 beyond this
    static constexpr float K            = 80000.f; // repulsion strength
                                                   // K / CUTOFF_M² = 2 deg nudge
                                                   // K / 10m²      = 800 deg (hard push)
    static constexpr float MAX_NUDGE_DEG= 90.f;   // cap on heading correction
    static constexpr float SPEED_SCALE  = 0.3f;   // how much speed reduces when pushed

    // ── Apply repulsion from all other drones to one agent ────────
    // Call once per agent per tick, after receiveAndTick().
    // Modifies agent._avoidNudgeDeg and agent._avoidSpeedFactor
    // which DroneAgent::tick() already applies if you wire it in
    // (see note at bottom of this file).
    static void apply(DroneAgent& agent,
                      const std::vector<DroneAgent>& agents)
    {
        if (!agent.kalman.ready) return;

        float forceX = 0.f;   // east  (world frame, metres/arbitrary)
        float forceZ = 0.f;   // north

        for (const auto& other : agents)
        {
            if (other.id == agent.id)    continue;
            if (!other.kalman.ready)     continue;

            float dx = (float)((agent.kalman.lon - other.kalman.lon)
                               * 63990.0);   // metres east
            float dz = (float)((agent.kalman.lat - other.kalman.lat)
                               * 111320.0);  // metres north
            float d2 = dx*dx + dz*dz;
            float d  = std::sqrt(d2);

            if (d > CUTOFF_M || d < 0.1f) continue;

            // Smooth cutoff: multiply by (1 - d/CUTOFF)²
            // so force tapers to zero at the boundary (no sharp edge)
            float taper = 1.f - (d / CUTOFF_M);
            float mag   = (K / d2) * taper * taper;

            // Unit vector pointing away from other drone
            forceX += (dx / d) * mag;
            forceZ += (dz / d) * mag;
        }

        // ── Convert force vector to heading nudge ─────────────────
        float forceMag = std::sqrt(forceX*forceX + forceZ*forceZ);

        if (forceMag < 0.01f)
        {
            agent.avoidNudgeDeg    = 0.f;
            agent.avoidSpeedFactor = 1.f;
            return;
        }

        // Direction the repulsion wants us to go (world-frame bearing)
        float repulsionBearing = std::atan2(forceX, forceZ)
                                 * 180.f / 3.14159265f;
        if (repulsionBearing < 0.f) repulsionBearing += 360.f;

        // Nudge = signed angle from current heading to repulsion bearing
        float currentHdg = agent.state.compass.heading;
        float nudge = _deltaAngle(currentHdg, repulsionBearing);

        // Scale nudge by force magnitude but cap it
        // At 10m: forceMag ≈ 800 → nudge capped to MAX_NUDGE_DEG
        // At 100m: forceMag ≈ 8  → nudge ≈ 8° (gentle)
        // At 190m: forceMag ≈ 0.2 → nudge ≈ 0.2° (barely felt)
        float scaledNudge = std::clamp(nudge * (forceMag / 100.f),
                                       -MAX_NUDGE_DEG, MAX_NUDGE_DEG);

        agent.avoidNudgeDeg = scaledNudge;

        // Speed reduction — slow down proportional to how hard we're pushed
        // Full speed when no force; SPEED_SCALE fraction at MAX_NUDGE_DEG
        float pushRatio = std::min(std::abs(scaledNudge) / MAX_NUDGE_DEG, 1.f);
        agent.avoidSpeedFactor = 1.f - SPEED_SCALE * pushRatio;
    }

private:
    static float _deltaAngle(float from, float to)
    {
        float d = to - from;
        while (d >  180.f) d -= 360.f;
        while (d < -180.f) d += 360.f;
        return d;
    }
};

// ================================================================
// WIRING NOTE
// To activate, add two fields to DroneAgent.h (public section):
//
//   float avoidNudgeDeg    = 0.f;
//   float avoidSpeedFactor = 1.f;
//
// Then in DroneAgent::tick(), replace the heading/speed lines with:
//
//   float tgtHdg   = nav.update(kalman.lat, kalman.lon);
//   float nudgedHdg = tgtHdg + avoidNudgeDeg;   // <-- add nudge
//   if (nudgedHdg <   0.f) nudgedHdg += 360.f;
//   if (nudgedHdg > 360.f) nudgedHdg -= 360.f;
//
//   float desiredSpd;
//   cmd.steer    = headingPID.update(pkt.compass.heading,
//                                    nudgedHdg, dt, desiredSpd);
//   desiredSpd  *= avoidSpeedFactor;              // <-- scale speed
//   cmd.throttle = speedP.update(pkt.velocity.speed_ms, desiredSpd);
//
// And in SwarmController::tick(), call after receiveAndTick():
//
//   CollisionAvoidance::apply(agent, agents);
// ================================================================
