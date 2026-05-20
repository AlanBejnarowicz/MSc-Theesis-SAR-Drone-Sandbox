#pragma once
// ================================================================
// SwarmController.h  —  YOUR swarm logic goes here
//
// Default behaviour (already wired):
//   - Grid divided into 300x300m cells from mission zone
//   - Each drone independently picks least-recently-visited cell
//   - LoRa shares: current target cell + recent visit timestamps
//   - Drone-drone molecular repulsion (CollisionAvoidance)
//   - AIS vessel avoidance with Mahalanobis ellipse zones
//
// To customise: edit init() and tick() below.
// ================================================================

#include "DroneAgent.h"
#include "MissionLoader.h"
#include "CollisionAvoidance.h"
#include <vector>
#include <iostream>
#include <numeric>

class SwarmController
{
public:
    void init(std::vector<DroneAgent>& agents, const MissionLoader& mission)
    {
        std::cout << "[Swarm] init — " << agents.size() << " drones\n";

        // Use first zone boundary as surveillance area
        std::vector<Waypoint> boundary;
        if (!mission.zones.empty())
            boundary = mission.zones[0].boundary;
        else if (mission.waypoints.size() >= 3)
            boundary = mission.waypoints;   // treat waypoints as polygon
        else {
            std::cerr << "[Swarm] No zone boundary found in mission file!\n";
            return;
        }

        // Init grid for each drone (same boundary, each tracks independently
        // then reconciles via LoRa)
        for (auto& agent : agents)
        {
            agent.initGrid(boundary);
            agent.headingPID.Kp          = 0.008f;
            agent.headingPID.Kd          = 0.003f;
            agent.headingPID.cruiseSpeed = 5.0f;
            agent.headingPID.minSpeedFactor = 0.25f;
            agent.speedP.Kp              = 0.4f;
        }

        std::cout << "[Swarm] Grid cells: " << agents[0].grid.cellCount()
                  << "  per drone\n";
    }

    void tick(std::vector<DroneAgent>& agents, float dt)
    {
        (void)dt;

        // Reset avoidance accumulators before each tick
        for (auto& agent : agents) {
            agent.avoidNudgeDeg    = 0.f;
            agent.avoidSpeedFactor = 1.f;
        }

        // Base processing: Kalman + grid nav + LoRa + AIS
        for (auto& agent : agents)
            agent.receiveAndTick();

        // Drone-drone molecular repulsion (adds to avoidNudgeDeg)
        for (auto& agent : agents)
            CollisionAvoidance::apply(agent, agents);

        // ── Add your extra swarm logic here ──────────────────────
    }

    // ── Helper: distance between two agents ──────────────────────
    static float distBetween(const DroneAgent& a, const DroneAgent& b)
    {
        return SurveillanceGrid::distM(a.kalman.lat, a.kalman.lon,
                                       b.kalman.lat, b.kalman.lon);
    }
};