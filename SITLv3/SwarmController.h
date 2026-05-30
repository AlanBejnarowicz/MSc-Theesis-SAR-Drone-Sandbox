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

#include "EKFLogger.h"
#include "GridLogger.h"



class SwarmController
{
public:

    EKFLogger ekfLogger;

    GridLogger logger;          // call logger.open() in main or here
    float      elapsedS = 0.f; // mission elapsed seconds


    void init(std::vector<DroneAgent>& agents, const MissionLoader& mission)
    {
        std::cout << "[Swarm] init — " << agents.size() << " drones\n";

        //ekfLogger.open("ekf_validation.csv");


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
            agent.headingPID.cruiseSpeed = 10.0f;
            agent.headingPID.minSpeedFactor = 0.35f;
            agent.speedP.Kp              = 0.4f;
        }

        std::cout << "[Swarm] Grid cells: " << agents[0].grid.cellCount()
                  << "  per drone\n";
    }

    void tick(std::vector<DroneAgent>& agents, float dt)
    {
       (void)dt;
 
        // ── Step 1: reset avoidance fields ───────────────────────
        for (auto& agent : agents) {
            agent.avoidNudgeDeg    = 0.f;
            agent.avoidSpeedFactor = 1.f;
        }
 
        // ── Step 2: compute drone-drone avoidance BEFORE commands ─
        // Reads kalman positions from previous tick (accurate enough).
        // Must run before receiveAndTick() so fields are set when
        // DroneAgent::tick() builds and sends the command this tick.
        for (auto& agent : agents)
            CollisionAvoidance::apply(agent, agents);
 
        // ── Step 3: receive packets, Kalman, grid nav, send ───────
        for (auto& agent : agents)
            agent.receiveAndTick();
 
        // ── Log coverage snapshot (wall-clock timed internally) ───
        logger.snapshot(agents);


        // if (agents[0].receiveAndTick() > 0)  // receiveAndTick() returns packet count
        // {

        //     auto& a = agents[0];
        //     ekfLogger.log(
        //         a.state.timestamp,
        //         a.state.pos_x,    a.state.pos_z,
        //         a.state.gps.lat,  a.state.gps.lon,  a.state.gps.accuracy,
        //         a.kalman.lat,          a.kalman.lon,
        //         a.kalman.stdLat,       a.kalman.stdLon,
        //         a.kalman.vx,           a.kalman.vz,
        //         a.state.velocity.speed_ms,
        //         a.state.compass.heading
        //     );

        // }





        // ── Add your extra swarm logic here ──────────────────────
    }

    // ── Helper: distance between two agents ──────────────────────
    static float distBetween(const DroneAgent& a, const DroneAgent& b)
    {
        return SurveillanceGrid::distM(a.kalman.lat, a.kalman.lon,
                                       b.kalman.lat, b.kalman.lon);
    }
};