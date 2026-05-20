#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "config.h"
#include "DroneAgent.h"
#include "MissionLoader.h"
#include "SwarmController.h"

// ================================================================
// main.cpp  —  Entry point, do not edit for normal use
//
// What happens here:
//   1. Load mission JSON
//   2. Open one UDP socket per drone
//   3. Call SwarmController::init()
//   4. Run control loop at CONTROL_LOOP_HZ
//      → drain UDP packets for every drone
//      → call SwarmController::tick()
//      → print status every 5 seconds
// ================================================================

int main(int argc, char* argv[])
{
    std::cout << "=== SAR Drone Swarm Controller ===\n"
              << "Drones : " << DRONE_COUNT << "\n\n";

    // ── Load mission ──────────────────────────────────────────────
    std::string missionFile = (argc > 1) ? argv[1] : MISSION_FILE;
    MissionLoader mission;
    if (!mission.load(missionFile))
    {
        std::cerr << "[WARN] No mission file loaded — drones will hover.\n"
                  << "       Usage: ./swarm_controller " << MISSION_FILE << "\n\n";
    }

    // ── Init agents ───────────────────────────────────────────────
    std::vector<DroneAgent> agents;
    agents.reserve(DRONE_COUNT);
    for (int i = 0; i < DRONE_COUNT; i++)
    {
        agents.emplace_back();
        agents.back().id = i;

        int rxPort = RECV_BASE_PORT + i;
        int txPort = SEND_BASE_PORT + i;
        bool ok = agents.back().open(UNITY_HOST, txPort, rxPort);

        std::cout << "[Drone " << std::setw(2) << i << "] "
                  << (ok ? "ready  " : "FAILED ")
                  << "RX:" << rxPort << "  TX:" << txPort << "\n";
    }

    // ── Init swarm controller ─────────────────────────────────────
    SwarmController swarm;
    swarm.init(agents, mission);

    // ── Control loop ─────────────────────────────────────────────
    const auto tickInterval = std::chrono::milliseconds(1000 / CONTROL_LOOP_HZ);
    auto       nextTick     = std::chrono::steady_clock::now();
    auto       lastStatus   = std::chrono::steady_clock::now();
    const auto statusEvery  = std::chrono::seconds(5);
    float      dt           = 1.f / CONTROL_LOOP_HZ;

    std::cout << "\nRunning at " << CONTROL_LOOP_HZ << " Hz — Ctrl+C to stop\n\n";

    while (true)
    {
        swarm.tick(agents, dt);

        // ── Periodic status print ─────────────────────────────────
        auto now = std::chrono::steady_clock::now();
        if (now - lastStatus >= statusEvery)
        {
            lastStatus = now;
            std::cout << "──────────────────────────────────────────\n";
            for (auto& a : agents) a.printStatus();
            std::cout << "\n";
        }

        nextTick += tickInterval;
        std::this_thread::sleep_until(nextTick);
    }
    return 0;
}