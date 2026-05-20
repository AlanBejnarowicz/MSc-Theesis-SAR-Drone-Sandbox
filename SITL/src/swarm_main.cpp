#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "config.h"
#include "packets.h"
#include "transport.h"
#include "json_parser.h"

// ── Swarm stack ──────────────────────────────────────────────────
#include "ZoneLoader.h"
#include "GridPatrol.h"
#include "SwarmDroneController.h"

// ================================================================
// MAIN.CPP  —  SAR Swarm Fleet Controller
//
// Architecture:
//   - One GridPatrol (shared) divides the zone into cells
//   - One SwarmDroneController per drone (Kalman + PID + Trajectory)
//   - UDP to/from Unity SITL unchanged (port scheme from config.h)
//   - LoRa beacons encoded in DroneCommandPacket.loraBroadcast
//
// Usage:
//   ./swarm_controller [zone_file.json]
//   Default zone file: anchorage_zone.json
// ================================================================

struct DroneAgent
{
    SwarmDroneController controller;
    UDPSocket            socket;
};

// ── Receive all pending packets for one drone ────────────────────
void receiveDrone(DroneAgent& agent)
{
    std::string raw;
    while (agent.socket.receive(raw))
    {
        DroneStatePacket state;
        if (parseStatePacket(raw, state))
        {
            agent.controller.onStateReceived(state);

            // Immediate response after each state packet
            DroneCommandPacket cmd = agent.controller.tick();
            agent.socket.send(serializeCommand(cmd));
        }
    }
}

// ── Status print every N seconds ─────────────────────────────────
static void printFleetStatus(const std::vector<DroneAgent>& agents,
                              const GridPatrol& patrol)
{
    std::cout << "\n─────────────────────────────────────────────────────\n";
    std::cout << "  Coverage: " << (int)patrol.coveragePercent() << "%"
              << "  Cells: " << patrol.totalCells()
              << "  MinVisit: " << patrol.minVisitCount() << "\n";
    for (auto& a : agents)
        a.controller.printStatus();
    std::cout << "─────────────────────────────────────────────────────\n\n";
}

// ================================================================
int main(int argc, char* argv[])
{
    std::cout << "=== SAR Swarm Fleet Controller ===\n"
              << "Drones: " << DRONE_COUNT << "\n\n";

    // ── Load patrol zone ─────────────────────────────────────────
    std::string zoneFile = (argc > 1) ? argv[1] : "anchorage_zone.json";

    ZoneLoader  zoneLoader;
    GridPatrol  patrol;

    if (!zoneLoader.loadFile(zoneFile))
    {
        std::cerr << "[ERROR] Failed to load zone file: " << zoneFile << "\n";
        return 1;
    }

    if (!zoneLoader.initPatrol(patrol, DRONE_COUNT))
    {
        std::cerr << "[ERROR] Failed to init patrol grid\n";
        return 1;
    }

    // ── Init drone agents ─────────────────────────────────────────
    std::vector<DroneAgent> agents;
    agents.reserve(DRONE_COUNT);   // prevent realloc/move that resets Kalman state
    agents.resize(DRONE_COUNT);

    for (int i = 0; i < DRONE_COUNT; i++)
    {
        agents[i].controller.init(i);
        agents[i].controller.patrol = &patrol;   // shared grid

        int rxPort = RECV_BASE_PORT + i;
        int txPort = SEND_BASE_PORT + i;

        bool ok = agents[i].socket.open(UNITY_HOST, txPort, rxPort);

        std::cout << "[Drone " << i << "] "
                  << (ok ? "ready" : "SOCKET FAILED")
                  << "  RX:" << rxPort
                  << "  TX:" << txPort << "\n";
    }

    // ── Control loop ─────────────────────────────────────────────
    const auto interval      = std::chrono::milliseconds(1000 / CONTROL_LOOP_HZ);
    auto       nextTick      = std::chrono::steady_clock::now();
    auto       lastStatusPrint = std::chrono::steady_clock::now();
    const auto statusInterval  = std::chrono::seconds(5);

    std::cout << "\nRunning at " << CONTROL_LOOP_HZ
              << " Hz — Ctrl+C to stop\n\n";

    while (true)
    {
        // 1. Drain incoming UDP (state packets), respond immediately
        for (auto& agent : agents)
            receiveDrone(agent);

        // 2. Lost detection + idle command for lost drones
        for (auto& agent : agents)
        {
            agent.controller.updateLostStatus();
            if (agent.controller.isLost)
            {
                DroneCommandPacket idle;
                idle.droneId  = agent.controller.droneId;
                idle.throttle = 0.0f;
                idle.steer    = 0.0f;
                agent.socket.send(serializeCommand(idle));
            }
        }

        // 3. Periodic status print
        auto now = std::chrono::steady_clock::now();
        if (now - lastStatusPrint >= statusInterval)
        {
            printFleetStatus(agents, patrol);
            lastStatusPrint = now;
        }

        nextTick += interval;
        std::this_thread::sleep_until(nextTick);
    }

    return 0;
}