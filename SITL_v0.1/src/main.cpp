#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

#include "config.h"
#include "packets.h"
#include "transport.h"
#include "json_parser.h"
#include "DroneController.h"



// ================================================================
// MAIN.CPP
// ================================================================

struct DroneAgent
{
    DroneController controller;
    UDPSocket       socket;
};

// ── Receive all pending packets, feed latest to controller ───────
void receive(DroneAgent& agent)
{
    std::string raw;
    int count = 0;
    while (agent.socket.receive(raw))
    {
        DroneStatePacket state;
        if (parseStatePacket(raw, state))
        {
            agent.controller.onStateReceived(state);
            DroneCommandPacket cmd  = agent.controller.tick();
            agent.socket.send(serializeCommand(cmd));
            count++;
        }
    }

    
    // if (count > 0)
    //     std::cout << "[Drone " << agent.controller.droneId 
    //               << "] received " << count << " packets\n";
    // else
    //     std::cout << "[Drone " << agent.controller.droneId 
    //               << "] no packets\n";
}

// ── Run controller, send command to Unity ────────────────────────
void send(DroneAgent& agent)
{
    agent.controller.updateLostStatus();

    DroneCommandPacket cmd  = agent.controller.tick();
    std::string        json = serializeCommand(cmd);
    agent.socket.send(json);

    // Status print
    std::cout << "[Drone " << cmd.droneId << "]"
              << "  hdg="  << agent.controller.state.compass.heading
              << "  spd="  << agent.controller.state.velocity.speed_knots << "kn"
              << "  thr="  << cmd.throttle
              << "  str="  << cmd.steer
              << (agent.controller.isLost ? "  [LOST]" : "")
              << "\n";
}

// ================================================================
int main()
{
    std::cout << "=== SAR Drone Fleet Controller 2.0 ===\n"
              << "Drones: " << DRONE_COUNT << "\n\n";

    // ── Init agents ──────────────────────────────────────────────
    std::vector<DroneAgent> agents(DRONE_COUNT);

    for (int i = 0; i < DRONE_COUNT; i++)
    {
        agents[i].controller.init(i);

        int rxPort = RECV_BASE_PORT + i;
        int txPort = SEND_BASE_PORT + i;

        bool ok = agents[i].socket.open(UNITY_HOST, txPort, rxPort);


        std::cout << "[Drone " << i << "] "
                  << (ok ? "ready" : "SOCKET FAILED")
                  << "  RX:" << rxPort
                  << "  TX:" << txPort << "\n";
    }

    // ── Control loop ─────────────────────────────────────────────
    const auto interval = std::chrono::milliseconds(1000 / CONTROL_LOOP_HZ);
    auto       nextTick = std::chrono::steady_clock::now();

    std::cout << "\nRunning at " << CONTROL_LOOP_HZ << " Hz — Ctrl+C to stop\n\n";

    while (true)
    {
        for (auto& agent : agents) receive(agent);

        // Timeout check only — no send here
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

        nextTick += interval;
        std::this_thread::sleep_until(nextTick);
    }

    return 0;

    // ── ESP-IDF: replace while(true) + sleep_until with ──────────
    // while (true) {
    //     for (auto& agent : agents) receive(agent);
    //     for (auto& agent : agents) send(agent);
    //     vTaskDelay(pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));
    // }
}
