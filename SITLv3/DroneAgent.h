#pragma once
// ================================================================
// DroneAgent.h  —  One self-contained drone agent
//
// Owns:
//   KalmanNav          — position estimation
//   HeadingPID         — steer toward target heading
//   SpeedP             — throttle toward desired speed
//   SurveillanceGrid   — 300x300m patrol grid, visit tracking
//   SurveillanceLoRa   — LoRa beacon encode/decode
//   UDPSocket          — comms with Unity SITL
//
// Avoidance fields written by CollisionAvoidance::apply():
//   avoidNudgeDeg      — heading nudge (drone-drone repulsion)
//   avoidSpeedFactor   — speed scale   (drone-drone repulsion)
//
// AIS avoidance is applied directly to the command in tick()
// via AISAvoidance::applyToCmd() — overrides steer/throttle.
// ================================================================

#include "KalmanNav.h"
#include "HeadingPID.h"
#include "SpeedP.h"
#include "SurveillanceGrid.h"
#include "SurveillanceLoRa.h"
#include "AISAvoidance.h"
#include "packets.h"
#include "transport.h"
#include "json_parser.h"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>

static long long agentNowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

class DroneAgent
{
public:
    // ── Identity ──────────────────────────────────────────────────
    int id = -1;

    // ── Sub-systems ───────────────────────────────────────────────
    KalmanNav         kalman;
    HeadingPID        headingPID;
    SpeedP            speedP;
    SurveillanceGrid  grid;
    SurveillanceLoRa  loraComm;
    UDPSocket         socket;

    // ── Last known state ──────────────────────────────────────────
    DroneStatePacket state;
    bool hasState  = false;
    bool isLost    = false;

    // ── Drone-drone avoidance (written by CollisionAvoidance) ─────
    float avoidNudgeDeg    = 0.f;
    float avoidSpeedFactor = 1.f;

    // ── Init grid from zone boundary ──────────────────────────────
    void initGrid(const std::vector<Waypoint>& boundary)
    {
        grid.init(boundary, id);
    }

    // ── Open UDP socket ───────────────────────────────────────────
    bool open(const std::string& host, int txPort, int rxPort)
    {
        return socket.open(host, txPort, rxPort);
    }

    // ── Main tick ─────────────────────────────────────────────────
    DroneCommandPacket tick(const DroneStatePacket& pkt)
    {
        state    = pkt;
        hasState = true;
        isLost   = false;

        long long now = agentNowMs();
        float dt = (_lastMs > 0)
                   ? std::clamp((float)(now - _lastMs) / 1000.f, 0.001f, 0.2f)
                   : 0.02f;
        _lastMs = now;

        // ── Kalman filter ─────────────────────────────────────────
        if (kalman.ready)
            kalman.predict(dt, pkt.imu.accel_x, pkt.imu.accel_z,
                           pkt.compass.heading);

        if (!kalman.ready && std::abs(pkt.gps.lat) > 1.0)
            kalman.init(pkt.gps.lat, pkt.gps.lon);

        if (kalman.ready) {
            if (std::abs(pkt.gps.lat) > 1.0)
                kalman.updateGPS(pkt.gps.lat, pkt.gps.lon, pkt.gps.accuracy);
            if (pkt.velocity.speed_ms > 0.01f)
                kalman.updateSpeedHeading(pkt.velocity.speed_ms,
                                         pkt.compass.heading);
        }

        // ── LoRa: decode neighbours, merge into grid ──────────────
        SurveillanceLoRa::processLoRa(pkt.lora, grid);

        // ── Build command ─────────────────────────────────────────
        DroneCommandPacket cmd;
        cmd.droneId   = id;
        cmd.packetSeq = _seq++;

        if (!kalman.ready) return cmd;

        // ── Grid: get target cell ─────────────────────────────────
        double tLat, tLon;
        bool   hasTarget = grid.tick(kalman.lat, kalman.lon, tLat, tLon);

        if (hasTarget)
        {
            float tgtHdg = SurveillanceGrid::bearingDeg(
                               kalman.lat, kalman.lon, tLat, tLon);

            // Apply drone-drone avoidance nudge on top of grid heading
            float nudgedHdg = tgtHdg + avoidNudgeDeg;
            if (nudgedHdg <   0.f) nudgedHdg += 360.f;
            if (nudgedHdg > 360.f) nudgedHdg -= 360.f;

            float desiredSpd;
            cmd.steer    = headingPID.update(pkt.compass.heading,
                                             nudgedHdg, dt, desiredSpd);
            desiredSpd  *= avoidSpeedFactor;
            cmd.throttle = speedP.update(pkt.velocity.speed_ms, desiredSpd);
        }

        // ── AIS avoidance: direct override (highest priority) ─────
        AISAvoidance::applyToCmd(kalman, state, cmd);

        // ── LoRa broadcast ────────────────────────────────────────
        if (loraComm.shouldBroadcast())
            cmd.loraBroadcast = SurveillanceLoRa::encode(
                                    grid, id, kalman.lat, kalman.lon);

        return cmd;
    }

    // ── Drain socket, tick, send reply ───────────────────────────
    int receiveAndTick()
    {
        std::string raw;
        int count = 0;
        while (socket.receive(raw))
        {
            DroneStatePacket pkt;
            if (!parseStatePacket(raw, pkt)) continue;
            DroneCommandPacket cmd = tick(pkt);
            socket.send(serializeCommand(cmd));
            count++;
        }
        if (hasState && agentNowMs() - _lastMs > 3000)
            isLost = true;
        return count;
    }

    // ── Status line ───────────────────────────────────────────────
    void printStatus() const
    {
        if (!hasState) { std::cout << "[" << id << "] no data\n"; return; }

        float cov    = grid.coverageFraction() * 100.f;
        float maxAge = grid.maxStaleness();
        int   tgt    = grid.currentTargetId();

        std::cout << "[" << std::setw(2) << id << "]"
                  << std::fixed << std::setprecision(5)
                  << "  pos=(" << kalman.lat << "," << kalman.lon << ")"
                  << std::setprecision(1)
                  << "  std=" << ((kalman.stdLat+kalman.stdLon)*0.5f) << "m"
                  << "  hdg=" << (int)state.compass.heading << "°"
                  << "  spd=" << state.velocity.speed_knots << "kn"
                  << "  cell=" << tgt
                  << "  cov=" << (int)cov << "%"
                  << "  maxAge=" << (int)(maxAge/60) << "min"
                  << (isLost ? "  [LOST]" : "")
                  << "\n";
    }

private:
    long long _lastMs = 0;
    int       _seq    = 0;
};