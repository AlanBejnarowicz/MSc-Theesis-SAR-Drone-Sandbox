#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

#include "KalmanNav.h"
#include "HeadingPID.h"
#include "SpeedP.h"
#include "WaypointNav.h"

#include "packets.h"
#include "transport.h"
#include "json_parser.h"
#include "config.h"

// ================================================================
// kalman_main.cpp  —  Kalman + PID + waypoint navigation
//
// Stack (bottom to top):
//   KalmanNav    — fuses GPS + IMU + speed/heading -> best position
//   WaypointNav  — bearing to next waypoint from KF position
//   HeadingPID   — steer toward bearing, scale speed by alignment
//   SpeedP       — throttle to reach desired speed
// ================================================================

// ── Waypoints ─────────────────────────────────────────────────────
// Small square near spawn (54.4383, 18.9254) — edit as needed
static const std::vector<Waypoint> WAYPOINTS = {
    {54.440, 18.910},
    {54.430, 18.910},
    {54.430, 18.940},
    {54.440, 18.940},
    {54.440, 18.910},
};

static long long nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

// Standalone helper (avoids calling private method)
static float deltaAngle(float from, float to)
{
    float d = to - from;
    while (d >  180.f) d -= 360.f;
    while (d < -180.f) d += 360.f;
    return d;
}

int main()
{
    std::cout << "=== Kalman + PID Navigation (Drone 0) ===\n\n";

    UDPSocket sock;
    if (!sock.open(UNITY_HOST, SEND_BASE_PORT, RECV_BASE_PORT))
    {
        std::cerr << "[ERROR] Socket failed\n";
        return 1;
    }
    std::cout << "Socket OK  RX:" << RECV_BASE_PORT
              << "  TX:" << SEND_BASE_PORT << "\n\n";

    KalmanNav   kf;
    HeadingPID  hdgPID;
    SpeedP      spdP;
    WaypointNav nav;
    nav.setWaypoints(WAYPOINTS);

    long long lastKalmanMs = 0;
    long long lastPrintMs  = nowMs();
    int       pktCount     = 0;
    int       cmdSeq       = 0;

    float lastHdg    = 0.f, lastSpdKn = 0.f, lastSpdMs = 0.f;
    float lastThr    = 0.f, lastStr   = 0.f, lastTgtHdg = 0.f;

    // Print header
    std::cout << std::left
              << std::setw(11) << "KF_lat"
              << std::setw(11) << "KF_lon"
              << std::setw(7)  << "std_m"
              << std::setw(6)  << "hdg"
              << std::setw(6)  << "tHdg"
              << std::setw(7)  << "err"
              << std::setw(8)  << "spd_kn"
              << std::setw(6)  << "thr"
              << std::setw(7)  << "str"
              << std::setw(4)  << "wp"
              << std::setw(8)  << "dist_m"
              << "pkts/s\n"
              << std::string(88, '-') << "\n";

    while (true)
    {
        std::string raw;
        while (sock.receive(raw))
        {
            DroneStatePacket pkt;
            if (!parseStatePacket(raw, pkt)) continue;

            long long now = nowMs();

            // ── dt ────────────────────────────────────────────────
            float dt = (lastKalmanMs > 0)
                       ? std::clamp((float)(now - lastKalmanMs) / 1000.f,
                                    0.001f, 0.2f)
                       : 0.02f;
            lastKalmanMs = now;

            // ── Kalman predict ────────────────────────────────────
            if (kf.ready)
                kf.predict(dt,
                           pkt.imu.accel_x,
                           pkt.imu.accel_z,
                           pkt.compass.heading);

            // ── Kalman init ───────────────────────────────────────
            if (!kf.ready && std::abs(pkt.gps.lat) > 1.0)
            {
                kf.init(pkt.gps.lat, pkt.gps.lon);
                std::cout << "[KF] Init at ("
                          << std::fixed << std::setprecision(6)
                          << pkt.gps.lat << ", " << pkt.gps.lon << ")\n\n";
            }

            // ── Kalman updates ────────────────────────────────────
            if (kf.ready)
            {
                if (std::abs(pkt.gps.lat) > 1.0)
                    kf.updateGPS(pkt.gps.lat, pkt.gps.lon, pkt.gps.accuracy);

                if (pkt.velocity.speed_ms > 0.01f)
                    kf.updateSpeedHeading(pkt.velocity.speed_ms,
                                         pkt.compass.heading);
            }

            // ── Control ───────────────────────────────────────────
            float thr = 0.f, str = 0.f;
            if (kf.ready && !nav.finished())
            {
                float tgtHdg  = nav.update(kf.lat, kf.lon);
                lastTgtHdg    = tgtHdg;
                float desiredSpd;
                str = hdgPID.update(pkt.compass.heading, tgtHdg, dt, desiredSpd);
                thr = spdP.update(pkt.velocity.speed_ms, desiredSpd);
            }

            lastThr    = thr;
            lastStr    = str;
            lastHdg    = pkt.compass.heading;
            lastSpdKn  = pkt.velocity.speed_knots;
            lastSpdMs  = pkt.velocity.speed_ms;

            // ── Send command ──────────────────────────────────────
            DroneCommandPacket cmd;
            cmd.droneId   = pkt.droneId;
            cmd.packetSeq = cmdSeq++;
            cmd.throttle  = thr;
            cmd.steer     = str;
            sock.send(serializeCommand(cmd));
            pktCount++;
        }

        // ── Print every second ────────────────────────────────────
        long long now = nowMs();
        if (now - lastPrintMs >= 1000)
        {
            lastPrintMs = now;
            if (!kf.ready) {
                std::cout << "  Waiting for GPS... (" << pktCount << " pkts)\n";
                pktCount = 0;
                continue;
            }

            float err   = deltaAngle(lastHdg, lastTgtHdg);
            float distM = nav.finished() ? 0.f
                        : nav.distToCurrentM(kf.lat, kf.lon);

            std::cout << std::fixed << std::setprecision(6) << std::left
                      << std::setw(11) << kf.lat
                      << std::setw(11) << kf.lon
                      << std::setprecision(1)
                      << std::setw(7)  << ((kf.stdLat + kf.stdLon) * 0.5f)
                      << std::setw(6)  << (int)lastHdg
                      << std::setw(6)  << (int)lastTgtHdg
                      << std::setprecision(1)
                      << std::setw(7)  << err
                      << std::setw(8)  << lastSpdKn
                      << std::setprecision(3)
                      << std::setw(6)  << lastThr
                      << std::setw(7)  << lastStr
                      << std::setw(4)  << nav.index()
                      << std::setw(8)  << (int)distM
                      << pktCount << "/s"
                      << (nav.finished() ? "  [DONE]" : "")
                      << "\n";
            pktCount = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return 0;
}
