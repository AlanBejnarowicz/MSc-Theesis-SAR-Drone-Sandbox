#pragma once
// ================================================================
// EKFLogger.h  —  Standalone EKF validation logger (drone 0 only)
//
// Decoupled from DroneAgent — no include dependency, no ordering issues.
// Call log() from SwarmController::tick() by passing fields explicitly.
//
// Usage in SwarmController.h:
//
//   #include "EKFLogger.h"           // add this include
//
//   EKFLogger ekfLogger;             // add as class member
//
//   void init(...) {
//       ekfLogger.open("ekf_validation.csv");
//   }
//
//   void tick(std::vector<DroneAgent>& agents, float dt) {
//       ...existing code...
//
//       // Log drone 0 — after receiveAndTick()
//       auto& a = agents[0];
//       ekfLogger.log(
//           a.lastPacket.timestamp,
//           a.lastPacket.pos_x,   a.lastPacket.pos_z,
//           a.lastPacket.gps.lat, a.lastPacket.gps.lon, a.lastPacket.gps.accuracy,
//           a.kalman.lat,         a.kalman.lon,
//           a.kalman.stdLat,      a.kalman.stdLon,
//           a.kalman.vx,          a.kalman.vz,
//           a.lastPacket.velocity.speed_ms,
//           a.lastPacket.compass.heading
//       );
//   }
//
// World origin: lat=54.52551, lon=18.7703
//   pos_x = East  (metres)
//   pos_z = North (metres)
// ================================================================

#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

class EKFLogger
{
public:
    // ── World origin ──────────────────────────────────────────────
    static constexpr double ORIGIN_LAT = 54.52551;
    static constexpr double ORIGIN_LON = 18.7703;


    static constexpr float UNITY_ORIGIN_X = -9999.0f - 7.0f;
    static constexpr float UNITY_ORIGIN_Z =  9636.0f;

    // Flat-earth scale at ~54°N  (same values as KalmanNav.h)
    static constexpr double MPL     = 111111.0;   // match vGPS exactly
    static constexpr double MPL_LON =  64528.0;   // 111111 * cos(54.3718°)

    // ── Open ──────────────────────────────────────────────────────
    bool open(const std::string& path = "ekf_validation.csv")
    {
        _file.open(path, std::ios::out | std::ios::trunc);
        if (!_file.is_open())
        {
            std::cerr << "[EKFLogger] Cannot open " << path << "\n";
            return false;
        }

        _file << "tick"
              << ",timestamp_s"
              << ",true_lat,true_lon"
              << ",gps_lat,gps_lon,gps_accuracy_m"
              << ",ekf_lat,ekf_lon"
              << ",ekf_std_lat_m,ekf_std_lon_m"
              << ",ekf_vx_ms,ekf_vz_ms"
              << ",speed_ms,heading_deg"
              << ",true_vs_ekf_err_m"
              << ",true_vs_gps_err_m"
              << "\n";

        std::cout << "[EKFLogger] Logging drone 0 → " << path << "\n";
        return true;
    }

    void close()
    {
        if (_file.is_open())
        {
            _file.flush();
            _file.close();
            std::cout << "[EKFLogger] Closed — " << _tick << " rows written.\n";
        }
    }

    ~EKFLogger() { close(); }

    // ── Log one tick ──────────────────────────────────────────────
    // All args come straight from agents[0] fields — see usage above.
    void log(float  timestamp,
             float  pos_x,     float  pos_z,       // Unity world metres
             double gps_lat,   double gps_lon,  float gps_acc,
             double ekf_lat,   double ekf_lon,
             float  ekf_std_lat, float ekf_std_lon,
             float  ekf_vx,    float  ekf_vz,
             float  speed_ms,  float  heading_deg)
    {
        if (!_file.is_open()) return;

        // Convert Unity world pos → lat/lon (truth)
        double true_lat = ORIGIN_LAT + (double)(pos_z - UNITY_ORIGIN_Z) / MPL;
        double true_lon = ORIGIN_LON + (double)(pos_x - UNITY_ORIGIN_X) / MPL_LON;

        // Elapsed time from first packet
        if (_tick == 0) _t0 = timestamp;
        float t_s = timestamp - _t0;

        float err_ekf = distM(true_lat, true_lon, ekf_lat, ekf_lon);
        float err_gps = distM(true_lat, true_lon, gps_lat, gps_lon);

        _file << std::fixed
              << _tick
              << "," << std::setprecision(3) << t_s
              << "," << std::setprecision(8)
              << true_lat << "," << true_lon
              << "," << gps_lat << "," << gps_lon
              << "," << std::setprecision(2) << gps_acc
              << "," << std::setprecision(8)
              << ekf_lat << "," << ekf_lon
              << "," << std::setprecision(3)
              << ekf_std_lat << "," << ekf_std_lon
              << "," << ekf_vx << "," << ekf_vz
              << "," << std::setprecision(2)
              << speed_ms << "," << heading_deg
              << "," << err_ekf
              << "," << err_gps
              << "\n";

        if ((_tick % 100) == 0) _file.flush();
        ++_tick;
    }

private:
    std::ofstream _file;
    long  _tick = 0;
    float _t0   = 0.f;

    static float distM(double lat1, double lon1, double lat2, double lon2)
    {
        float dy = (float)((lat2 - lat1) * MPL);
        float dx = (float)((lon2 - lon1) * MPL_LON);
        return std::sqrt(dx * dx + dy * dy);
    }
};