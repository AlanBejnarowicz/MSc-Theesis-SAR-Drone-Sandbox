#pragma once
// ================================================================
// GridLogger.h  —  Periodic CSV logger for swarm coverage validation
//
// Writes three CSV files (all timestamped with real wall-clock time):
//
//  1. grid_coverage.csv  — one row per snapshot (every LOG_INTERVAL_S):
//     t_s, coverage_pct, cells_visited, cells_total,
//     min_staleness_s, max_staleness_s, mean_staleness_s, drones_active
//
//  2. grid_cells.csv  — one row per cell per snapshot:
//     t_s, cell_id, centre_lat, centre_lon, staleness_s  (-1 = never)
//
//  3. drone_positions.csv  — one row per drone per snapshot:
//     t_s, drone_id, lat, lon, heading_deg, speed_kn, kf_std_m, is_lost
//
// All timestamps are real elapsed seconds from logger open() call.
// Uses std::chrono wall clock — immune to control loop rate variation.
//
// Usage in SwarmController::tick():
//   logger.snapshot(agents);   // no t_s argument needed
// ================================================================

#include "DroneAgent.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iomanip>

class GridLogger
{
public:
    float LOG_INTERVAL_S   = 10.f;   // coverage + cell snapshot interval
    float POS_INTERVAL_S   = 1.f;    // drone position snapshot interval (finer)

    // ── Open all three CSV files ───────────────────────────────────
    bool open(const std::string& coveragePath  = "grid_coverage.csv",
              const std::string& cellsPath     = "grid_cells.csv",
              const std::string& positionsPath = "drone_positions.csv")
    {
        _cov.open(coveragePath);
        _cells.open(cellsPath);
        _pos.open(positionsPath);

        if (!_cov.is_open() || !_cells.is_open() || !_pos.is_open()) {
            std::cerr << "[GridLogger] Cannot open one or more CSV files\n";
            return false;
        }

        _cov   << "t_s,coverage_pct,cells_visited,cells_total,"
               << "min_staleness_s,max_staleness_s,mean_staleness_s,"
               << "drones_active\n";

        _cells << "t_s,cell_id,centre_lat,centre_lon,staleness_s\n";

        _pos   << "t_s,drone_id,lat,lon,heading_deg,speed_kn,"
               << "kf_std_m,is_lost\n";

        // Record mission start time
        _startMs = _nowMs();
        _lastCovSnapshotMs  = _startMs;
        _lastPosSnapshotMs  = _startMs;

        std::cout << "[GridLogger] Logging to:\n"
                  << "  " << coveragePath  << "\n"
                  << "  " << cellsPath     << "\n"
                  << "  " << positionsPath << "\n";
        return true;
    }

    // ── Call every tick — uses wall clock internally ───────────────
    void snapshot(const std::vector<DroneAgent>& agents)
    {
        if (!_cov.is_open()) return;

        long long nowMs  = _nowMs();
        float     t_s    = (float)(nowMs - _startMs) / 1000.f;

        // ── Drone positions (every POS_INTERVAL_S) ────────────────
        if ((float)(nowMs - _lastPosSnapshotMs) / 1000.f >= POS_INTERVAL_S)
        {
            _lastPosSnapshotMs = nowMs;
            for (auto& a : agents)
            {
                if (!a.hasState) continue;
                float stdM = (a.kalman.stdLat + a.kalman.stdLon) * 0.5f;
                _pos << std::fixed << std::setprecision(2)
                     << t_s                           << ","
                     << a.id                          << ","
                     << std::setprecision(7)
                     << (a.kalman.ready ? a.kalman.lat : a.state.gps.lat) << ","
                     << (a.kalman.ready ? a.kalman.lon : a.state.gps.lon) << ","
                     << std::setprecision(1)
                     << a.state.compass.heading       << ","
                     << a.state.velocity.speed_knots  << ","
                     << std::setprecision(2)
                     << stdM                          << ","
                     << (int)a.isLost                 << "\n";
            }
            _pos.flush();
            _posRows += (int)agents.size();
        }

        // ── Coverage + cells (every LOG_INTERVAL_S) ───────────────
        if ((float)(nowMs - _lastCovSnapshotMs) / 1000.f < LOG_INTERVAL_S)
            return;
        _lastCovSnapshotMs = nowMs;

        if (agents.empty()) return;
        const SurveillanceGrid& grid = agents[0].grid;
        const auto& cells = grid.cells();
        if (cells.empty()) return;

        int   visited  = 0;
        float minS = 1e9f, maxS = 0.f, sumS = 0.f;
        int   stalCount = 0;

        for (auto& c : cells)
        {
            float s = grid.staleness(c.id);
            if (c.lastVisitMs > 0) {
                visited++;
                minS = std::min(minS, s);
                maxS = std::max(maxS, s);
                sumS += s;
                stalCount++;
            }
        }

        int   total   = (int)cells.size();
        float covPct  = 100.f * visited / total;
        float meanS   = stalCount > 0 ? sumS / stalCount : 0.f;
        if (minS > 1e8f) minS = 0.f;

        int dronesActive = 0;
        for (auto& a : agents)
            if (a.hasState && !a.isLost) dronesActive++;

        _cov << std::fixed << std::setprecision(1) << t_s << ","
             << std::setprecision(2) << covPct  << ","
             << visited << "," << total          << ","
             << std::setprecision(1)
             << minS << "," << maxS << "," << meanS << ","
             << dronesActive << "\n";
        _cov.flush();

        // Per-cell snapshot
        for (auto& c : cells)
        {
            float staleS = (c.lastVisitMs > 0)
                           ? (float)((nowMs - c.lastVisitMs) / 1000LL)
                           : -1.f;
            _cells << std::fixed << std::setprecision(1) << t_s << ","
                   << c.id << ","
                   << std::setprecision(7)
                   << c.centreLat << "," << c.centreLon << ","
                   << std::setprecision(1) << staleS << "\n";
        }
        _cells.flush();
        _covSnapshots++;
    }

    void close()
    {
        if (_cov.is_open())   { _cov.close();   }
        if (_cells.is_open()) { _cells.close();  }
        if (_pos.is_open())   { _pos.close();    }
        std::cout << "[GridLogger] Closed."
                  << "  Coverage snapshots: " << _covSnapshots
                  << "  Position rows: "      << _posRows << "\n";
    }

    ~GridLogger() { close(); }

private:
    std::ofstream _cov, _cells, _pos;
    long long     _startMs            = 0;
    long long     _lastCovSnapshotMs  = 0;
    long long     _lastPosSnapshotMs  = 0;
    int           _covSnapshots       = 0;
    int           _posRows            = 0;

    static long long _nowMs() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }
};