#pragma once
// ================================================================
// SurveillanceGrid.h  —  Divide a geo-polygon into 300x300m cells,
//                        track last-visit time per cell across the
//                        whole swarm, assign targets to drones.
//
// Each drone owns ONE SurveillanceGrid instance.
// The grid map is shared via LoRa (see SurveillanceLoRa.h).
//
// Cell selection strategy:
//   Score = timeSinceVisit(cell) - CONTENTION_PENALTY * claimedByOthers
//   Drone picks the highest-score uncontested cell near itself.
//   "Claimed" = another drone broadcast it as its target this epoch.
// ================================================================

#include "MissionLoader.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

static constexpr double SG_MPL     = 111320.0;
static constexpr double SG_MPL_LON = 63990.0;

// ── Utilities ─────────────────────────────────────────────────────
static long long sg_nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

static bool sg_pointInPolygon(double lat, double lon,
                               const std::vector<Waypoint>& poly)
{
    int n = (int)poly.size();
    bool inside = false;
    for (int i = 0, j = n-1; i < n; j = i++)
    {
        double xi = poly[i].lon, yi = poly[i].lat;
        double xj = poly[j].lon, yj = poly[j].lat;
        if (((yi > lat) != (yj > lat)) &&
            (lon < (xj-xi)*(lat-yi)/(yj-yi)+xi))
            inside = !inside;
    }
    return inside;
}

// ── Grid cell ─────────────────────────────────────────────────────
struct GridCell
{
    int    id          = -1;
    double centreLat   = 0.0;
    double centreLon   = 0.0;
    long long lastVisitMs = 0;     // 0 = never visited
    int    claimedBy   = -1;       // drone id currently targeting this cell
    int    targetedBy  = -1;       // drone id that will visit next (from LoRa)
};

class SurveillanceGrid
{
public:
    static constexpr float CELL_SIZE_M      = 300.f;
    static constexpr float ARRIVAL_DIST_M   = 80.f;   // cell considered visited
    static constexpr float CONTENTION_PENALTY= 120.f; // seconds penalty if claimed

    int ownDroneId = -1;

    // ── Build grid from zone boundary polygon ─────────────────────
    void init(const std::vector<Waypoint>& boundary, int droneId)
    {
        ownDroneId = droneId;
        cells_.clear();
        currentTargetId_ = -1;

        if (boundary.empty()) return;

        double minLat=1e9, maxLat=-1e9, minLon=1e9, maxLon=-1e9;
        for (auto& p : boundary) {
            minLat = std::min(minLat, p.lat); maxLat = std::max(maxLat, p.lat);
            minLon = std::min(minLon, p.lon); maxLon = std::max(maxLon, p.lon);
        }

        double dLat = CELL_SIZE_M / SG_MPL;
        double dLon = CELL_SIZE_M / SG_MPL_LON;

        int id = 0;
        for (double lat = minLat + dLat*0.5; lat < maxLat; lat += dLat)
            for (double lon = minLon + dLon*0.5; lon < maxLon; lon += dLon)
                if (sg_pointInPolygon(lat, lon, boundary))
                    cells_.push_back({id++, lat, lon, 0, -1, -1});

        std::cout << "[Grid] Drone " << droneId << ": "
                  << cells_.size() << " cells ("
                  << (int)CELL_SIZE_M << "m)\n";
    }

    // ── Call every tick: check arrival, pick next target ──────────
    // Returns current target cell centre. Call after updateFromLoRa().
    bool tick(double myLat, double myLon,
              double& targetLat, double& targetLon)
    {
        if (cells_.empty()) return false;

        // Check arrival at current target
        if (currentTargetId_ >= 0)
        {
            GridCell& cur = cells_[currentTargetId_];
            float dist = distM(myLat, myLon, cur.centreLat, cur.centreLon);
            if (dist < ARRIVAL_DIST_M)
            {
                cur.lastVisitMs = sg_nowMs();
                cur.claimedBy   = -1;
                std::cout << "[Grid] Drone " << ownDroneId
                          << " visited cell " << currentTargetId_ << "\n";
                currentTargetId_ = -1;
            }
        }

        // Pick new target if needed
        if (currentTargetId_ < 0)
        {
            currentTargetId_ = pickBestCell(myLat, myLon);
            if (currentTargetId_ >= 0)
                cells_[currentTargetId_].claimedBy = ownDroneId;
        }

        if (currentTargetId_ < 0) return false;

        targetLat = cells_[currentTargetId_].centreLat;
        targetLon = cells_[currentTargetId_].centreLon;
        return true;
    }

    // ── Merge visit data received from a neighbour ────────────────
    // If neighbour reports a more recent visit, accept it.
    void mergeVisit(int cellId, long long visitMs)
    {
        if (cellId < 0 || cellId >= (int)cells_.size()) return;
        if (visitMs > cells_[cellId].lastVisitMs)
            cells_[cellId].lastVisitMs = visitMs;
    }

    // ── Record a neighbour's claimed target ───────────────────────
    void setNeighbourTarget(int droneId, int cellId)
    {
        // Clear previous claim from this drone
        for (auto& c : cells_)
            if (c.targetedBy == droneId) c.targetedBy = -1;

        if (cellId >= 0 && cellId < (int)cells_.size())
            cells_[cellId].targetedBy = droneId;
    }

    // ── Accessors ─────────────────────────────────────────────────
    int  currentTargetId()  const { return currentTargetId_; }
    int  cellCount()        const { return (int)cells_.size(); }
    const std::vector<GridCell>& cells() const { return cells_; }

    // Seconds since a cell was last visited (large if never visited)
    float staleness(int cellId) const
    {
        if (cellId < 0 || cellId >= (int)cells_.size()) return 0.f;
        long long lv = cells_[cellId].lastVisitMs;
        if (lv == 0) return 1e6f;   // never visited → highest priority
        return (float)(sg_nowMs() - lv) / 1000.f;
    }

    // Coverage: fraction of cells visited at least once
    float coverageFraction() const
    {
        if (cells_.empty()) return 0.f;
        int visited = 0;
        for (auto& c : cells_) if (c.lastVisitMs > 0) visited++;
        return (float)visited / cells_.size();
    }

    // Most stale cell's age in seconds
    float maxStaleness() const
    {
        float mx = 0.f;
        for (int i = 0; i < (int)cells_.size(); i++)
            mx = std::max(mx, staleness(i));
        return mx;
    }

    static float distM(double lat1, double lon1, double lat2, double lon2)
    {
        float dy = (float)((lat2-lat1)*SG_MPL);
        float dx = (float)((lon2-lon1)*SG_MPL_LON);
        return std::sqrt(dx*dx+dy*dy);
    }

    static float bearingDeg(double lat1, double lon1, double lat2, double lon2)
    {
        float dy = (float)((lat2-lat1)*SG_MPL);
        float dx = (float)((lon2-lon1)*SG_MPL_LON);
        float b  = std::atan2(dx, dy) * 180.f / 3.14159265f;
        return b < 0.f ? b + 360.f : b;
    }

private:
    std::vector<GridCell> cells_;
    int currentTargetId_ = -1;

    // ── Cell selection ────────────────────────────────────────────
    // Score = staleness + startup_offset
    //       - CONTENTION_PENALTY  if targeted or claimed by another drone
    //       - distance_penalty    prefer closer cells
    //
    // Startup spreading: when all cells are never-visited, every drone
    // would score identically and pile onto the same cell.
    // Replace the identical 1e6 sentinel with a deterministic per-drone
    // hash so each drone gets a unique priority ordering from tick 1.
    int pickBestCell(double myLat, double myLon)
    {
        int   best      = -1;
        float bestScore = -std::numeric_limits<float>::max();

        bool allFresh = true;
        for (auto& c : cells_)
            if (c.lastVisitMs > 0) { allFresh = false; break; }

        for (auto& c : cells_)
        {
            if (c.claimedBy == ownDroneId) continue;

            float stale;
            if (allFresh) {
                // Per-drone deterministic hash: unique ordering per drone
                unsigned int h = (unsigned int)(c.id * 2654435761u)
                               ^ (unsigned int)(ownDroneId * 40503u);
                stale = (float)(h & 0xFFFF);
            } else {
                stale = staleness(c.id);
            }

            // Penalise cells already taken by another drone
            float penalty = 0.f;
            if (c.claimedBy  >= 0 && c.claimedBy  != ownDroneId)
                penalty += CONTENTION_PENALTY;
            if (c.targetedBy >= 0 && c.targetedBy != ownDroneId)
                penalty += CONTENTION_PENALTY;

            float dist        = distM(myLat, myLon, c.centreLat, c.centreLon);
            float distPenalty = dist / 5000.f * 60.f;

            float score = stale - penalty - distPenalty;

            if (score > bestScore) { bestScore = score; best = c.id; }
        }
        return best;
    }
};