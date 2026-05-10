#pragma once
// ================================================================
// GridPatrol.h  —  Divide a geo-polygon into a square grid and
//                  assign cells to drones for equal coverage patrol
//
// Algorithm:
//   1. Compute bounding box of the search zone polygon
//   2. Rasterise into CELL_SIZE_M × CELL_SIZE_M square cells
//   3. Keep only cells whose centre is inside the polygon (ray-cast)
//   4. Assign cells to drones round-robin by their coverage count
//      (drone with fewest visits gets next assignment)
//   5. Within a drone's assigned cells, route via nearest-unvisited
// ================================================================

#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>

static constexpr double GRID_METRES_PER_LAT = 111320.0;
static constexpr double GRID_METRES_PER_LON = 63990.0;

struct GeoPoint { double lat, lon; };

struct GridCell
{
    int    id        = 0;
    double centreLat = 0.0;
    double centreLon = 0.0;
    int    visitCount= 0;      // how many times any drone visited
    int    assignedTo= -1;     // which drone currently targeting this
};

// ── Point-in-polygon (ray casting) ───────────────────────────────
inline bool pointInPolygon(double lat, double lon,
                            const std::vector<GeoPoint>& poly)
{
    int n = (int)poly.size();
    bool inside = false;
    for (int i = 0, j = n-1; i < n; j = i++)
    {
        double xi = poly[i].lon, yi = poly[i].lat;
        double xj = poly[j].lon, yj = poly[j].lat;
        bool intersect = ((yi > lat) != (yj > lat)) &&
                         (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

class GridPatrol
{
public:
    static constexpr float CELL_SIZE_M   = 300.0f;  // metres per cell side
    static constexpr float ARRIVAL_DIST  = 80.0f;   // metres — cell considered "visited"
    static constexpr float REASSIGN_DIST = 500.0f;  // hand off cell if drone diverges

    // ── Build grid from polygon ───────────────────────────────────
    void init(const std::vector<GeoPoint>& polygon, int numDrones)
    {
        polygon_  = polygon;
        numDrones_= numDrones;
        cells_.clear();
        droneTargets_.assign(numDrones, -1);
        droneProgress_.assign(numDrones, 0);

        if (polygon.empty()) return;

        // Bounding box
        double minLat= 1e9, maxLat=-1e9, minLon= 1e9, maxLon=-1e9;
        for (auto& p : polygon) {
            minLat=std::min(minLat,p.lat); maxLat=std::max(maxLat,p.lat);
            minLon=std::min(minLon,p.lon); maxLon=std::max(maxLon,p.lon);
        }

        // Step sizes in degrees
        double dLat = CELL_SIZE_M / GRID_METRES_PER_LAT;
        double dLon = CELL_SIZE_M / GRID_METRES_PER_LON;

        int id = 0;
        for (double lat = minLat + dLat*0.5; lat < maxLat; lat += dLat)
            for (double lon = minLon + dLon*0.5; lon < maxLon; lon += dLon)
                if (pointInPolygon(lat, lon, polygon))
                    cells_.push_back({id++, lat, lon, 0, -1});

        std::cout << "[GridPatrol] Zone has " << cells_.size()
                  << " cells of " << (int)CELL_SIZE_M << "m for "
                  << numDrones << " drones\n";
    }

    // ── Called each tick: returns target cell centre for drone i ──
    // dronePos: current drone position
    // Returns false if no cells available (shouldn't happen)
    bool getTarget(int droneId, double droneLat, double droneLon,
                   double& targetLat, double& targetLon)
    {
        if (cells_.empty()) return false;

        int currentTarget = droneTargets_[droneId];

        // Check if we've arrived at current target
        if (currentTarget >= 0 && currentTarget < (int)cells_.size())
        {
            float dist = distMetres(droneLat, droneLon,
                                    cells_[currentTarget].centreLat,
                                    cells_[currentTarget].centreLon);
            if (dist < ARRIVAL_DIST)
            {
                cells_[currentTarget].visitCount++;
                cells_[currentTarget].assignedTo = -1;
                droneTargets_[droneId] = -1;
                droneProgress_[droneId]++;
                currentTarget = -1;
            }
        }

        // Assign new target if needed
        if (currentTarget < 0)
        {
            currentTarget = pickBestCell(droneId, droneLat, droneLon);
            droneTargets_[droneId] = currentTarget;
            if (currentTarget >= 0)
                cells_[currentTarget].assignedTo = droneId;
        }

        if (currentTarget < 0) return false;

        targetLat = cells_[currentTarget].centreLat;
        targetLon = cells_[currentTarget].centreLon;
        return true;
    }

    // ── Release a cell if drone is reassigned / lost ──────────────
    void releaseTarget(int droneId)
    {
        int t = droneTargets_[droneId];
        if (t >= 0 && t < (int)cells_.size())
            cells_[t].assignedTo = -1;
        droneTargets_[droneId] = -1;
    }

    // ── Stats ─────────────────────────────────────────────────────
    int totalCells()  const { return (int)cells_.size(); }

    float coveragePercent() const
    {
        if (cells_.empty()) return 0.0f;
        int visited = 0;
        for (auto& c : cells_) if (c.visitCount > 0) visited++;
        return 100.0f * visited / cells_.size();
    }

    int minVisitCount() const
    {
        if (cells_.empty()) return 0;
        int mn = cells_[0].visitCount;
        for (auto& c : cells_) mn = std::min(mn, c.visitCount);
        return mn;
    }

    int droneProgress(int id) const { return droneProgress_[id]; }

    const std::vector<GridCell>& cells() const { return cells_; }

    // ── Distance helper ───────────────────────────────────────────
    static float distMetres(double lat1, double lon1,
                             double lat2, double lon2)
    {
        float dy = (float)((lat2 - lat1) * GRID_METRES_PER_LAT);
        float dx = (float)((lon2 - lon1) * GRID_METRES_PER_LON);
        return std::sqrt(dx*dx + dy*dy);
    }

    // Bearing from point1 to point2, degrees 0-360
    static float bearingDeg(double lat1, double lon1,
                             double lat2, double lon2)
    {
        float dy = (float)((lat2 - lat1) * GRID_METRES_PER_LAT);
        float dx = (float)((lon2 - lon1) * GRID_METRES_PER_LON);
        float b  = std::atan2(dx, dy) * 180.0f / 3.14159265f;
        if (b < 0.0f) b += 360.0f;
        return b;
    }

private:
    std::vector<GeoPoint>  polygon_;
    std::vector<GridCell>  cells_;
    int                    numDrones_ = 1;
    std::vector<int>       droneTargets_;     // cell index per drone
    std::vector<int>       droneProgress_;    // visit count per drone

    // ── Select the cell that balances coverage + distance ─────────
    // Score = visitCount * VISIT_WEIGHT  +  dist * DIST_WEIGHT
    // Unassigned cells preferred, nearest among least-visited
    int pickBestCell(int droneId, double droneLat, double droneLon)
    {
        int   best      = -1;
        float bestScore = std::numeric_limits<float>::max();

        // Find minimum visit count among free cells
        int minVisit = std::numeric_limits<int>::max();
        for (auto& c : cells_)
            if (c.assignedTo < 0)
                minVisit = std::min(minVisit, c.visitCount);

        if (minVisit == std::numeric_limits<int>::max()) {
            // All cells assigned — pick the one assigned to self already
            // or just the closest free cell ignoring assignments
            for (auto& c : cells_) {
                float d = distMetres(droneLat, droneLon,
                                     c.centreLat, c.centreLon);
                if (d < bestScore) { bestScore = d; best = c.id; }
            }
            return best;
        }

        // Score: prefer least visited, then nearest
        const float VISIT_W = 5000.0f;  // 1 extra visit = 5 km penalty
        for (auto& c : cells_)
        {
            if (c.assignedTo >= 0 && c.assignedTo != droneId) continue;
            float d     = distMetres(droneLat, droneLon,
                                     c.centreLat, c.centreLon);
            float score = (float)(c.visitCount - minVisit) * VISIT_W + d;
            if (score < bestScore) { bestScore = score; best = c.id; }
        }
        return best;
    }
};
