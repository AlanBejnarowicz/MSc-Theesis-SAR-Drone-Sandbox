#pragma once
// ================================================================
// MissionLoader.h  —  Load waypoints and search zones from JSON
//
// Supports two JSON formats:
//
//  Format A — simple waypoint list:
//  {
//    "waypoints": [
//      {"lat": 54.44, "lon": 18.91, "label": "WP1"},
//      {"lat": 54.43, "lon": 18.90}
//    ]
//  }
//
//  Format B — your anchorage_zone.json / SAR zone format:
//  {
//    "waypoints": [...],
//    "search_zones": [
//      { "name": "SEARCH_ALPHA",
//        "boundary": [{"lat":..,"lon":..}, ...] }
//    ]
//  }
//
//  Both formats can coexist in one file.
//
// Usage:
//   MissionLoader ml;
//   ml.load("mission.json");
//   nav.setWaypoints(ml.waypoints);          // explicit waypoints
//   nav.setWaypoints(ml.zoneToWaypoints(0)); // boundary of zone 0
// ================================================================

#include "WaypointNav.h"
#include "json.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

using json = nlohmann::json;

struct SearchZone
{
    std::string           name;
    std::string           color;
    std::vector<Waypoint> boundary;  // polygon vertices
};

class MissionLoader
{
public:
    std::vector<Waypoint>   waypoints;   // explicit ordered waypoints
    std::vector<SearchZone> zones;       // named search polygons

    // ── Load from file ────────────────────────────────────────────
    bool load(const std::string& path)
    {
        std::ifstream f(path);
        if (!f.is_open()) {
            std::cerr << "[Mission] Cannot open: " << path << "\n";
            return false;
        }

        json j;
        try { f >> j; }
        catch (std::exception& e) {
            std::cerr << "[Mission] JSON parse error: " << e.what() << "\n";
            return false;
        }

        waypoints.clear();
        zones.clear();

        // ── Explicit waypoints ────────────────────────────────────
        if (j.contains("waypoints"))
            for (auto& wp : j["waypoints"])
                if (wp.contains("lat") && wp.contains("lon"))
                    waypoints.push_back({wp["lat"].get<double>(),
                                        wp["lon"].get<double>()});

        // ── Search zones ──────────────────────────────────────────
        if (j.contains("search_zones"))
        {
            for (auto& z : j["search_zones"])
            {
                SearchZone sz;
                sz.name  = z.value("name",  "");
                sz.color = z.value("color", "");

                if (z.contains("boundary"))
                    for (auto& pt : z["boundary"])
                        sz.boundary.push_back({pt.value("lat", 0.0),
                                               pt.value("lon", 0.0)});

                if (!sz.boundary.empty())
                    zones.push_back(std::move(sz));
            }
        }

        std::cout << "[Mission] Loaded from " << path << ":\n"
                  << "  waypoints : " << waypoints.size() << "\n"
                  << "  zones     : " << zones.size() << "\n";
        for (auto& z : zones)
            std::cout << "    [" << z.name << "] "
                      << z.boundary.size() << " vertices\n";

        return !waypoints.empty() || !zones.empty();
    }

    // ── Convert zone boundary to an ordered waypoint list ─────────
    // Simply walks the polygon vertices in order — useful for
    // perimeter patrol.  Pass zone index (0-based).
    std::vector<Waypoint> zoneToWaypoints(int zoneIdx, bool closeLoop = true) const
    {
        if (zoneIdx < 0 || zoneIdx >= (int)zones.size()) return {};
        auto wps = zones[zoneIdx].boundary;
        if (closeLoop && !wps.empty())
            wps.push_back(wps.front());   // return to start
        return wps;
    }

    // ── Save a waypoint list back to file (simple format) ─────────
    bool save(const std::string& path) const
    {
        json j;
        json arr = json::array();
        for (auto& wp : waypoints)
            arr.push_back({{"lat", wp.lat}, {"lon", wp.lon}});
        j["waypoints"] = arr;

        std::ofstream f(path);
        if (!f.is_open()) return false;
        f << j.dump(2);
        return true;
    }
};
