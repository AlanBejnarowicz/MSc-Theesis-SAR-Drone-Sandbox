#pragma once
// ================================================================
// ZoneLoader.h  —  Parse anchorage_zone.json / patrol zone files
//
// Supports the format used in anchorage_zone.json:
//   { "search_zones": [{ "boundary": [{"lat":..,"lon":..}, ...] }] }
// ================================================================

#include "GridPatrol.h"
#include "json.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

using json = nlohmann::json;

struct SearchZone
{
    long long           id   = 0;
    std::string         name;
    std::string         color;
    std::vector<GeoPoint> boundary;
};

class ZoneLoader
{
public:
    std::vector<SearchZone> zones;

    bool loadFile(const std::string& path)
    {
        std::ifstream f(path);
        if (!f.is_open()) {
            std::cerr << "[ZoneLoader] Cannot open: " << path << "\n";
            return false;
        }

        json j;
        try { f >> j; }
        catch (std::exception& e) {
            std::cerr << "[ZoneLoader] JSON parse error: " << e.what() << "\n";
            return false;
        }

        zones.clear();
        if (!j.contains("search_zones")) return false;

        for (auto& z : j["search_zones"])
        {
            SearchZone sz;
            sz.id    = z.value("id",    0LL);
            sz.name  = z.value("name",  "");
            sz.color = z.value("color", "");

            if (z.contains("boundary"))
                for (auto& pt : z["boundary"])
                    sz.boundary.push_back({ pt.value("lat", 0.0),
                                            pt.value("lon", 0.0) });

            if (!sz.boundary.empty())
                zones.push_back(std::move(sz));
        }

        std::cout << "[ZoneLoader] Loaded " << zones.size()
                  << " zone(s) from " << path << "\n";
        for (auto& z : zones)
            std::cout << "  Zone: " << z.name
                      << "  (" << z.boundary.size() << " vertices)\n";

        return !zones.empty();
    }

    // Convenience: init GridPatrol with the first (or named) zone
    bool initPatrol(GridPatrol& patrol, int numDrones,
                    const std::string& zoneName = "") const
    {
        if (zones.empty()) return false;

        const SearchZone* target = &zones[0];
        if (!zoneName.empty())
            for (auto& z : zones)
                if (z.name == zoneName) { target = &z; break; }

        patrol.init(target->boundary, numDrones);
        return true;
    }
};
