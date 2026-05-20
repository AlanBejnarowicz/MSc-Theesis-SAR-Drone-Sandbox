#pragma once
// ================================================================
// WaypointNav.h  —  Navigate to a list of lat/lon waypoints
//
// Usage:
//   WaypointNav nav;
//   nav.setWaypoints({{54.44, 18.90}, {54.43, 18.88}});
//   // each tick:
//   float targetHdg = nav.update(myLat, myLon);
//   bool  done      = nav.finished();
// ================================================================

#include <vector>
#include <cmath>

static constexpr double WN_MPL     = 111320.0;
static constexpr double WN_MPL_LON = 63990.0;

struct Waypoint { double lat, lon; };

class WaypointNav
{
public:
    float arrivalRadiusM = 50.f;   // metres — waypoint considered reached

    void setWaypoints(const std::vector<Waypoint>& wps)
    {
        _wps   = wps;
        _index = 0;
    }

    // Returns desired heading (0-360°) toward current waypoint.
    // Call every tick; advances to next waypoint on arrival.
    float update(double myLat, double myLon)
    {
        if (finished()) return 0.f;

        const Waypoint& wp = _wps[_index];

        float dist = distM(myLat, myLon, wp.lat, wp.lon);
        if (dist < arrivalRadiusM)
        {
            std::cout << "[Nav] Waypoint " << _index << " reached  ("
                      << wp.lat << "," << wp.lon << ")\n";
            _index++;
            if (finished()) return 0.f;
        }

        return bearingDeg(myLat, myLon, _wps[_index].lat, _wps[_index].lon);
    }

    bool finished() const { return _index >= (int)_wps.size(); }
    int  index()    const { return _index; }
    int  total()    const { return (int)_wps.size(); }

    float distToCurrentM(double myLat, double myLon) const
    {
        if (finished()) return 0.f;
        return distM(myLat, myLon, _wps[_index].lat, _wps[_index].lon);
    }

    static float distM(double lat1, double lon1, double lat2, double lon2)
    {
        float dy = (float)((lat2 - lat1) * WN_MPL);
        float dx = (float)((lon2 - lon1) * WN_MPL_LON);
        return std::sqrt(dx*dx + dy*dy);
    }

    static float bearingDeg(double lat1, double lon1, double lat2, double lon2)
    {
        float dy = (float)((lat2 - lat1) * WN_MPL);
        float dx = (float)((lon2 - lon1) * WN_MPL_LON);
        float b  = std::atan2(dx, dy) * 180.f / 3.14159265f;
        if (b < 0.f) b += 360.f;
        return b;
    }

private:
    std::vector<Waypoint> _wps;
    int _index = 0;
};
