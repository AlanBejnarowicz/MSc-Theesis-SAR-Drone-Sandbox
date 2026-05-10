#pragma once
// ================================================================
// LoraSwarm.h  —  Compact LoRa inter-drone communication
//
// Design goals:
//   - Minimal payload (≤32 bytes per broadcast)
//   - Each drone broadcasts its position + target cell every N ticks
//   - Neighbours use this to avoid duplicating patrol cells
//   - Range (ToF) from LoRa used to update Kalman filter
//
// Payload format (32 bytes, packed):
//   [0]    droneId    uint8   (0-255)
//   [1]    flags      uint8   bit0=hasGPS, bit1=isLost
//   [2-5]  lat        int32   microdegrees  (×1e6)
//   [6-9]  lon        int32   microdegrees  (×1e6)
//   [10-11]heading    uint16  centidegrees  (0-36000)
//   [12-13]speed      uint16  mm/s          (0-65535)
//   [14-15]cellId     uint16  current target cell
//   [16-17]cellVisits uint16  total cells visited
//   [18]   posStdM    uint8   position stddev metres (0-255)
//   [19-31]           reserved / future
//
// Base64 encode → loraBroadcast field in DroneCommandPacket
// ================================================================

#include "packets.h"
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// ── Base64 (minimal, no deps) ─────────────────────────────────────
static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline std::string b64encode(const uint8_t* data, size_t len)
{
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3)
    {
        uint32_t val = (uint32_t)data[i] << 16;
        if (i+1 < len) val |= (uint32_t)data[i+1] << 8;
        if (i+2 < len) val |= (uint32_t)data[i+2];
        out += B64[(val >> 18) & 63];
        out += B64[(val >> 12) & 63];
        out += (i+1 < len) ? B64[(val >>  6) & 63] : '=';
        out += (i+2 < len) ? B64[ val        & 63] : '=';
    }
    return out;
}

inline std::vector<uint8_t> b64decode(const std::string& s)
{
    auto decChar = [](char c) -> int {
        if (c>='A'&&c<='Z') return c-'A';
        if (c>='a'&&c<='z') return c-'a'+26;
        if (c>='0'&&c<='9') return c-'0'+52;
        if (c=='+') return 62;
        if (c=='/') return 63;
        return -1;
    };
    std::vector<uint8_t> out;
    for (size_t i = 0; i+3 < s.size(); i += 4)
    {
        int a=decChar(s[i]), b=decChar(s[i+1]),
            c=decChar(s[i+2]), d=decChar(s[i+3]);
        if (a<0||b<0) break;
        out.push_back((uint8_t)((a<<2)|(b>>4)));
        if (c>=0) out.push_back((uint8_t)((b<<4)|(c>>2)));
        if (d>=0) out.push_back((uint8_t)((c<<6)|d));
    }
    return out;
}

// ── Packed beacon ─────────────────────────────────────────────────
#pragma pack(push,1)
struct LoraBeacon
{
    uint8_t  droneId;
    uint8_t  flags;         // bit0=hasGPS, bit1=isLost
    int32_t  lat_ud;        // microdegrees
    int32_t  lon_ud;        // microdegrees
    uint16_t heading_cd;    // centidegrees
    uint16_t speed_mms;     // mm/s
    uint16_t cellId;        // current patrol target
    uint16_t cellsVisited;  // total
    uint8_t  posStdM;       // Kalman position stddev clamped to 255
    uint8_t  reserved[13];  // pad to 32 bytes
};
#pragma pack(pop)
static_assert(sizeof(LoraBeacon) == 32, "LoraBeacon must be 32 bytes");

// ── Parsed info about a neighbour drone from LoRa ─────────────────
struct SwarmNeighbour
{
    int     droneId    = -1;
    double  lat        = 0.0;
    double  lon        = 0.0;
    float   heading    = 0.0f;
    float   speed_ms   = 0.0f;
    int     cellId     = -1;
    int     cellsVisited = 0;
    float   posStdM    = 99.0f;
    bool    isLost     = false;
    float   rssi       = 0.0f;      // from LoRa block
    float   rangeMetre = 0.0f;      // from LoRa ToF distance
    long long lastSeenMs = 0;
};

class LoraSwarm
{
public:
    static constexpr int   BROADCAST_EVERY_N  = 10;   // ticks between own beacons
    static constexpr float NEIGHBOUR_TIMEOUT  = 5.0f; // seconds

    // ── Encode own state into base64 beacon string ────────────────
    std::string encodeBeacon(int droneId, double lat, double lon,
                              float heading, float speed_ms,
                              int targetCell, int cellsVisited,
                              float posStdM, bool isLost, bool hasGPS)
    {
        LoraBeacon b{};
        b.droneId       = (uint8_t)std::clamp(droneId, 0, 255);
        b.flags         = (hasGPS ? 0x01 : 0) | (isLost ? 0x02 : 0);
        b.lat_ud        = (int32_t)(lat * 1e6);
        b.lon_ud        = (int32_t)(lon * 1e6);
        b.heading_cd    = (uint16_t)std::clamp((int)(heading * 100), 0, 36000);
        b.speed_mms     = (uint16_t)std::clamp((int)(speed_ms * 1000), 0, 65535);
        b.cellId        = (uint16_t)std::clamp(targetCell, 0, 65535);
        b.cellsVisited  = (uint16_t)std::clamp(cellsVisited, 0, 65535);
        b.posStdM       = (uint8_t)std::clamp((int)posStdM, 0, 255);

        return b64encode(reinterpret_cast<uint8_t*>(&b), sizeof(b));
    }

    // ── Decode incoming LoRa neighbour data ───────────────────────
    // Call with the LoRa_Block from incoming DroneStatePacket
    void processLoraBlock(const LoRa_Block& loraBlock, long long nowMs)
    {
        for (auto& n : loraBlock.neighbours)
        {
            if (n.lastPayload.empty()) continue;
            auto raw = b64decode(n.lastPayload);
            if (raw.size() < sizeof(LoraBeacon)) continue;

            LoraBeacon b;
            std::memcpy(&b, raw.data(), sizeof(LoraBeacon));

            SwarmNeighbour& sw = neighbours_[b.droneId];
            sw.droneId      = b.droneId;
            sw.lat          = b.lat_ud / 1e6;
            sw.lon          = b.lon_ud / 1e6;
            sw.heading      = b.heading_cd / 100.0f;
            sw.speed_ms     = b.speed_mms  / 1000.0f;
            sw.cellId       = b.cellId;
            sw.cellsVisited = b.cellsVisited;
            sw.posStdM      = (float)b.posStdM;
            sw.isLost       = (b.flags & 0x02) != 0;
            sw.rssi         = n.rssi;
            sw.rangeMetre   = n.distanceMetres;   // ToF distance
            sw.lastSeenMs   = nowMs;
        }

        // Evict stale neighbours
        long long timeout = (long long)(NEIGHBOUR_TIMEOUT * 1000.0f);
        for (auto it = neighbours_.begin(); it != neighbours_.end(); )
        {
            if (nowMs - it->second.lastSeenMs > timeout)
                it = neighbours_.erase(it);
            else
                ++it;
        }
    }

    // ── Check if a given cellId is already targeted by a neighbour ──
    // Used by GridPatrol to prefer uncontested cells
    bool isCellTakenByNeighbour(int cellId, int ownDroneId) const
    {
        for (auto& [id, n] : neighbours_)
        {
            if (id == ownDroneId) continue;
            if (n.cellId == cellId && !n.isLost) return true;
        }
        return false;
    }

    // ── Collision avoidance: list drones within range ─────────────
    struct NearbyDrone { int id; float dist; float bearing; };
    std::vector<NearbyDrone> nearbyDrones(double myLat, double myLon,
                                           float alertRangeM) const
    {
        std::vector<NearbyDrone> out;
        for (auto& [id, n] : neighbours_)
        {
            if (n.isLost) continue;
            float dy  = (float)((n.lat - myLat) * GRID_METRES_PER_LAT_S);
            float dx  = (float)((n.lon - myLon) * GRID_METRES_PER_LON_S);
            float dist= std::sqrt(dx*dx + dy*dy);
            if (dist < alertRangeM)
            {
                float bear = std::atan2(dx, dy) * 180.0f / 3.14159265f;
                if (bear < 0.0f) bear += 360.0f;
                out.push_back({id, dist, bear});
            }
        }
        return out;
    }

    const std::unordered_map<int, SwarmNeighbour>& allNeighbours() const
    { return neighbours_; }

    bool shouldBroadcastThisTick()
    {
        tick_++;
        return (tick_ % BROADCAST_EVERY_N == 0);
    }

private:
    static constexpr double GRID_METRES_PER_LAT_S = 111320.0;
    static constexpr double GRID_METRES_PER_LON_S = 63990.0;

    std::unordered_map<int, SwarmNeighbour> neighbours_;
    int tick_ = 0;
};
