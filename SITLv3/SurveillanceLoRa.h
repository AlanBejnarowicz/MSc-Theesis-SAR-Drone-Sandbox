#pragma once
// ================================================================
// SurveillanceLoRa.h  —  Compact LoRa protocol for grid sharing
//
// Each drone broadcasts a small beacon every N ticks containing:
//   - Its drone ID
//   - Its current target cell ID
//   - Its current cell ID (where it is now)
//   - Up to MAX_UPDATES recent cell visit timestamps
//     (prioritised: stalest cells first so neighbours learn fastest)
//
// Beacon wire format (binary, base64 encoded):
//   Byte 0      : droneId         (uint8)
//   Byte 1-2    : currentCell     (int16, -1 = none)
//   Byte 3-4    : occupiedCell    (int16, cell drone is in now)
//   Byte 5      : numUpdates      (uint8, 0-MAX_UPDATES)
//   For each update (5 bytes each):
//     Byte 0-1  : cellId          (uint16)
//     Byte 2-4  : age_seconds     (uint24, seconds since visit, max 16M)
//
// Max payload with MAX_UPDATES=6: 6 + 6*5 = 36 bytes → 48 base64 chars
// Well within LoRa practical limits.
// ================================================================

#include "SurveillanceGrid.h"
#include "packets.h"
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

// ── Minimal base64 ────────────────────────────────────────────────
static const char SL_B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string sl_b64enc(const uint8_t* data, size_t len)
{
    std::string out;
    out.reserve(((len+2)/3)*4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)data[i] << 16;
        if (i+1<len) v |= (uint32_t)data[i+1]<<8;
        if (i+2<len) v |= data[i+2];
        out += SL_B64[(v>>18)&63];
        out += SL_B64[(v>>12)&63];
        out += (i+1<len) ? SL_B64[(v>>6)&63] : '=';
        out += (i+2<len) ? SL_B64[v&63]      : '=';
    }
    return out;
}

static std::vector<uint8_t> sl_b64dec(const std::string& s)
{
    auto dc = [](char c) -> int {
        if (c>='A'&&c<='Z') return c-'A';
        if (c>='a'&&c<='z') return c-'a'+26;
        if (c>='0'&&c<='9') return c-'0'+52;
        if (c=='+') return 62;
        if (c=='/') return 63;
        return -1;
    };
    std::vector<uint8_t> out;
    for (size_t i=0; i+3<s.size(); i+=4) {
        int a=dc(s[i]),b=dc(s[i+1]),c=dc(s[i+2]),d=dc(s[i+3]);
        if (a<0||b<0) break;
        out.push_back((uint8_t)((a<<2)|(b>>4)));
        if (c>=0) out.push_back((uint8_t)((b<<4)|(c>>2)));
        if (d>=0) out.push_back((uint8_t)((c<<6)|d));
    }
    return out;
}

// ── Beacon ────────────────────────────────────────────────────────
class SurveillanceLoRa
{
public:
    static constexpr int MAX_UPDATES     = 6;    // cell updates per beacon
    static constexpr int BROADCAST_EVERY = 15;   // ticks between broadcasts

    // ── Encode: build beacon from grid state ──────────────────────
    static std::string encode(const SurveillanceGrid& grid,
                              int droneId,
                              double myLat, double myLon)
    {
        // Find which cell we're currently IN (for occupancy sharing)
        int occupiedCell = -1;
        for (auto& c : grid.cells()) {
            float d = SurveillanceGrid::distM(myLat,myLon,c.centreLat,c.centreLon);
            if (d < SurveillanceGrid::ARRIVAL_DIST_M) {
                occupiedCell = c.id;
                break;
            }
        }

        // Pick cells with most recent visits to share
        // (neighbours benefit most from learning fresh data)
        struct CellAge { int id; long long visitMs; };
        std::vector<CellAge> visited;
        for (auto& c : grid.cells())
            if (c.lastVisitMs > 0)
                visited.push_back({c.id, c.lastVisitMs});

        // Sort by most recently visited (they send newest info first)
        std::sort(visited.begin(), visited.end(),
                  [](const CellAge& a, const CellAge& b){
                      return a.visitMs > b.visitMs; });

        int numUp = (int)std::min((int)visited.size(), MAX_UPDATES);

        // Pack binary buffer
        std::vector<uint8_t> buf;
        buf.reserve(6 + numUp * 5);

        auto push8  = [&](uint8_t v)  { buf.push_back(v); };
        auto push16 = [&](int16_t v)  {
            buf.push_back((uint8_t)(v>>8));
            buf.push_back((uint8_t)(v&0xFF));
        };
        auto push24 = [&](uint32_t v) {
            buf.push_back((uint8_t)((v>>16)&0xFF));
            buf.push_back((uint8_t)((v>> 8)&0xFF));
            buf.push_back((uint8_t)( v     &0xFF));
        };

        push8 ((uint8_t)std::clamp(droneId, 0, 255));
        push16((int16_t)std::clamp(grid.currentTargetId(), -1, 32767));
        push16((int16_t)std::clamp(occupiedCell,           -1, 32767));
        push8 ((uint8_t)numUp);

        long long now = sg_nowMs();
        for (int i = 0; i < numUp; i++) {
            push16((uint16_t)visited[i].id);
            uint32_t ageSec = (uint32_t)std::clamp(
                (long long)((now - visited[i].visitMs)/1000), 0LL, (long long)0xFFFFFF);
            push24(ageSec);
        }

        return sl_b64enc(buf.data(), buf.size());
    }

    // ── Decoded neighbour info ────────────────────────────────────
    struct NeighbourBeacon
    {
        int droneId      = -1;
        int targetCell   = -1;   // cell this drone is heading to
        int occupiedCell = -1;   // cell this drone is currently in
        struct VisitUpdate { int cellId; long long visitMs; };
        std::vector<VisitUpdate> updates;
    };

    // ── Decode a raw base64 payload from a LoRa neighbour ─────────
    static bool decode(const std::string& payload, NeighbourBeacon& out)
    {
        auto buf = sl_b64dec(payload);
        if (buf.size() < 6) return false;

        auto r8  = [&](int i) -> uint8_t  { return buf[i]; };
        auto r16 = [&](int i) -> int16_t  {
            return (int16_t)((buf[i]<<8)|buf[i+1]); };
        auto r24 = [&](int i) -> uint32_t {
            return ((uint32_t)buf[i]<<16)|((uint32_t)buf[i+1]<<8)|buf[i+2]; };

        out.droneId      = r8(0);
        out.targetCell   = r16(1);
        out.occupiedCell = r16(3);
        int numUp        = r8(5);

        out.updates.clear();
        long long now = sg_nowMs();
        int offset = 6;
        for (int i = 0; i < numUp && offset+4 < (int)buf.size(); i++, offset+=5)
        {
            int      cellId  = (uint16_t)r16(offset);
            uint32_t ageSec  = r24(offset+2);
            long long visitMs = now - (long long)ageSec * 1000;
            out.updates.push_back({cellId, visitMs});
        }
        return out.droneId >= 0;
    }

    // ── Process all LoRa neighbours, update grid ──────────────────
    // Call once per tick after receiveAndTick().
    static void processLoRa(const LoRa_Block& lora, SurveillanceGrid& grid)
    {
        for (auto& neighbour : lora.neighbours)
        {
            if (neighbour.lastPayload.empty()) continue;

            NeighbourBeacon beacon;
            if (!decode(neighbour.lastPayload, beacon)) continue;

            // Don't process our own echoed packets
            if (beacon.droneId == grid.ownDroneId) continue;

            // Tell grid which cell this neighbour is targeting
            grid.setNeighbourTarget(beacon.droneId, beacon.targetCell);

            // Merge visit timestamps — accept newer info only
            for (auto& upd : beacon.updates)
                grid.mergeVisit(upd.cellId, upd.visitMs);
        }
    }

    // ── Should we broadcast this tick? ───────────────────────────
    bool shouldBroadcast()
    {
        return (++_tick % BROADCAST_EVERY == 0);
    }

private:
    int _tick = 0;
};