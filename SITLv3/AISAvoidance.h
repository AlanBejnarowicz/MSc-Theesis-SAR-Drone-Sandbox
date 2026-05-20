#pragma once
// ================================================================
// AISAvoidance.h  —  Direct command override for AIS vessel avoidance
//
// No circular include: takes raw data (packets + kalman), not DroneAgent.
// ================================================================

#include "packets.h"
#include "KalmanNav.h"
#include <cmath>
#include <algorithm>
#include <string>
#include <limits>

class AISAvoidance
{
public:
    static constexpr float MIN_RADIUS_M  = 300.f;
    static constexpr float MAX_RADIUS_M  = 1000.f;
    static constexpr float MAX_SHIP_LEN  = 300.f;
    static constexpr float SPEED_STRETCH = 0.2f;
    static constexpr float BEHIND_FACTOR = 0.4f;

    static constexpr float ZONE_DANGER   = 0.4f;
    static constexpr float ZONE_WARNING  = 0.7f;

    static constexpr float STEER_DANGER  = 1.0f;
    static constexpr float STEER_WARNING = 0.7f;
    static constexpr float STEER_CAUTION = 0.3f;

    static constexpr float THROTTLE_DANGER  = 0.25f;
    static constexpr float THROTTLE_WARNING = 0.55f;
    static constexpr float THROTTLE_CAUTION = 1.0f;

    enum class Zone { CLEAR, CAUTION, WARNING, DANGER };

    struct Result
    {
        Zone  zone          = Zone::CLEAR;
        bool  active        = false;
        float steer         = 0.f;
        float throttleMul   = 1.f;
        float closestDMaha  = 1.f;
        int   shipCount     = 0;
    };

    // ── Main evaluation — pass kalman + state directly ────────────
    static Result evaluate(const KalmanNav&        kalman,
                           const DroneStatePacket& state)
    {
        Result result;
        if (!kalman.ready) return result;

        float myHdg      = state.compass.heading;
        float worstDMaha = std::numeric_limits<float>::max();
        float worstSteer = 0.f;
        float worstThrMul= 1.f;
        Zone  worstZone  = Zone::CLEAR;

        for (const auto& contact : state.ais.contacts)
        {
            if (contact.vesselName.rfind("UAV_", 0) == 0) continue;

            float baseRadius = radiusForShip(contact);
            float shipSpeed  = contact.speed_knots * 0.5144f;

            float dx = (float)((kalman.lon - contact.lon) * 63990.0);
            float dz = (float)((kalman.lat - contact.lat) * 111320.0);
            float dEuclid = std::sqrt(dx*dx + dz*dz);
            if (dEuclid < 1.f) continue;

            float hRad = contact.heading * 3.14159265f / 180.f;
            float ux   =  std::sin(hRad);
            float uz   =  std::cos(hRad);
            float px   = -uz, pz = ux;

            float along = dx * ux + dz * uz;
            float perp  = dx * px + dz * pz;

            float sAlong = (along >= 0.f)
                           ? baseRadius * (1.f + SPEED_STRETCH * shipSpeed)
                           : baseRadius * BEHIND_FACTOR;
            float sPerp  = baseRadius;

            float dMaha = std::sqrt(  (along/sAlong)*(along/sAlong)
                                    + (perp /sPerp )*(perp /sPerp ));
            if (dMaha >= 1.f) continue;

            result.shipCount++;
            if (dMaha < result.closestDMaha)
                result.closestDMaha = dMaha;

            // Escape perpendicular to ship track, on the side we're already on
            float escapeBearing = (perp >= 0.f)
                                  ? contact.heading - 90.f
                                  : contact.heading + 90.f;
            while (escapeBearing <   0.f) escapeBearing += 360.f;
            while (escapeBearing > 360.f) escapeBearing -= 360.f;

            float hdgErr   = _deltaAngle(myHdg, escapeBearing);
            float steerSign= (hdgErr >= 0.f) ? 1.f : -1.f;

            float steerMag, thrMul;
            Zone  zone;
            if (dMaha < ZONE_DANGER) {
                steerMag = STEER_DANGER;
                thrMul   = THROTTLE_DANGER;
                zone     = Zone::DANGER;
            } else if (dMaha < ZONE_WARNING) {
                float t  = (dMaha - ZONE_DANGER) / (ZONE_WARNING - ZONE_DANGER);
                steerMag = STEER_DANGER   + t * (STEER_WARNING   - STEER_DANGER);
                thrMul   = THROTTLE_DANGER+ t * (THROTTLE_WARNING - THROTTLE_DANGER);
                zone     = Zone::WARNING;
            } else {
                float t  = (dMaha - ZONE_WARNING) / (1.f - ZONE_WARNING);
                steerMag = STEER_WARNING + t * (STEER_CAUTION  - STEER_WARNING);
                thrMul   = THROTTLE_WARNING + t * (THROTTLE_CAUTION - THROTTLE_WARNING);
                zone     = Zone::CAUTION;
            }

            if (dMaha < worstDMaha) {
                worstDMaha  = dMaha;
                worstSteer  = steerSign * steerMag;
                worstThrMul = thrMul;
                worstZone   = zone;
            }
        }

        if (result.shipCount > 0) {
            result.active      = true;
            result.zone        = worstZone;
            result.steer       = worstSteer;
            result.throttleMul = worstThrMul;
        }
        return result;
    }

    // ── Modify cmd in place ───────────────────────────────────────
    static void applyToCmd(const KalmanNav&        kalman,
                           const DroneStatePacket& state,
                           DroneCommandPacket&     cmd)
    {
        Result r = evaluate(kalman, state);
        if (!r.active) return;
        cmd.steer    = r.steer;
        cmd.throttle = std::clamp(cmd.throttle * r.throttleMul, 0.f, 1.f);
    }

    static float radiusForShip(const AIS_Contact& contact)
    {
        float t = std::clamp(estimateLength(contact) / MAX_SHIP_LEN, 0.f, 1.f);
        return MIN_RADIUS_M + t * (MAX_RADIUS_M - MIN_RADIUS_M);
    }

    static const char* zoneName(Zone z)
    {
        switch(z) {
            case Zone::DANGER:  return "DANGER";
            case Zone::WARNING: return "WARNING";
            case Zone::CAUTION: return "CAUTION";
            default:            return "CLEAR";
        }
    }

private:
    static float estimateLength(const AIS_Contact& c)
    {
        const std::string& t = c.shipType;
        if (t == "Tanker")          return 250.f;
        if (t == "Cargo")           return 180.f;
        if (t == "Passenger")       return 200.f;
        if (t == "HighSpeedCraft")  return  40.f;
        if (t == "Tug")             return  35.f;
        if (t == "Fishing")         return  25.f;
        if (t == "Sailing")         return  20.f;
        if (t == "PleasureCraft")   return  12.f;
        if (t == "PilotVessel")     return  20.f;
        if (t == "SearchAndRescue") return  25.f;
        if (t == "AtoN")            return   5.f;
        return 60.f;
    }

    static float _deltaAngle(float from, float to)
    {
        float d = to - from;
        while (d >  180.f) d -= 360.f;
        while (d < -180.f) d += 360.f;
        return d;
    }
};