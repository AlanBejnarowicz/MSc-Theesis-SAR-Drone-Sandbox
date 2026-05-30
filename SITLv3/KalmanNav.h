#pragma once
// ================================================================
// KalmanNav.h  —  Simple 4-state navigation Kalman filter
//
// State:  [lat_deg, lon_deg, vx_ms, vz_ms]
//         vx = east velocity (m/s)
//         vz = north velocity (m/s)
//
// Inputs:
//   predict()     — called every tick with dt (seconds)
//                   optionally with IMU accel if available
//   updateGPS()   — called when GPS packet arrives
//   updateSpeed() — called with speed+heading from compass/velocity
//
// Design rules:
//   - All P stored in METRES (not degrees) — no unit mixing
//   - Position states converted to/from degrees only at boundaries
//   - No LoRa, no heading state — add those later once GPS works
//   - GPS accuracy=0 → assumed 5m (SITL GPS is perfect, be conservative)
// ================================================================

#include <cmath>
#include <array>
#include <algorithm>

// Baltic Sea ~54°N  (update if operating elsewhere)
static constexpr double KN_MPL     = 111111.0;   // metres per degree lat
static constexpr double KN_MPL_LON = 64528.0;    // metres per degree lon

class KalmanNav
{
public:
    // ── Estimated state — read these ─────────────────────────────
    double lat     = 0.0;   // degrees
    double lon     = 0.0;   // degrees
    float  vx      = 0.0f;  // m/s east
    float  vz      = 0.0f;  // m/s north

    bool   ready   = false; // true after first GPS fix

    // ── Standard deviations (metres) for display ─────────────────
    float  stdLat  = 999.f;
    float  stdLon  = 999.f;

    // ── Initialise on first GPS fix ───────────────────────────────
    void init(double initLat, double initLon)
    {
        lat   = initLat;
        lon   = initLon;
        vx    = vz = 0.0f;

        // P in metres²
        P[0]  = 25.0f;   // lat: 5m std
        P[1]  = 0.0f;    // lat-lon cross
        P[2]  = 0.0f;    // lat-vx cross
        P[3]  = 0.0f;    // lat-vz cross
        P[4]  = 25.0f;   // lon: 5m std
        P[5]  = 0.0f;    // lon-vx cross
        P[6]  = 0.0f;    // lon-vz cross
        P[7]  = 1.0f;    // vx: 1 m/s std²
        P[8]  = 0.0f;    // vx-vz cross
        P[9]  = 1.0f;    // vz: 1 m/s std²

        ready = true;
        _updateStd();
    }

    // ── Predict ───────────────────────────────────────────────────
    // Call every control tick.
    // accel_x/z: body-frame acceleration (m/s²), 0 if no IMU.
    // heading_deg: current compass heading, needed to rotate IMU.
    void predict(float dt, float accel_x = 0.f, float accel_z = 0.f,
                 float heading_deg = 0.f)
    {
        if (!ready || dt <= 0.f || dt > 0.5f) return;

        // Rotate body accel to world frame if IMU available
        float ax = 0.f, az = 0.f;
        if (accel_x != 0.f || accel_z != 0.f)
        {
            float h = heading_deg * 3.14159265f / 180.f;
            ax = accel_x * std::cos(h) - accel_z * std::sin(h);
            az = accel_x * std::sin(h) + accel_z * std::cos(h);
        }

        // Velocity decay (3s time constant) prevents unbounded IMU drift
        float decay = std::exp(-dt / 3.f);
        vx = vx * decay + ax * dt;
        vz = vz * decay + az * dt;

        // Integrate position (metres → degrees)
        lat += (double)(vz * dt) / KN_MPL;
        lon += (double)(vx * dt) / KN_MPL_LON;

        // Covariance predict: P = F*P*F' + Q
        // F (in metres): pos += vel*dt, vel decays
        // Simplified: only the key coupling terms
        // p_lat += 2*dt*p_latz + dt²*p_zz   (lat-vz coupling)
        // p_lon += 2*dt*p_lonx + dt²*p_xx   (lon-vx coupling)
        float dt2 = dt * dt;

        // Update off-diagonal couplings first
        float new_p_lat_vz = P[3] * decay + P[0] * dt;   // cov(lat, vz)
        float new_p_lon_vx = P[5] * decay + P[4] * dt;   // cov(lon, vx) — P[5]=lon-vx

        // Position variances grow with velocity variance
        P[0] += 2.f * dt * P[3] + dt2 * P[9];   // var(lat) via vz
        P[4] += 2.f * dt * P[5] + dt2 * P[7];   // var(lon) via vx

        // Velocity variances decay²
        P[7] *= decay * decay;
        P[9] *= decay * decay;

        // Update cross terms
        P[3] = new_p_lat_vz;
        P[5] = new_p_lon_vx;

        // Process noise Q (metres²)
        P[0] += Q_pos;    // lat
        P[4] += Q_pos;    // lon
        P[7] += Q_vel;    // vx
        P[9] += Q_vel;    // vz

        _updateStd();
    }

    // ── GPS update ────────────────────────────────────────────────
    // accuracyM: 1-sigma metres. Pass 0 if unknown (defaults to 5m).
    void updateGPS(double gpsLat, double gpsLon, float accuracyM = 0.f)
    {
        if (!ready) return;

        float R = (accuracyM > 0.5f) ? accuracyM * accuracyM : 25.f; // 5m default

        // Innovation in metres
        float inn_lat = (float)((gpsLat - lat) * KN_MPL);
        float inn_lon = (float)((gpsLon - lon) * KN_MPL_LON);

        // Scalar update for lat (H=[1,0,0,0])
        float S_lat = P[0] + R;
        float K_lat = P[0] / S_lat;            // Kalman gain for lat
        float K_vz  = P[3] / S_lat;            // gain propagated to vz via coupling

        lat += K_lat * inn_lat / KN_MPL;
        vz  += K_vz  * inn_lat;
        // Update P after lat measurement
        P[0] *= (1.f - K_lat);
        P[3] *= (1.f - K_lat);
        P[9] -= K_vz * P[3];                   // var(vz) reduced

        // Scalar update for lon (H=[0,1,0,0])
        float S_lon = P[4] + R;
        float K_lon = P[4] / S_lon;
        float K_vx  = P[5] / S_lon;

        lon += K_lon * inn_lon / KN_MPL_LON;
        vx  += K_vx  * inn_lon;
        P[4] *= (1.f - K_lon);
        P[5] *= (1.f - K_lon);
        P[7] -= K_vx * P[5];

        // Clamp variances to avoid going negative from numerical drift
        P[0] = std::max(P[0], 0.01f);
        P[4] = std::max(P[4], 0.01f);
        P[7] = std::max(P[7], 0.01f);
        P[9] = std::max(P[9], 0.01f);

        _updateStd();
    }

    // ── Speed+heading update ──────────────────────────────────────
    // Use when you know speed (knots or m/s) and compass heading.
    // This updates vx/vz directly as a measurement.
    void updateSpeedHeading(float speed_ms, float heading_deg)
    {
        if (!ready) return;

        float h  = heading_deg * 3.14159265f / 180.f;
        float mVx = speed_ms * std::sin(h);   // east
        float mVz = speed_ms * std::cos(h);   // north

        float R_v = 0.5f * 0.5f;   // 0.5 m/s speed measurement noise

        // vx update
        float inn_vx = mVx - vx;
        float S_vx   = P[7] + R_v;
        float K_vx   = P[7] / S_vx;
        float K_lon  = P[5] / S_vx;
        vx  += K_vx  * inn_vx;
        lon += K_lon * inn_vx / KN_MPL_LON;
        P[7] *= (1.f - K_vx);
        P[5] *= (1.f - K_vx);

        // vz update
        float inn_vz = mVz - vz;
        float S_vz   = P[9] + R_v;
        float K_vz   = P[9] / S_vz;
        float K_lat  = P[3] / S_vz;
        vz  += K_vz  * inn_vz;
        lat += K_lat * inn_vz / KN_MPL;
        P[9] *= (1.f - K_vz);
        P[3] *= (1.f - K_vz);

        P[7] = std::max(P[7], 0.01f);
        P[9] = std::max(P[9], 0.01f);

        _updateStd();
    }

    // ── Tuning knobs ──────────────────────────────────────────────
    // Q_pos: how much position uncertainty grows per tick without GPS
    //        higher = trust GPS more, filter reacts faster
    // Q_vel: how much velocity uncertainty grows per tick
    //        higher = velocity estimate is less trusted between updates
    float Q_pos = 0.5f;   // m² per tick — grows during packet gaps
    float Q_vel = 0.5f;   // (m/s)² per tick

private:
    // Upper-triangle covariance (metres²):
    // [0]=var(lat_m)  [1]=cov(lat,lon)  [2]=cov(lat,vx)  [3]=cov(lat,vz)
    //                 [4]=var(lon_m)    [5]=cov(lon,vx)  [6]=cov(lon,vz)
    //                                   [7]=var(vx)      [8]=cov(vx,vz)
    //                                                    [9]=var(vz)
    float P[10] = {};

    void _updateStd()
    {
        stdLat = std::sqrt(std::max(P[0], 0.f));
        stdLon = std::sqrt(std::max(P[4], 0.f));
    }
};
