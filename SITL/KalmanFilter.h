#pragma once
// ================================================================
// KalmanFilter.h  —  EKF, 5-state, fully consistent units
//
// State:   [lat_deg, lon_deg, vx_ms, vz_ms, hdg_deg]
//
// P is stored in its natural units per state:
//   P[0,0]  deg²   (lat)
//   P[1,1]  deg²   (lon)
//   P[2,2]  (m/s)²
//   P[3,3]  (m/s)²
//   P[4,4]  deg²   (heading)
//
// All innovations and Kalman gains are computed consistently.
// No mixed metres/degrees in the covariance matrix.
// ================================================================

#include <cmath>
#include <array>
#include <algorithm>

using Mat5 = std::array<double, 25>;   // double for numerical stability

static constexpr double KF_DEG2RAD   = 3.14159265358979 / 180.0;
static constexpr double KF_MPL       = 111320.0;          // metres per degree lat
static constexpr double KF_MPL_LON   = 63990.0;           // metres per degree lon (54°N)

// ── Tiny matrix helpers ───────────────────────────────────────────
inline Mat5 m5_eye()
{ Mat5 m{}; m[0]=m[6]=m[12]=m[18]=m[24]=1.0; return m; }

inline Mat5 m5_mul(const Mat5& A, const Mat5& B)
{ Mat5 C{}; for(int r=0;r<5;r++) for(int c=0;c<5;c++) for(int k=0;k<5;k++) C[r*5+c]+=A[r*5+k]*B[k*5+c]; return C; }

inline Mat5 m5_add(const Mat5& A, const Mat5& B)
{ Mat5 C; for(int i=0;i<25;i++) C[i]=A[i]+B[i]; return C; }

inline Mat5 m5_T(const Mat5& A)
{ Mat5 T; for(int r=0;r<5;r++) for(int c=0;c<5;c++) T[c*5+r]=A[r*5+c]; return T; }

class KalmanFilter
{
public:
    // ── Public state (read these) ──────────────────────────────────
    double lat     = 0.0;    // degrees
    double lon     = 0.0;    // degrees
    float  vx      = 0.0f;   // m/s  east
    float  vz      = 0.0f;   // m/s  north
    float  heading = 0.0f;   // degrees 0-360

    bool isReady() const { return init_; }

    // ── posStdM: position 1-sigma in metres (for display) ─────────
    float posStdM() const
    {
        double vLat = P_[0*5+0];   // deg²
        double vLon = P_[1*5+1];   // deg²
        double mLat = std::sqrt(std::max(0.0, vLat)) * KF_MPL;
        double mLon = std::sqrt(std::max(0.0, vLon)) * KF_MPL_LON;
        float result = (float)((mLat + mLon) * 0.5);
        return init_ ? std::max(result, 0.1f) : 0.0f;
    }

    // ── Init ──────────────────────────────────────────────────────
    void init(double iLat, double iLon, float iHdg)
    {
        lat = iLat; lon = iLon; heading = iHdg; vx = vz = 0.0f;

        P_ = {};
        // Diagonal only — off-diagonal starts at 0
        // lat/lon: 5m initial std  → in degrees: 5/MPL
        double posStdDeg_lat = 5.0 / KF_MPL;
        double posStdDeg_lon = 5.0 / KF_MPL_LON;
        P_[0*5+0] = posStdDeg_lat * posStdDeg_lat;
        P_[1*5+1] = posStdDeg_lon * posStdDeg_lon;
        P_[2*5+2] = 1.0;      // vx: 1 m/s std
        P_[3*5+3] = 1.0;      // vz: 1 m/s std
        P_[4*5+4] = 100.0;    // hdg: 10° std
        init_ = true;
    }

    // ── Predict: IMU dead-reckoning, dt in seconds ─────────────────
    void predict(float accel_x, float accel_z, float dt)
    {
        if (!init_ || dt <= 0.0f || dt > 0.2f) return;

        // Rotate body accel → world frame
        double hRad = heading * KF_DEG2RAD;
        double sH = std::sin(hRad), cH = std::cos(hRad);
        double ax = accel_x * cH - accel_z * sH;
        double az = accel_x * sH + accel_z * cH;

        // Velocity with damping (3 s time constant) to limit IMU drift
        double decay = std::exp(-dt / 3.0);
        vx = (float)(vx * decay + ax * dt);
        vz = (float)(vz * decay + az * dt);

        // Integrate position in degrees
        lat += vz * dt / KF_MPL;
        lon += vx * dt / KF_MPL_LON;

        // ── F matrix ─────────────────────────────────────────────
        // d(lat_deg)/d(vz) = dt/MPL
        // d(lon_deg)/d(vx) = dt/MPL_LON
        Mat5 F = m5_eye();
        F[0*5+3] = dt / KF_MPL;        // lat  ← vz
        F[1*5+2] = dt / KF_MPL_LON;    // lon  ← vx

        // ── Process noise Q in state units ────────────────────────
        // pos noise: 0.1 m/tick → in degrees
        double qp_lat = (0.1 / KF_MPL)    * (0.1 / KF_MPL);
        double qp_lon = (0.1 / KF_MPL_LON)* (0.1 / KF_MPL_LON);
        Mat5 Q{};
        Q[0*5+0] = qp_lat;
        Q[1*5+1] = qp_lon;
        Q[2*5+2] = 0.3;     // vx noise (m/s)²
        Q[3*5+3] = 0.3;     // vz noise (m/s)²
        Q[4*5+4] = 0.5;     // hdg noise deg²

        P_ = m5_add(m5_mul(m5_mul(F, P_), m5_T(F)), Q);
    }

    // ── GPS update ────────────────────────────────────────────────
    void updateGPS(double gpsLat, double gpsLon, float accuracyM)
    {
        if (!init_) return;
        if (accuracyM < 0.5f) accuracyM = 3.0f;   // Unity often sends 0
        accuracyM = std::min(accuracyM, 30.0f);

        // Convert accuracy to degrees² for each axis
        // Hard minimum: 2m equivalent so P never collapses to zero
        double minAccM = 2.0f;
        double R_lat = std::max((double)(accuracyM / KF_MPL)    * (accuracyM / KF_MPL),
                                (minAccM / KF_MPL)    * (minAccM / KF_MPL));
        double R_lon = std::max((double)(accuracyM / KF_MPL_LON) * (accuracyM / KF_MPL_LON),
                                (minAccM / KF_MPL_LON) * (minAccM / KF_MPL_LON));

        // Innovation directly in degrees (same units as state)
        scalarUpdate(0, gpsLat - lat, R_lat);
        scalarUpdate(1, gpsLon - lon, R_lon);
    }

    // ── Compass update ────────────────────────────────────────────
    void updateCompass(float measHdg, float noiseSigDeg)
    {
        if (!init_) return;
        if (noiseSigDeg < 0.5f) noiseSigDeg = 2.0f;
        double R = (double)noiseSigDeg * noiseSigDeg;

        double innov = measHdg - heading;
        while (innov >  180.0) innov -= 360.0;
        while (innov < -180.0) innov += 360.0;

        scalarUpdate(4, innov, R);
        while (heading >= 360.0f) heading -= 360.0f;
        while (heading <    0.0f) heading += 360.0f;
    }

    // ── LoRa ToF range update ─────────────────────────────────────
    void updateLoRaRange(double nLat, double nLon,
                         float rangeMeasured, float rangeStdM)
    {
        if (!init_ || rangeMeasured < 1.0f || rangeMeasured > 2000.0f) return;

        // Predicted range
        double dy = (nLat - lat) * KF_MPL;
        double dx = (nLon - lon) * KF_MPL_LON;
        double pred = std::sqrt(dx*dx + dy*dy);
        if (pred < 0.1) return;

        double innov = rangeMeasured - pred;

        // Jacobian in degree units:
        // H[lat] = -(dy/pred) / KF_MPL    (∂range/∂lat_deg)
        // H[lon] = -(dx/pred) / KF_MPL_LON
        double H0 = -(dy / pred) / KF_MPL;
        double H1 = -(dx / pred) / KF_MPL_LON;
        double R  = (double)rangeStdM * rangeStdM;

        double S = H0*H0*P_[0] + 2.0*H0*H1*P_[1] + H1*H1*P_[6] + R;
        if (S < 1e-12) return;

        double K0 = (H0*P_[0] + H1*P_[1]) / S;
        double K1 = (H0*P_[1] + H1*P_[6]) / S;

        lat     += K0 * innov;
        lon     += K1 * innov;

        for (int j = 0; j < 5; j++) {
            double p0j = P_[0*5+j], p1j = P_[1*5+j];
            P_[0*5+j] -= K0 * (H0*p0j + H1*p1j);
            P_[1*5+j] -= K1 * (H0*p0j + H1*p1j);
        }
    }

private:
    Mat5 P_{};
    bool init_ = false;

    // Scalar update: state[idx] += K*innov,  H=1 direct measurement
    void scalarUpdate(int idx, double innov, double R)
    {
        double Pii = P_[idx*5+idx];
        double S   = Pii + R;
        if (S < 1e-20) return;
        double K = Pii / S;

        double delta = K * innov;
        switch(idx) {
            case 0: lat     += delta;          break;
            case 1: lon     += delta;          break;
            case 2: vx      += (float)delta;   break;
            case 3: vz      += (float)delta;   break;
            case 4: heading += (float)delta;   break;
        }

        // Joseph-form row/col update
        double f = 1.0 - K;
        for (int j = 0; j < 5; j++) {
            P_[idx*5+j] *= f;
            P_[j*5+idx] *= f;
        }
    }
};