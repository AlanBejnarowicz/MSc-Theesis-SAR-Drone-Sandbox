#pragma once
// ================================================================
// KalmanFilter.h  —  Extended Kalman Filter for drone state estimation
//
// State vector X = [lat, lon, vx, vz, heading]  (5 states)
//
// Sensors fused:
//   - GPS      (lat/lon, ~2-5m accuracy)
//   - IMU      (accel x/z → velocity prediction)
//   - Compass  (heading update)
//   - LoRa ToF (range to neighbour → position constraint)
//
// Units: lat/lon in degrees, velocity in m/s, heading in degrees
// ================================================================

#include <cmath>
#include <array>
#include <algorithm>

// ── 5x5 matrix helpers (flat row-major arrays) ───────────────────
using Mat5   = std::array<float, 25>;
using Vec5   = std::array<float, 5>;

static constexpr float DEG2RAD = 3.14159265f / 180.0f;
static constexpr float RAD2DEG = 180.0f / 3.14159265f;

// Rough metres-per-degree constants for Baltic Sea (~54°N)
static constexpr double METRES_PER_LAT = 111320.0;
static constexpr double METRES_PER_LON = 63990.0;  // cos(54°) * 111320

inline Mat5 mat5_identity()
{
    Mat5 m{};
    m[0]=m[6]=m[12]=m[18]=m[24]=1.0f;
    return m;
}

inline Mat5 mat5_mul(const Mat5& A, const Mat5& B)
{
    Mat5 C{};
    for(int r=0;r<5;r++)
        for(int c=0;c<5;c++)
            for(int k=0;k<5;k++)
                C[r*5+c] += A[r*5+k] * B[k*5+c];
    return C;
}

inline Mat5 mat5_add(const Mat5& A, const Mat5& B)
{
    Mat5 C;
    for(int i=0;i<25;i++) C[i]=A[i]+B[i];
    return C;
}

inline Mat5 mat5_transpose(const Mat5& A)
{
    Mat5 T;
    for(int r=0;r<5;r++)
        for(int c=0;c<5;c++)
            T[c*5+r]=A[r*5+c];
    return T;
}

// Cofactor expansion 5x5 inverse (numerically fine for small matrices)
// We use a simpler approach: only invert 2x2 sub-block for each update
// (Joseph form per-sensor scalar update for numerical stability)

class KalmanFilter
{
public:
    // ── State ─────────────────────────────────────────────────────
    double lat     = 0.0;   // degrees
    double lon     = 0.0;   // degrees
    float  vx      = 0.0f;  // m/s east
    float  vz      = 0.0f;  // m/s north
    float  heading = 0.0f;  // degrees

    // Covariance diagonal for fast access
    float pos_std  = 10.0f; // metres (initial uncertainty)
    float vel_std  = 1.0f;
    float hdg_std  = 5.0f;

    void init(double initLat, double initLon, float initHeading)
    {
        lat     = initLat;
        lon     = initLon;
        heading = initHeading;
        vx      = 0.0f;
        vz      = 0.0f;

        // Initial covariance
        P_ = mat5_identity();
        P_[0]  = 10.0f * 10.0f;   // lat var (in metres^2)
        P_[6]  = 10.0f * 10.0f;   // lon var
        P_[12] = 1.0f;             // vx var
        P_[18] = 1.0f;             // vz var
        P_[24] = 25.0f;            // heading var (deg^2)

        initialised_ = true;
    }

    // ── Predict step — call every control tick (dt seconds) ───────
    // Uses IMU acceleration to propagate velocity and position
    void predict(float accel_x, float accel_z, float dt)
    {
        if (!initialised_) return;
        if (dt <= 0.0f || dt > 0.5f) return;   // sanity

        // ── Integrate velocity ────────────────────────────────────
        // Rotate body-frame IMU to world frame using current heading
        float hRad = heading * DEG2RAD;
        float sinH = std::sin(hRad);
        float cosH = std::cos(hRad);

        float ax_world = accel_x * cosH - accel_z * sinH;
        float az_world = accel_x * sinH + accel_z * cosH;

        vx += ax_world * dt;
        vz += az_world * dt;

        // ── Integrate position ────────────────────────────────────
        lat += (vz * dt) / METRES_PER_LAT;
        lon += (vx * dt) / METRES_PER_LON;

        // ── Propagate covariance  P = F*P*F' + Q ─────────────────
        // State transition (linearised)
        // F = I + ...velocity→position coupling
        Mat5 F = mat5_identity();
        // d(lat)/d(vz) = dt / METRES_PER_LAT
        F[0*5+3] = dt / (float)METRES_PER_LAT;   // lat from vz
        // d(lon)/d(vx) = dt / METRES_PER_LON
        F[1*5+2] = dt / (float)METRES_PER_LON;   // lon from vx

        P_ = mat5_add(mat5_mul(mat5_mul(F, P_), mat5_transpose(F)), Q_);
    }

    // ── GPS update ────────────────────────────────────────────────
    // Measurement: [lat_m, lon_m] relative noise ~ accuracy metres
    void updateGPS(double gpsLat, double gpsLon, float accuracy)
    {
        if (!initialised_) return;
        accuracy = std::max(accuracy, 1.0f);

        // Innovation in metres
        float innov_lat = (float)((gpsLat - lat) * METRES_PER_LAT);
        float innov_lon = (float)((gpsLon - lon) * METRES_PER_LON);

        float R = accuracy * accuracy;

        // Scalar update for lat (state 0)
        scalarUpdate(0, innov_lat, R, 1.0f / (float)METRES_PER_LAT);
        // Scalar update for lon (state 1)
        scalarUpdate(1, innov_lon, R, 1.0f / (float)METRES_PER_LON);
    }

    // ── Compass update ────────────────────────────────────────────
    void updateCompass(float measuredHeading, float noiseSigma)
    {
        if (!initialised_) return;
        float R = std::max(noiseSigma, 1.0f);
        R = R * R;

        // Wrap innovation to -180..180
        float innov = measuredHeading - heading;
        while (innov >  180.0f) innov -= 360.0f;
        while (innov < -180.0f) innov += 360.0f;

        scalarUpdate(4, innov, R, 1.0f);
        // Wrap heading output
        while (heading >  360.0f) heading -= 360.0f;
        while (heading <  0.0f  ) heading += 360.0f;
    }

    // ── LoRa ToF range update ─────────────────────────────────────
    // Uses range to a neighbour with known position to constrain
    // our position estimate (range-only update, linearised)
    void updateLoRaRange(double neighbourLat, double neighbourLon,
                         float measuredRange, float rangeStdM)
    {
        if (!initialised_) return;
        if (measuredRange < 1.0f || measuredRange > 2000.0f) return;

        float dx = (float)((neighbourLon - lon) * METRES_PER_LON);
        float dy = (float)((neighbourLat - lat) * METRES_PER_LAT);
        float predictedRange = std::sqrt(dx*dx + dy*dy);
        if (predictedRange < 0.1f) return;

        float innov = measuredRange - predictedRange;

        // Measurement Jacobian H = [d_range/d_lat, d_range/d_lon, 0, 0, 0]
        float H_lat = -dy / predictedRange / (float)METRES_PER_LAT;
        float H_lon = -dx / predictedRange / (float)METRES_PER_LON;

        float R = rangeStdM * rangeStdM;

        // S = H*P*H' + R  (scalar)
        float S = H_lat*H_lat*P_[0]
                + H_lon*H_lon*P_[6]
                + 2.0f*H_lat*H_lon*P_[1]
                + R;
        if (S < 1e-6f) return;

        // K = P * H' / S
        float K_lat = (H_lat * P_[0] + H_lon * P_[1]) / S;
        float K_lon = (H_lat * P_[1] + H_lon * P_[6]) / S;
        float K_vx  = (H_lat * P_[2] + H_lon * P_[7]) / S;
        float K_vz  = (H_lat * P_[3] + H_lon * P_[8]) / S;

        // State update
        lat     += K_lat * innov / METRES_PER_LAT;
        lon     += K_lon * innov / METRES_PER_LON;
        vx      += K_vx  * innov;
        vz      += K_vz  * innov;

        // Covariance update P = (I - K*H) * P  (simplified)
        // Just update affected rows/cols 0 and 1
        float KH_lat = K_lat * H_lat;
        float KH_lon = K_lon * H_lon;
        for (int j = 0; j < 5; j++)
        {
            float tmp = P_[0*5+j];
            P_[0*5+j] -= KH_lat * tmp + K_lat * H_lon * P_[1*5+j];
            P_[1*5+j] -= KH_lon * P_[1*5+j] + K_lon * H_lat * tmp;
        }
    }

    // Read-back stddevs (for telemetry / logging)
    float posStdM()  const { return std::sqrt(std::max(0.0f, (P_[0]+P_[6])*0.5f)) * (float)METRES_PER_LAT; }
    float velStdMs() const { return std::sqrt(std::max(0.0f, (P_[12]+P_[18])*0.5f)); }
    float hdgStdDeg()const { return std::sqrt(std::max(0.0f,  P_[24])); }

private:
    Mat5 P_{};          // Covariance
    bool initialised_ = false;

    // ── Process noise Q ───────────────────────────────────────────
    // Tune: higher = trust IMU less, converge to GPS faster
    Mat5 Q_ = [](){
        Mat5 q{};
        q[0]  = 0.01f;    // lat process noise (m^2/s normalised)
        q[6]  = 0.01f;    // lon
        q[12] = 0.1f;     // vx
        q[18] = 0.1f;     // vz
        q[24] = 0.5f;     // heading (deg^2/s)
        return q;
    }();

    // ── Scalar Kalman update (for states measured directly) ───────
    // state_idx: which state element
    // innov:     measured - predicted (in state units)
    // R:         measurement noise variance
    // H_val:     H matrix value (1.0 for direct measurement)
    void scalarUpdate(int idx, float innov, float R, float H_val)
    {
        int i = idx * 5 + idx;
        float S = H_val * H_val * P_[i] + R;
        if (S < 1e-9f) return;
        float K = P_[i] * H_val / S;

        // Update state
        switch(idx) {
            case 0: lat     += K * innov / METRES_PER_LAT; break;
            case 1: lon     += K * innov / METRES_PER_LON; break;
            case 2: vx      += K * innov; break;
            case 3: vz      += K * innov; break;
            case 4: heading += K * innov; break;
        }

        // Update covariance row/col for this state
        float factor = 1.0f - K * H_val;
        for (int j = 0; j < 5; j++)
        {
            P_[idx*5+j] *= factor;
            P_[j*5+idx] *= factor;
        }
    }
};
