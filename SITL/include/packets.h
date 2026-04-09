#pragma once
#include <string>
#include <vector>

// ================================================================
// PACKETS.H  —  Mirror of Unity DroneStatePacket / DroneCommandPacket
// Keep in sync with DroneUDPBridge.cs packet classes
// ================================================================

// ── Incoming from Unity ──────────────────────────────────────────

struct GPS_Block {
    double lat        = 0.0;
    double lon        = 0.0;
    float  accuracy   = 0.0f;
};

struct Compass_Block {
    float heading     = 0.0f;
    float noise_sigma = 0.0f;
};

struct IMU_Block {
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
    float gyro_x  = 0.0f, gyro_y  = 0.0f, gyro_z  = 0.0f;
};

struct Velocity_Block {
    float speed_ms    = 0.0f;
    float speed_knots = 0.0f;
    float vx = 0.0f, vz = 0.0f;
};

struct AIS_Contact {
    int         mmsi        = 0;
    std::string vesselName;
    std::string shipType;
    float       distance    = 0.0f;
    float       bearing     = 0.0f;   // Relative to own heading (-180 to 180)
    float       heading     = 0.0f;
    float       speed_knots = 0.0f;
    double      lat         = 0.0;
    double      lon         = 0.0;
};

struct AIS_Block {
    int                      contactCount = 0;
    std::vector<AIS_Contact> contacts;
};

struct DroneStatePacket {
    int   droneId   = -1;
    int   mmsi      = 0;
    float timestamp = 0.0f;
    int   packetSeq = 0;
    float pos_x     = 0.0f;
    float pos_z     = 0.0f;

    GPS_Block      gps;
    Compass_Block  compass;
    IMU_Block      imu;
    Velocity_Block velocity;
    AIS_Block      ais;
};

// ── Outgoing to Unity ────────────────────────────────────────────

struct DroneCommandPacket {
    int   droneId   = -1;
    int   packetSeq = 0;
    float throttle  = 0.0f;
    float steer     = 0.0f;
};
