#pragma once

// ================================================================
// JSON_PARSER.H  —  Isolated JSON parsing layer
//
// SITL:    uses nlohmann/json (header-only, easy on PC)
// ESP-IDF: swap this file for cJSON implementation
//          Everything else in the project stays identical
// ================================================================

#include "packets.h"
#include <string>

// nlohmann/json — single header, download from:
// https://github.com/nlohmann/json/releases  (json.hpp)
// Place json.hpp in include/ folder
#include "json.hpp"
using json = nlohmann::json;

// ── Parse incoming state packet ──────────────────────────────────
inline bool parseStatePacket(const std::string& raw, DroneStatePacket& out)
{
    try
    {
        json j = json::parse(raw);

        out.droneId   = j.value("droneId",   -1);
        out.mmsi      = j.value("mmsi",        0);
        out.timestamp = j.value("timestamp", 0.0f);
        out.packetSeq = j.value("packetSeq",   0);
        out.pos_x     = j.value("pos_x",     0.0f);
        out.pos_z     = j.value("pos_z",     0.0f);

        // GPS
        if (j.contains("gps")) {
            out.gps.lat      = j["gps"].value("lat",      0.0);
            out.gps.lon      = j["gps"].value("lon",      0.0);
            out.gps.accuracy = j["gps"].value("accuracy", 0.0f);
        }

        // Compass
        if (j.contains("compass")) {
            out.compass.heading     = j["compass"].value("heading",     0.0f);
            out.compass.noise_sigma = j["compass"].value("noise_sigma", 0.0f);
        }

        // IMU
        if (j.contains("imu")) {
            out.imu.accel_x = j["imu"].value("accel_x", 0.0f);
            out.imu.accel_y = j["imu"].value("accel_y", 0.0f);
            out.imu.accel_z = j["imu"].value("accel_z", 0.0f);
            out.imu.gyro_x  = j["imu"].value("gyro_x",  0.0f);
            out.imu.gyro_y  = j["imu"].value("gyro_y",  0.0f);
            out.imu.gyro_z  = j["imu"].value("gyro_z",  0.0f);
        }

        // Velocity
        if (j.contains("velocity")) {
            out.velocity.speed_ms    = j["velocity"].value("speed_ms",    0.0f);
            out.velocity.speed_knots = j["velocity"].value("speed_knots", 0.0f);
            out.velocity.vx          = j["velocity"].value("vx",          0.0f);
            out.velocity.vz          = j["velocity"].value("vz",          0.0f);
        }

        // AIS
        if (j.contains("ais")) {
            out.ais.contactCount = j["ais"].value("contactCount", 0);
            out.ais.contacts.clear();

            if (j["ais"].contains("contacts")) {
                for (auto& c : j["ais"]["contacts"]) {
                    AIS_Contact contact;
                    contact.mmsi        = c.value("mmsi",        0);
                    contact.vesselName  = c.value("vesselName",  "");
                    contact.shipType    = c.value("shipType",    "");
                    contact.distance    = c.value("distance",    0.0f);
                    contact.bearing     = c.value("bearing",     0.0f);
                    contact.heading     = c.value("heading",     0.0f);
                    contact.speed_knots = c.value("speed_knots", 0.0f);
                    contact.lat         = c.value("lat",         0.0);
                    contact.lon         = c.value("lon",         0.0);
                    out.ais.contacts.push_back(contact);
                }
            }
        }


        if (j.contains("lora")) {
            out.lora.neighbourCount = j["lora"].value("neighbourCount", 0);
            out.lora.neighbours.clear();
            if (j["lora"].contains("neighbours")) {
                for (auto& n : j["lora"]["neighbours"]) {
                    LoRa_Neighbour nb;
                    nb.nodeId         = n.value("nodeId",         0);
                    nb.distanceMetres = n.value("distanceMetres", 0.0f);
                    nb.rssi           = n.value("rssi",           0.0f);
                    nb.snr            = n.value("snr",            0.0f);
                    nb.lastSeenAge    = n.value("lastSeenAge",    0.0f);
                    nb.lastPayload    = n.value("lastPayload",    "");
                    out.lora.neighbours.push_back(nb);
                }
        }
}

        return out.droneId >= 0;
    }
    catch (...) {
        return false;
    }
}

// ── Serialize outgoing command ───────────────────────────────────
inline std::string serializeCommand(const DroneCommandPacket& cmd)
{
    json j;
    j["droneId"]   = cmd.droneId;
    j["packetSeq"] = cmd.packetSeq;
    j["throttle"]  = cmd.throttle;
    j["steer"]     = cmd.steer;

    j["loraBroadcast"] = cmd.loraBroadcast;

    return j.dump();
}

// ================================================================
// ESP-IDF SWAP GUIDE
// Replace above functions with cJSON equivalents:
//
// parseStatePacket:
//   cJSON* root = cJSON_Parse(raw.c_str());
//   out.droneId = cJSON_GetObjectItem(root, "droneId")->valueint;
//   ... etc
//   cJSON_Delete(root);
//
// serializeCommand:
//   cJSON* root = cJSON_CreateObject();
//   cJSON_AddNumberToObject(root, "droneId", cmd.droneId);
//   ... etc
//   char* str = cJSON_Print(root);
//   std::string result(str);
//   cJSON_Delete(root); free(str);
//   return result;
// ================================================================
