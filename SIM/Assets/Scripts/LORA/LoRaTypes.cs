using UnityEngine;
using System;

// ================================================================
// LoRa SX1280 2.4GHz Simulation
// Models: messaging, ranging, RSSI, packet loss, half-duplex
// ================================================================

// ── Radio configuration ──────────────────────────────────────────
[Serializable]
public class LoRaConfig
{
    [Header("RF Parameters")]
    [Tooltip("Spreading Factor 5-12. Higher = longer range, slower data rate")]
    [Range(5, 12)]
    public int   spreadingFactor  = 7;

    [Tooltip("Bandwidth in kHz: 203, 406, 812, 1625")]
    public float bandwidthKHz     = 406f;

    [Tooltip("Coding Rate 1=4/5 ... 4=4/8")]
    [Range(1, 4)]
    public int   codingRate       = 1;

    [Tooltip("Transmit power in dBm. SX1280 max = +13dBm")]
    [Range(-18, 13)]
    public float txPowerdBm       = 10f;

    [Header("Derived (auto-calculated)")]
    public float maxRangeMetres   = 0f;   // Set by LoRaRadio.Init()
    public float dataRateBps      = 0f;
}

// ── A single message in the air ──────────────────────────────────
[Serializable]
public class LoRaPacket
{
    public int    senderId;
    public int    senderMmsi;
    public string payload;          // JSON string — arbitrary data
    public float  txTime;           // Time.time when sent
    public float  rssi;             // Filled in by receiver
    public float  snr;              // Signal-to-noise ratio (dB)
    public bool   isRangingRequest; // True = ranging ping
    public bool   isRangingReply;
    public int    rangingTargetId;  // For directed ranging

}

// ── Result of a ranging exchange ────────────────────────────────
public struct RangingResult
{
    public int   targetId;
    public float distanceMetres;
    public float errorMetres;       // Simulated measurement error
    public bool  success;
}
