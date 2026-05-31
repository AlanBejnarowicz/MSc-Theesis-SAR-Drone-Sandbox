using UnityEngine;
using System.Collections.Generic;

// ================================================================
// LoRaMedium — singleton that simulates the RF medium
// All radios register here. Handles propagation, RSSI, packet loss.
// Attach to any GameObject in scene.
// ================================================================
public class LoRaMedium : MonoBehaviour
{
    public static LoRaMedium Instance { get; private set; }

    [Header("Environment")]
    [Tooltip("Free-space path loss exponent. 2.0 = ideal, 2.5-3.5 = urban/maritime")]
    public float pathLossExponent = 2.5f;

    [Tooltip("Receiver sensitivity in dBm. SX1280 typical = -100 to -105 dBm")]
    public float receiverSensitivitydBm = -100f;

    [Tooltip("RSSI noise standard deviation in dB")]
    public float rssiNoiseSigma = 3f;

    [Header("Collision Model")]
    [Tooltip("Probability two simultaneous transmissions collide")]
    [Range(0f, 1f)]
    public float collisionProbability = 0.3f;

    [Header("Debug")]
    public bool  logTransmissions = false;
    public int   totalPacketsSent = 0;
    public int   totalPacketsLost = 0;

    // All registered radios
    private List<LoRaRadio> _radios = new List<LoRaRadio>();

    // Packets currently in the air (mid-transmission)
    private List<(LoRaPacket packet, LoRaRadio sender, float arrivalTime)> _inFlight
        = new List<(LoRaPacket, LoRaRadio, float)>();

    void Awake() => Instance = this;

    void Update()
    {
        // Deliver packets whose air-time has elapsed
        float now = Time.time;
        for (int i = _inFlight.Count - 1; i >= 0; i--)
        {
            var (packet, sender, arrivalTime) = _inFlight[i];
            if (now >= arrivalTime)
            {
                DeliverPacket(packet, sender);
                _inFlight.RemoveAt(i);
            }
        }
    }

    public static void Register(LoRaRadio radio)
    {
        if (Instance != null && !Instance._radios.Contains(radio))
            Instance._radios.Add(radio);
    }

    public static void Unregister(LoRaRadio radio)
    {
        Instance?._radios.Remove(radio);
    }

    // ── Called by LoRaRadio when it transmits ────────────────────
    public void Transmit(LoRaPacket packet, LoRaRadio sender)
    {
        totalPacketsSent++;

        // Air time = time for packet to propagate (simplified)
        float airTime = CalculateAirTime(packet.payload.Length, sender.config);

        _inFlight.Add((packet, sender, Time.time + airTime));

        if (logTransmissions)
            Debug.Log($"[LoRa] TX from {sender.nodeId} | " +
                      $"payload={packet.payload.Length}B | " +
                      $"airTime={airTime*1000:F1}ms");
    }

    // ── Deliver to all radios in range ───────────────────────────
    private void DeliverPacket(LoRaPacket packet, LoRaRadio sender)
    {
        // Check for simultaneous transmissions (collision)
        bool collision = CheckCollision(sender, packet.txTime);
        if (collision)
        {
            totalPacketsLost++;
            if (logTransmissions)
                Debug.Log($"[LoRa] COLLISION — packet from {sender.nodeId} lost");
            return;
        }

        foreach (var radio in _radios)
        {
            if (radio == sender) continue;
            if (!radio.isActiveAndEnabled) continue;

            float dist  = Vector3.Distance(sender.transform.position,
                                           radio.transform.position);
            float rssi  = CalculateRSSI(dist, sender.config);
            float snr   = rssi - receiverSensitivitydBm + GaussianNoise(0, 4f);

            // Below sensitivity = packet lost
            if (rssi < receiverSensitivitydBm)
            {
                totalPacketsLost++;
                continue;
            }

            // Deliver with measured RSSI
            LoRaPacket received  = packet;
            received.rssi        = rssi + GaussianNoise(0, rssiNoiseSigma);
            received.snr         = snr;

            radio.OnPacketReceived(received, sender);
        }
    }

    // ── RSSI model: Friis free-space path loss ───────────────────
    // RSSI(d) = TxPower - 20*log10(4πd/λ) * pathLossExp correction
    public float CalculateRSSI(float distanceMetres, LoRaConfig config)
    {
        if (distanceMetres < 0.1f) return config.txPowerdBm;

        float wavelength    = 0.125f;   // 2.4GHz: λ = c/f ≈ 0.125m
        float freeSpaceLoss = 20f * Mathf.Log10(4f * Mathf.PI * distanceMetres / wavelength);
        float pathLoss      = freeSpaceLoss * (pathLossExponent / 2f);

        return config.txPowerdBm - pathLoss;
    }

    // ── Max range from config ────────────────────────────────────
    public float CalculateMaxRange(LoRaConfig config)
    {
        // Invert RSSI formula to find distance at sensitivity threshold
        float wavelength = 0.125f;
        float maxLoss    = config.txPowerdBm - receiverSensitivitydBm;
        float adjLoss    = maxLoss * (2f / pathLossExponent);
        float range      = (wavelength / (4f * Mathf.PI))
                         * Mathf.Pow(10f, adjLoss / 20f);
        return range;
    }

    // ── Expose registered radio list for proactive ranging ───────
    public IReadOnlyList<LoRaRadio> GetRegisteredRadios() => _radios;

    // ── Packet air time (LoRa ToA approximation) ─────────────────
    // FIX: made public so LoRaRadio can use the real air-time to hold
    // the half-duplex lock for the correct duration (was private before)
    public float CalculateAirTime(int payloadBytes, LoRaConfig config)
    {
        // Simplified LoRa ToA formula
        float symbolRate = config.bandwidthKHz * 1000f
                         / Mathf.Pow(2f, config.spreadingFactor);
        float symbolTime = 1f / symbolRate;

        // Payload symbols: header(20) + data + CRC
        float payloadSymbols = 20f + payloadBytes * 8f
                             / config.spreadingFactor;
        return payloadSymbols * symbolTime;
    }

    // ── Collision check ──────────────────────────────────────────
    private bool CheckCollision(LoRaRadio sender, float txTime)
    {
        // Count other transmissions within the same window
        float window = 0.01f;   // 10ms collision window
        int   count  = 0;
        foreach (var (_, otherSender, _) in _inFlight)
            if (otherSender != sender &&
                Mathf.Abs(otherSender.lastTxTime - txTime) < window)
                count++;

        return count > 0 && Random.value < collisionProbability;
    }

    private float GaussianNoise(float mean, float sigma)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        return mean + sigma * Mathf.Sqrt(-2f * Mathf.Log(u1))
                    * Mathf.Sin(2f * Mathf.PI * u2);
    }
}
