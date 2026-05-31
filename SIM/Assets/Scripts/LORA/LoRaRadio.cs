using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

// ================================================================
// LoRaRadio — attach to each drone/buoy that has a radio
// Handles messaging, ranging, half-duplex state machine
// ================================================================
public class LoRaRadio : MonoBehaviour
{
    [Header("Identity")]
    public int nodeId = 0;

    [Header("Configuration")]
    public LoRaConfig config = new LoRaConfig();

    [Header("Ranging")]
    [Tooltip("How often to ping each known node for range")]
    public float rangingInterval  = 2f;
    [Tooltip("Ranging measurement noise in metres")]
    public float rangingNoiseSigma = 1.5f;

    [Header("Status (Read Only)")]
    public bool  isTranmitting   = false;
    public bool  isReceiving     = false;
    public float lastRssi        = 0f;
    public int   packetsReceived = 0;

    // ── Neighbour table ──────────────────────────────────────────
    [Serializable]
    public class NeighbourEntry
    {
        public int    nodeId;
        public int    mmsi;          // FIX (Design): expose MMSI separately so C++ consumers
                                     // are never confused between radio nodeId and vessel MMSI
        public float  rssi;
        public float  snr;
        public float  distanceMetres;   // From ranging
        public float  lastSeenTime;
        public string lastPayload;
    }

    [Header("Neighbours (Read Only)")]
    public List<NeighbourEntry> neighbours = new List<NeighbourEntry>();

    // ── Events ───────────────────────────────────────────────────
    public event Action<LoRaPacket>    OnMessageReceived;
    public event Action<RangingResult> OnRangingComplete;

    // ── Internal ─────────────────────────────────────────────────
    [HideInInspector] public float lastTxTime = -999f;

    private enum RadioState { Idle, Transmitting, WaitingRangingReply }
    private RadioState _state = RadioState.Idle;

    private Dictionary<int, float> _pendingRanging
        = new Dictionary<int, float>();   // nodeId → request send time

    // FIX (Design): track all radios we know exist (from medium) so
    // ranging can be proactive, not just reactive to first heard packet
    private AISTransponder _ais;

    // ================================================================
    void Start()
    {
        _ais = GetComponent<AISTransponder>();

        // Calculate derived config values
        config.maxRangeMetres = LoRaMedium.Instance != null
            ? LoRaMedium.Instance.CalculateMaxRange(config)
            : 500f;

        config.dataRateBps = (config.bandwidthKHz * 1000f
            * config.spreadingFactor)
            / Mathf.Pow(2f, config.spreadingFactor);

        LoRaMedium.Register(this);

        StartCoroutine(RangingLoop());
    }

    void OnDisable() => LoRaMedium.Unregister(this);

    // ================================================================
    // PUBLIC API
    // ================================================================

    // ── Broadcast a message to all nodes in range ────────────────
    public void Broadcast(string payload)
    {
        if (isTranmitting) return;   // Half-duplex: busy

        LoRaPacket packet = new LoRaPacket
        {
            senderId         = nodeId,
            senderMmsi       = _ais != null ? _ais.mmsi : 0,
            payload          = payload,
            txTime           = Time.time,
            isRangingRequest = false,
            isRangingReply   = false
        };

        TransmitPacket(packet);
    }

    // ── Send a ranging request to a specific node ────────────────
    public void RequestRanging(int targetNodeId)
    {
        if (isTranmitting) return;

        LoRaPacket packet = new LoRaPacket
        {
            senderId          = nodeId,
            senderMmsi        = _ais != null ? _ais.mmsi : 0,
            payload           = "",
            txTime            = Time.time,
            isRangingRequest  = true,
            isRangingReply    = false,
            rangingTargetId   = targetNodeId
        };

        _pendingRanging[targetNodeId] = Time.time;
        _state = RadioState.WaitingRangingReply;

        TransmitPacket(packet);
    }

    // ── Get last known distance to a node ────────────────────────
    public float GetDistance(int targetNodeId)
    {
        var neighbour = neighbours.Find(n => n.nodeId == targetNodeId);
        return neighbour != null ? neighbour.distanceMetres : -1f;
    }

    // ================================================================
    // RECEIVE — called by LoRaMedium
    // ================================================================
    public void OnPacketReceived(LoRaPacket packet, LoRaRadio sender)
    {
        lastRssi = packet.rssi;
        packetsReceived++;

        // Update neighbour table
        UpdateNeighbour(packet, sender);

        // ── Ranging reply ────────────────────────────────────────
        if (packet.isRangingReply && packet.rangingTargetId == nodeId)
        {
            ProcessRangingReply(packet, sender);
            return;
        }

        // ── Ranging request directed at us ───────────────────────
        if (packet.isRangingRequest && packet.rangingTargetId == nodeId)
        {
            SendRangingReply(packet.senderId);
            return;
        }

        // ── Normal message ───────────────────────────────────────
        if (!packet.isRangingRequest && !packet.isRangingReply)
            OnMessageReceived?.Invoke(packet);
    }

    // ================================================================
    // INTERNAL
    // ================================================================
    private void TransmitPacket(LoRaPacket packet)
    {
        isTranmitting = true;
        lastTxTime    = Time.time;

        LoRaMedium.Instance?.Transmit(packet, this);

        // FIX (Logic): ask the medium for the real air-time so the half-duplex
        // lock is held for exactly as long as the packet is in the air.
        // Previously a hardcoded 0.05 / 0.01 s was used, which could release
        // the transmitter before the medium had finished delivering the packet,
        // allowing back-to-back transmissions the collision model would miss.
        float airTime = LoRaMedium.Instance != null
            ? LoRaMedium.Instance.CalculateAirTime(packet.payload.Length, config)
            : (packet.payload.Length > 0 ? 0.05f : 0.01f);

        StartCoroutine(ReleaseTransmitter(airTime));
    }

    private IEnumerator ReleaseTransmitter(float delay)
    {
        yield return new WaitForSeconds(delay);
        isTranmitting = false;
        if (_state == RadioState.Transmitting)
            _state = RadioState.Idle;
    }

    private void SendRangingReply(int requesterId)
    {
        LoRaPacket reply = new LoRaPacket
        {
            senderId         = nodeId,
            senderMmsi       = _ais != null ? _ais.mmsi : 0,
            payload          = "",
            txTime           = Time.time,
            isRangingRequest = false,
            isRangingReply   = true,
            rangingTargetId  = requesterId
        };
        TransmitPacket(reply);
    }

    private void ProcessRangingReply(LoRaPacket reply, LoRaRadio sender)
    {
        if (!_pendingRanging.TryGetValue(reply.senderId, out float requestTime))
            return;

        _pendingRanging.Remove(reply.senderId);
        _state = RadioState.Idle;

        // True distance from simulation
        float trueDistance = Vector3.Distance(
            transform.position, sender.transform.position);

        // Add measurement noise (SX1280 ranging accuracy ~2m at 1km)
        float noise        = GaussianNoise(0f, rangingNoiseSigma);
        float measuredDist = Mathf.Max(0f, trueDistance + noise);

        // Update neighbour
        var neighbour = GetOrCreateNeighbour(reply.senderId);
        neighbour.distanceMetres = measuredDist;

        RangingResult result = new RangingResult
        {
            targetId       = reply.senderId,
            distanceMetres = measuredDist,
            errorMetres    = noise,
            success        = true
        };

        OnRangingComplete?.Invoke(result);
    }

    private void UpdateNeighbour(LoRaPacket packet, LoRaRadio sender)
    {
        var entry          = GetOrCreateNeighbour(packet.senderId);
        entry.mmsi         = packet.senderMmsi;   // FIX (Design): store MMSI in neighbour entry
        entry.rssi         = packet.rssi;
        entry.snr          = packet.snr;
        entry.lastSeenTime = Time.time;
        entry.lastPayload  = packet.payload;
    }

    private NeighbourEntry GetOrCreateNeighbour(int id)
    {
        var entry = neighbours.Find(n => n.nodeId == id);
        if (entry == null)
        {
            entry = new NeighbourEntry { nodeId = id };
            neighbours.Add(entry);
        }
        return entry;
    }

    // ── Periodic ranging loop ────────────────────────────────────
    // FIX (Logic): previously only ranged nodes already in the neighbour
    // table, meaning a drone with no prior contact would never measure
    // distance to anyone. Now we also range all OTHER registered radios
    // from the medium, so ranging is proactive from the start.
    private IEnumerator RangingLoop()
    {
        // Stagger start so drones don't all range simultaneously
        yield return new WaitForSeconds(UnityEngine.Random.Range(0f, rangingInterval));

        while (true)
        {
            // Build a snapshot of candidate node IDs to range:
            // - all registered radios from the medium (proactive discovery)
            // - plus any in the neighbour table not already covered
            var candidates = new HashSet<int>();

            if (LoRaMedium.Instance != null)
                foreach (var radio in LoRaMedium.Instance.GetRegisteredRadios())
                    if (radio != this)
                        candidates.Add(radio.nodeId);

            foreach (var neighbour in neighbours)
                candidates.Add(neighbour.nodeId);

            foreach (int targetId in candidates)
            {
                if (!isTranmitting)
                    RequestRanging(targetId);

                yield return new WaitForSeconds(0.1f);  // Gap between requests
            }

            yield return new WaitForSeconds(rangingInterval);
        }
    }

    private float GaussianNoise(float mean, float sigma)
    {
        float u1 = 1f - UnityEngine.Random.value;
        float u2 = 1f - UnityEngine.Random.value;
        return mean + sigma * Mathf.Sqrt(-2f * Mathf.Log(u1))
                    * Mathf.Sin(2f * Mathf.PI * u2);
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (config.maxRangeMetres < 1f) return;
        UnityEditor.Handles.color = new Color(0.2f, 0.8f, 0.2f, 0.08f);
        UnityEditor.Handles.DrawSolidDisc(
            transform.position, Vector3.up, config.maxRangeMetres);
        UnityEditor.Handles.color = new Color(0.2f, 0.8f, 0.2f, 0.5f);
        UnityEditor.Handles.DrawWireDisc(
            transform.position, Vector3.up, config.maxRangeMetres);
    }
#endif
}
