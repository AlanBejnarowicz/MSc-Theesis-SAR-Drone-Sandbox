using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;

// ================================================================
// PACKET DEFINITIONS
// ================================================================

[Serializable]
public class GPS_Block
{
    public double lat;
    public double lon;
    public float  accuracy;
}

[Serializable]
public class Compass_Block
{
    public float heading;
    public float noise_sigma;
}

[Serializable]
public class IMU_Block
{
    public float accel_x, accel_y, accel_z;
    public float gyro_x,  gyro_y,  gyro_z;
}

[Serializable]
public class Velocity_Block
{
    public float speed_ms;
    public float speed_knots;
    public float vx, vz;
}

[Serializable]
public class AIS_Contact
{
    public int    mmsi;
    public string vesselName;
    public string shipType;
    public float  distance;
    public float  bearing;
    public float  heading;
    public float  speed_knots;
    public double lat;
    public double lon;
}

[Serializable]
public class AIS_Block
{
    public int               contactCount;
    public List<AIS_Contact> contacts = new List<AIS_Contact>();
}

// ── LoRa — raw payload, no interpretation ────────────────────────
[Serializable]
public class LoRa_Neighbour
{
    public int    nodeId;
    public int    mmsi;             // FIX (Design): vessel MMSI — distinct from radio nodeId.
                                    // C++ must use nodeId for LoRa addressing and mmsi for
                                    // AIS cross-referencing. Never treat them as interchangeable.
    public float  distanceMetres;   // From ranging
    public float  rssi;
    public float  snr;
    public float  lastSeenAge;      // Seconds since last heard
    public string lastPayload;      // Raw base64 — C++ defines the format
}

[Serializable]
public class LoRa_Block
{
    public int                  neighbourCount;
    public List<LoRa_Neighbour> neighbours = new List<LoRa_Neighbour>();
}

// ── Outgoing state packet ─────────────────────────────────────────
[Serializable]
public class DroneStatePacket
{
    public int    droneId;
    public int    mmsi;
    public float  timestamp;
    public int    packetSeq;
    public float  pos_x;
    public float  pos_z;

    public GPS_Block      gps      = new GPS_Block();
    public Compass_Block  compass  = new Compass_Block();
    public IMU_Block      imu      = new IMU_Block();
    public Velocity_Block velocity = new Velocity_Block();
    public AIS_Block      ais      = new AIS_Block();
    public LoRa_Block     lora     = new LoRa_Block();

    // Future sensors:
    // public Radar_Block  radar = new Radar_Block();
    // public Wind_Block   wind  = new Wind_Block();
    // public Depth_Block  depth = new Depth_Block();
}

// ── Incoming command packet ───────────────────────────────────────
[Serializable]
public class DroneCommandPacket
{
    public int    droneId;
    public int    packetSeq;
    public float  throttle;
    public float  steer;
    public string loraBroadcast = "";    // Base64 encoded bytes — empty = no broadcast
                                    // C++ defines payload format freely
}

// ================================================================
// PER-DRONE UDP BRIDGE
// ================================================================
[RequireComponent(typeof(AdvancedShipController))]
[RequireComponent(typeof(DroneShipData))]
public class DroneUDPBridge : MonoBehaviour
{
    [Header("Network — Send (Unity → C++)")]
    public string sendHost    = "127.0.0.1";
    public int    sendPort    = 5000;

    [Header("Network — Receive (C++ → Unity)")]
    public int    receivePort = 6000;

    [Header("Timing")]
    public float  sendRate       = 20f;
    public float  commandTimeout = 0.5f;

    [Header("AIS")]
    public int    maxAisContacts  = 10;

    [Header("LoRa")]
    public int    maxLoraNeighbours = 10;

    [Header("Debug")]
    public bool               logPackets = false;
    public DroneStatePacket   lastSentState;
    public DroneCommandPacket lastReceivedCommand;

    // ── Components ──────────────────────────────────────────────
    private AdvancedShipController _driver;
    private DroneShipData          _data;
    private vGPS                   _gps;
    private vCOMPASS               _compass;
    private vIMU                   _imu;
    private AISReceiver            _aisReceiver;
    private LoRaRadio              _loraRadio;
    private Rigidbody              _rb;

    // ── LoRa raw message cache ───────────────────────────────────
    // Key: senderId, Value: last raw packet — C++ defines content
    private Dictionary<int, LoRaPacket> _lastLoRaMessages
        = new Dictionary<int, LoRaPacket>();

    // ── Network ─────────────────────────────────────────────────
    private UdpClient  _sender;
    private UdpClient  _receiver;
    private IPEndPoint _sendEndpoint;
    private Thread     _receiveThread;
    private bool       _running = false;

    private ConcurrentQueue<DroneCommandPacket> _commandQueue
        = new ConcurrentQueue<DroneCommandPacket>();

    private float _lastCommandTime = -999f;
    private float _sendTimer       = 0f;
    private int   _packetSeq       = 0;

    private int _actualSendPort;
    private int _actualReceivePort;

    // ================================================================
    void Start()
    {
        _driver      = GetComponent<AdvancedShipController>();
        _data        = GetComponent<DroneShipData>();
        _gps         = GetComponent<vGPS>();
        _compass     = GetComponent<vCOMPASS>();
        _imu         = GetComponent<vIMU>();
        _aisReceiver = GetComponent<AISReceiver>();
        _loraRadio   = GetComponent<LoRaRadio>();
        _rb          = GetComponent<Rigidbody>();

        // Subscribe to LoRa messages — store raw payload, no interpretation
        if (_loraRadio != null)
            _loraRadio.OnMessageReceived += OnLoRaMessageReceived;

        _actualSendPort    = sendPort;
        _actualReceivePort = receivePort;

        InitNetwork();
    }

    void Update()
    {
        ApplyPendingCommands();

        _sendTimer += Time.deltaTime;
        if (_sendTimer >= 1f / sendRate)
        {
            SendState();
            _sendTimer = 0f;
        }

        if (Time.time - _lastCommandTime > commandTimeout)
        {
            _driver.throttle   = 0f;
            _driver.steerInput = 0f;
        }
    }

    void OnDestroy()         => Shutdown();
    void OnApplicationQuit() => Shutdown();
    void OnDisable()         => Shutdown();


    // ================================================================
    // LoRa receive — cache raw packet, no parsing
    // ================================================================
    private void OnLoRaMessageReceived(LoRaPacket packet)
    {
        _lastLoRaMessages[packet.senderId] = packet;
    }

    // ================================================================
    // Build + send state
    // ================================================================
    private void SendState()
    {
        if (_sender == null) return;

        try
        {
            DroneStatePacket packet = BuildStatePacket();
            lastSentState = packet;

            string json  = JsonUtility.ToJson(packet);
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            _sender.Send(bytes, bytes.Length, _sendEndpoint);

            if (logPackets)
                Debug.Log($"[TX {_data.ShipId}] seq={packet.packetSeq} " +
                          $"ais={packet.ais.contactCount} " +
                          $"lora={packet.lora.neighbourCount}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[DroneUDPBridge] Send error: {e.Message}");
        }
    }

    private DroneStatePacket BuildStatePacket()
    {
        DroneStatePacket p = new DroneStatePacket
        {
            droneId   = _data.ShipId,
            mmsi      = _data.drone_MMSI,
            timestamp = Time.time,
            packetSeq = _packetSeq++,
            pos_x     = transform.position.x,
            pos_z     = transform.position.z
        };

        // GPS
        if (_gps != null)
        {
            p.gps.lat      = _gps.currentLat;
            p.gps.lon      = _gps.currentLon;
            p.gps.accuracy = _gps.positionalNoiseSigma;
        }

        // Compass
        if (_compass != null)
        {
            p.compass.heading     = _compass.currentHeading;
            p.compass.noise_sigma = _compass.headingNoiseSigma;
        }

        // IMU
        if (_imu != null)
        {
            p.imu.accel_x = _imu.AccelReadings.x;
            p.imu.accel_y = _imu.AccelReadings.y;
            p.imu.accel_z = _imu.AccelReadings.z;
            p.imu.gyro_x  = _imu.GyroReadings.x;
            p.imu.gyro_y  = _imu.GyroReadings.y;
            p.imu.gyro_z  = _imu.GyroReadings.z;
        }

        // Velocity
        if (_rb != null)
        {
            Vector3 vel            = _rb.velocity;
            float   spd            = new Vector3(vel.x, 0, vel.z).magnitude;
            p.velocity.speed_ms    = spd;
            p.velocity.speed_knots = spd * 1.94384f;
            p.velocity.vx          = vel.x;
            p.velocity.vz          = vel.z;
        }

        // AIS
        if (_aisReceiver != null)
        {
            var contacts       = _aisReceiver.knownVessels;
            p.ais.contactCount = Mathf.Min(contacts.Count, maxAisContacts);

            foreach (var kvp in contacts)
            {
                if (p.ais.contacts.Count >= maxAisContacts) break;

                AISTransponder t = kvp.Value;
                p.ais.contacts.Add(new AIS_Contact
                {
                    mmsi        = t.mmsi,
                    vesselName  = t.vesselName,
                    shipType    = t.shipType.ToString(),
                    distance    = Vector3.Distance(transform.position, t.position),
                    bearing     = RelativeBearing(t.position),
                    heading     = t.heading,
                    speed_knots = t.speedKnots,
                    lat         = t.latitude,
                    lon         = t.longitude
                });
            }
        }

        // LoRa — raw payloads from neighbours, no interpretation
        BuildLoRaBlock(p);

        return p;
    }

    private void BuildLoRaBlock(DroneStatePacket p)
    {
        if (_loraRadio == null) return;

        foreach (var n in _loraRadio.neighbours)
        {
            if (p.lora.neighbours.Count >= maxLoraNeighbours) break;

            // Fetch last raw payload from this neighbour — may be empty
            string rawPayload = "";
            if (_lastLoRaMessages.TryGetValue(n.nodeId, out LoRaPacket msg))
                rawPayload = msg.payload;

            p.lora.neighbours.Add(new LoRa_Neighbour
            {
                nodeId         = n.nodeId,
                mmsi           = n.mmsi,           // FIX (Design): propagate MMSI to C++
                distanceMetres = n.distanceMetres,
                rssi           = n.rssi,
                snr            = n.snr,
                lastSeenAge    = Time.time - n.lastSeenTime,
                lastPayload    = rawPayload   // Base64 from C++ — untouched
            });
        }

        p.lora.neighbourCount = p.lora.neighbours.Count;
    }

    private float RelativeBearing(Vector3 targetPos)
    {
        Vector3 dir        = targetPos - transform.position;
        dir.y              = 0f;
        float worldBearing = Mathf.Atan2(dir.x, dir.z) * Mathf.Rad2Deg;
        float ownHeading   = _compass != null
            ? _compass.currentHeading
            : transform.eulerAngles.y;
        return Mathf.DeltaAngle(ownHeading, worldBearing);
    }

    // ================================================================
    // Receive loop (background thread)
    // ================================================================
    private void ReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);

        while (_running)
        {
            try
            {
                byte[] data = _receiver.Receive(ref remote);
                string json = Encoding.UTF8.GetString(data);

                DroneCommandPacket cmd =
                    JsonUtility.FromJson<DroneCommandPacket>(json);

                if (cmd != null && cmd.droneId == _data.ShipId)
                    _commandQueue.Enqueue(cmd);
            }
            catch (SocketException) { }
            catch (Exception e)
            {
                if (_running)
                    Debug.LogWarning($"[DroneUDPBridge] RX error: {e.Message}");
            }
        }
    }

    // ================================================================
    // Apply commands on main thread
    // ================================================================
    private void ApplyPendingCommands()
    {
        while (_commandQueue.TryDequeue(out DroneCommandPacket cmd))
        {
            _driver.throttle    = Mathf.Clamp(cmd.throttle, -1f, 1f);
            _driver.steerInput  = Mathf.Clamp(cmd.steer,    -1f, 1f);
            _lastCommandTime    = Time.time;
            lastReceivedCommand = cmd;

            // Forward LoRa broadcast if provided — Unity never looks inside
            if (_loraRadio != null && !string.IsNullOrEmpty(cmd.loraBroadcast))
                _loraRadio.Broadcast(cmd.loraBroadcast);
        }
    }

    // ================================================================
    // Network init / teardown
    // ================================================================
    private void InitNetwork()
    {
        try
        {
            _sender       = new UdpClient();
            _sendEndpoint = new IPEndPoint(
                IPAddress.Parse(sendHost), _actualSendPort);

            _receiver = new UdpClient(_actualReceivePort);
            _receiver.Client.ReceiveTimeout = 10;
            _receiver.Client.SetSocketOption(
                SocketOptionLevel.Socket,
                SocketOptionName.ReuseAddress, true);

            int bufSize = 2 * 1024 * 1024;
            _receiver.Client.ReceiveBufferSize = bufSize;

            _running       = true;
            _receiveThread = new Thread(ReceiveLoop) { IsBackground = true };
            _receiveThread.Start();

            Debug.Log($"[DroneUDPBridge] Drone {_data.ShipId} " +
                      $"TX→{sendHost}:{_actualSendPort} " +
                      $"RX←:{_actualReceivePort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DroneUDPBridge] Init failed: {e.Message}");
        }
    }

    private void Shutdown()
    {
        if (!_running) return;   // Already shut down
        _running = false;
        _receiveThread?.Join(200);
        _sender?.Close();
        _sender = null;
        _receiver?.Close();
        _receiver = null;
        Debug.Log($"[DroneUDPBridge] Drone {_data?.ShipId} shutdown");
    }


}