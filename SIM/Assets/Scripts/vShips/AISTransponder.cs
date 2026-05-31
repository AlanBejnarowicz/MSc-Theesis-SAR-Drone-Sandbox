using Unity.Mathematics;
using UnityEngine;

public enum AisShipType : byte
    {
        Unknown        = 0,
        Fishing        = 30,
        Sailing        = 36,
        PleasureCraft  = 37,
        HighSpeedCraft = 40,
        PilotVessel    = 50,
        SearchAndRescue= 51,
        Tug            = 52,
        Passenger      = 60,
        Cargo          = 70,
        Tanker         = 80,
        AtoN           = 99
    }


public class AISTransponder : MonoBehaviour
{




    [Header("Vessel Info")]
    public string vesselName = "Ship_01";
    public int    mmsi       = 123456789;   // Unique vessel ID



    public AisShipType shipType   = AisShipType.Unknown;  // <- dropdown in Inspector




    [Header("Broadcast")]
    public float broadcastInterval = 2f;    // Real AIS: 2-10s depending on speed

    public bool initWithRandomMMSI = false;

    // Current AIS data — read by anyone who wants it
    [HideInInspector] public Vector3  position;
    public float    heading;
    public float    speedKnots;

    public float ship_lenght = 5.0f;
    [HideInInspector] public bool     isActive = true;

    [HideInInspector] public double latitude;
    [HideInInspector] public double longitude;

    private vGPS _gps;

    private Rigidbody _rb;
    private float     _timer;

    void Start()
    {
        _rb  = GetComponent<Rigidbody>();
        _gps = GetComponent<vGPS>();         // Must be on the same GameObject


        if (initWithRandomMMSI)
        {
            mmsi = UnityEngine.Random.Range(100000000, 999999999);  
        }

        DroneShipData data = this.GetComponent<DroneShipData>();
        if(data != null)
        {
            data.drone_MMSI = mmsi;
        }

        AISManager.Register(this);
        Broadcast();
    }

    void OnDisable() => AISManager.Unregister(this);

    void Update()
    {
        _timer += Time.deltaTime;
        if (_timer >= broadcastInterval)
        {
            _timer = 0f;
            Broadcast();
        }
    }

    private void Broadcast()
    {
        position  = transform.position;     // Keep for range checks (Unity units)
        heading   = transform.eulerAngles.y;
        speedKnots = _rb != null ? _rb.velocity.magnitude * 1.94384f : 0f;
        

        // Real-world coordinates from GPS
        if (_gps != null)
        {
            latitude  = _gps.currentLat;
            longitude = _gps.currentLon;
        }

        AISManager.OnBroadcast(this);
    }


}