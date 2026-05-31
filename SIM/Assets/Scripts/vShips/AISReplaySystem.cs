using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

// =============================================================================
//  AISReplaySystem
//  – Parses a CSV of AIS records (timestamp, mmsi, name, ship_type,
//    lat, lon, speed_kn, course_deg)
//  – Detects & optionally removes GPS-spoofed vessels (>=20 km jumps)
//  – Spawns a prefab per vessel and drives it along a Catmull-Rom spline
//  – Geo->Unity uses the same math as vGPS (GPS_CORD_ORIGIN pivot,
//    111111 m/deg lat, 111111*cos(lat) m/deg lon)
//  – Each vessel carries a live AISTransponder that broadcasts correctly
// =============================================================================
public class AISReplaySystem : MonoBehaviour
{
    // ─────────────────────────────────────────────────────────────────────────
    //  Inspector fields
    // ─────────────────────────────────────────────────────────────────────────

    [Header("─── Data Source ───────────────────────────────")]
    [Tooltip("Path relative to StreamingAssets, or an absolute path.")]
    public string csvFilePath = "AIS/ais_1h_20260428_130901.csv";

    [Header("─── Time Offset ────────────────────────────────")]
    [Tooltip("Start replay from a specific moment instead of the first CSV record.")]
    public bool   useCustomStartTime = false;
    [Tooltip("ISO-8601 date/time, e.g.  2026-04-28T13:30:00")]
    public string customStartTimeISO = "2026-04-28T13:00:00";

    [Header("─── Spoof Filter ───────────────────────────────")]
    [Tooltip("Exclude vessels whose consecutive AIS positions jump by more than the threshold.")]
    public bool  filterSpoofedVessels      = true;
    [Tooltip("Distance in km that flags a jump as GPS spoofing.")]
    public float spoofDetectionThresholdKm = 20f;

    [Header("─── Replay Speed ───────────────────────────────")]
    [Tooltip("1 = real-time.  2 = 2x speed, 0.5 = half speed, etc.")]
    [Range(0.01f, 200f)]
    public float replaySpeed = 1f;

    [Header("─── World Reference (must match vGPS) ──────────")]
    [Tooltip("The GPS_CORD_ORIGIN object used by your vGPS script. " +
             "Leave null to find it automatically by name.")]
    public Transform gpsOriginTransform;
    [Tooltip("Origin latitude  – must match vGPS.originLatitude.")]
    public double originLatitude  = 54.3718;
    [Tooltip("Origin longitude – must match vGPS.originLongitude.")]
    public double originLongitude = 18.6127;

    [Header("─── Vessel Prefab ──────────────────────────────")]
    [Tooltip("Ship prefab (scale 1 unit = 1 m). " +
             "Should contain AISTransponder. " +
             "If null, plain GameObjects are created.")]
    public GameObject vesselPrefab;

    [Tooltip("Length of the ship model in metres. Used to scale the spawned prefab correctly. " +
             "E.g. 100 means the prefab will be scaled to 100 m long.")]
    public float defaultShipLengthM = 50f;

    [Tooltip("Name of the child GameObject inside the prefab that holds the 3D mesh. " +
             "Leave blank if the mesh is on the root.")]
    public string modelChildName = "Model";

    [Header("─── Simulation Time (read-only) ────────────────")]
    [Tooltip("Current simulated UTC time in the AIS dataset.")]
    [SerializeField] private string _simTimeDisplay = "--";
    [Tooltip("Seconds elapsed in simulation since replay start.")]
    [SerializeField] private double _simElapsedSec  = 0;
    [Tooltip("Number of active vessels being simulated.")]
    [SerializeField] private int    _activeVessels  = 0;

    // ─────────────────────────────────────────────────────────────────────────
    //  Internal types
    // ─────────────────────────────────────────────────────────────────────────

    private struct AISRecord
    {
        public DateTime Timestamp;
        public int      MMSI;
        public string   Name;
        public int      ShipType;
        public double   Lat;
        public double   Lon;
        public float    SpeedKn;
        public float    CourseDeg;
    }

    private class VesselPlayback
    {
        public int             MMSI;
        public string          Name;
        public AisShipType     ShipType;
        public List<AISRecord> Records   = new List<AISRecord>();
        // SegIdx = index of p1 (departure waypoint of the current segment)
        public int             SegIdx    = 0;
        public GameObject      Go;
        public AISTransponder  Transponder;
        public ReplayAISGPS    GPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  State
    // ─────────────────────────────────────────────────────────────────────────

    private readonly List<VesselPlayback>           _vessels = new List<VesselPlayback>();
    private readonly Dictionary<int,VesselPlayback> _byMMSI  = new Dictionary<int,VesselPlayback>();

    private DateTime _dataStart;        // earliest UTC timestamp in the dataset
    private double   _startOffsetSec;   // seconds into the data we begin from

    // Accumulated simulation time — driven by Time.deltaTime so it respects
    // Time.timeScale and avoids the DateTime.UtcNow drift / leap-second issues
    // that caused static ships in the previous version.
    private double _accumSimSec = 0;
    private bool   _playing     = false;

    private double    _metersPerDegLat;
    private double    _metersPerDegLon;
    private Transform _vesselContainer;   // auto-created child of this GO

    // ─────────────────────────────────────────────────────────────────────────
    //  Unity lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    void Start()
    {
        ResolveOriginTransform();
        ComputeGeoScale();

        // Create a dedicated child to hold all spawned AIS vessels
        var containerGO = new GameObject("AIS_Vessels");
        containerGO.transform.SetParent(this.transform, false);
        _vesselContainer = containerGO.transform;

        LoadAndStart();
    }

    void Update()
    {
        if (!_playing) return;

        // Accumulate simulation time using deltaTime * speed.
        // This is the correct Unity pattern — avoids wall-clock drift and
        // respects Time.timeScale (pause, slow-mo, etc.).
        _accumSimSec += Time.deltaTime * replaySpeed;

        DateTime simNow = _dataStart.AddSeconds(_startOffsetSec + _accumSimSec);

        // Update read-only inspector displays
        _simTimeDisplay = simNow.ToString("yyyy-MM-dd  HH:mm:ss  UTC");
        _simElapsedSec  = _accumSimSec;

        foreach (var vp in _vessels)
            DriveVessel(vp, simNow);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Public API
    // ─────────────────────────────────────────────────────────────────────────

    public void LoadAndStart()
    {
        StopReplay();

        string path = ResolvePath(csvFilePath);
        if (!File.Exists(path))
        {
            Debug.LogError($"[AISReplay] CSV not found: {path}");
            return;
        }

        var all = ParseCSV(path);
        if (all.Count == 0)
        {
            Debug.LogWarning("[AISReplay] CSV parsed but contains no valid records.");
            return;
        }

        // Group by MMSI, sort chronologically, spoof-filter
        foreach (var grp in all.GroupBy(r => r.MMSI))
        {
            var recs = grp.OrderBy(r => r.Timestamp).ToList();

            if (filterSpoofedVessels && HasSpoofedJump(recs))
            {
                Debug.Log($"[AISReplay] SPOOFED -> excluded   MMSI {grp.Key}  \"{recs[0].Name}\"");
                continue;
            }

            var vp = new VesselPlayback
            {
                MMSI     = grp.Key,
                Name     = recs[0].Name,
                ShipType = (AisShipType)Mathf.Clamp(recs[0].ShipType, 0, 99),
                Records  = recs,
            };
            _vessels.Add(vp);
            _byMMSI[grp.Key] = vp;
        }

        Debug.Log($"[AISReplay] {_vessels.Count} vessel(s) loaded  " +
                  $"(spoof filter = {filterSpoofedVessels}).");
        if (_vessels.Count == 0) return;

        // ── Timeline ─────────────────────────────────────────────────────
        _dataStart      = _vessels.SelectMany(v => v.Records).Min(r => r.Timestamp);
        _startOffsetSec = 0;
        _accumSimSec    = 0;   // always reset accumulator

        if (useCustomStartTime)
        {
            if (DateTime.TryParse(customStartTimeISO, null,
                    DateTimeStyles.RoundtripKind, out DateTime cdt))
            {
                double req = (cdt.ToUniversalTime() - _dataStart).TotalSeconds;
                _startOffsetSec = Math.Max(0, req);
                Debug.Log($"[AISReplay] Start offset = {_startOffsetSec:F1} s  ({cdt:u})");
            }
            else
            {
                Debug.LogWarning($"[AISReplay] Cannot parse '{customStartTimeISO}' – starting from t=0.");
            }
        }

        // ── Prime each vessel's SegIdx to the correct starting segment ────
        // We do this BEFORE setting _playing = true so DriveVessel's
        // while-loop never runs at t=0 and immediately races to the end.
        DateTime startSimTime = _dataStart.AddSeconds(_startOffsetSec);
        foreach (var vp in _vessels)
        {
            // Find the last record whose timestamp is <= startSimTime
            // That becomes p1 (departure side) of the first live segment.
            int idx = 0;
            for (int i = 0; i < vp.Records.Count - 1; i++)
            {
                if (vp.Records[i].Timestamp <= startSimTime) idx = i;
                else break;
            }
            vp.SegIdx = idx;   // clamped to [0 .. Count-2] by the loop above
            SpawnVessel(vp, startSimTime);
        }

        _activeVessels = _vessels.Count;
        _playing = true;

        Debug.Log($"[AISReplay] Playing  data-origin={_dataStart:u}  " +
                  $"start-offset={_startOffsetSec:F1}s  speed={replaySpeed}x");
    }

    public void StopReplay()
    {
        _playing = false;
        _accumSimSec = 0;

        // Destroy all spawned vessel GameObjects
        foreach (var vp in _vessels)
            if (vp.Go != null) Destroy(vp.Go);

        _vessels.Clear();
        _byMMSI.Clear();
        _activeVessels  = 0;
        _simTimeDisplay = "--";
        _simElapsedSec  = 0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Geo conversion  (identical constants and formula to vGPS)
    // ─────────────────────────────────────────────────────────────────────────

    private void ResolveOriginTransform()
    {
        if (gpsOriginTransform != null) return;
        var go = GameObject.Find("GPS_CORD_ORIGIN");
        if (go != null)
            gpsOriginTransform = go.transform;
        else
            Debug.LogWarning("[AISReplay] GPS_CORD_ORIGIN not found in scene. " +
                             "Vessels will be positioned relative to world origin.");
    }

    private void ComputeGeoScale()
    {
        // Matches vGPS UpdateGPS() exactly
        _metersPerDegLat = 111111.0;
        _metersPerDegLon = 111111.0 * Math.Cos(originLatitude * Math.PI / 180.0);
    }

    /// <summary>
    /// Converts geodetic lat/lon to Unity world-space XZ.
    /// Uses the same offset-from-origin formula as vGPS so both coordinate
    /// systems are always in sync.
    /// </summary>
    private Vector3 GeoToWorld(double lat, double lon)
    {
        float x = (float)((lon - originLongitude) * _metersPerDegLon);
        float z = (float)((lat - originLatitude)  * _metersPerDegLat);

        Vector3 pos = new Vector3(x, 0f, z);

        if (gpsOriginTransform != null)
            pos += gpsOriginTransform.position;

        return pos;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Spoof detection
    // ─────────────────────────────────────────────────────────────────────────

    private bool HasSpoofedJump(List<AISRecord> recs)
    {
        for (int i = 1; i < recs.Count; i++)
        {
            double d = HaversineKm(recs[i-1].Lat, recs[i-1].Lon,
                                    recs[i  ].Lat, recs[i  ].Lon);
            if (d >= spoofDetectionThresholdKm)
            {
                Debug.Log($"[AISReplay]   jump {d:F1} km at record {i}  " +
                          $"MMSI={recs[0].MMSI}  \"{recs[0].Name}\"");
                return true;
            }
        }
        return false;
    }

    private static double HaversineKm(double la1, double lo1, double la2, double lo2)
    {
        const double R  = 6371.0;
        double dLa = (la2 - la1) * Math.PI / 180.0;
        double dLo = (lo2 - lo1) * Math.PI / 180.0;
        double a   = Math.Sin(dLa / 2) * Math.Sin(dLa / 2)
                   + Math.Cos(la1 * Math.PI / 180.0) * Math.Cos(la2 * Math.PI / 180.0)
                   * Math.Sin(dLo / 2) * Math.Sin(dLo / 2);
        return R * 2.0 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1.0 - a));
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Vessel spawning
    // ─────────────────────────────────────────────────────────────────────────

    private void SpawnVessel(VesselPlayback vp, DateTime startSimTime)
    {
        // ── Instantiate under the auto-created AIS_Vessels container ─────
        GameObject go;
        if (vesselPrefab != null)
        {
            go = Instantiate(vesselPrefab, _vesselContainer);
        }
        else
        {
            go = new GameObject();
            go.transform.SetParent(_vesselContainer, false);
        }
        go.name = $"AIS_{vp.MMSI}_{vp.Name}";

        // ── AISTransponder ────────────────────────────────────────────────
        AISTransponder tr = go.GetComponent<AISTransponder>();
        if (tr == null) tr = go.AddComponent<AISTransponder>();

        // AISTransponder.Start() only calls Register() and Broadcast() —
        // it does NOT reset shipType or ship_lenght, so set everything here directly.
        float shipLen         = ShipLengthForType(vp.ShipType);
        tr.vesselName         = vp.Name;
        tr.mmsi               = vp.MMSI;
        tr.shipType           = vp.ShipType;
        tr.ship_lenght        = shipLen;        // note: matches the typo in AISTransponder.cs
        tr.broadcastInterval  = 2f;
        tr.initWithRandomMMSI = false;

        // ── Model child scale ─────────────────────────────────────────────
        // Scale the Model child to match the vessel's real-world length.
        // Root stays at (1,1,1) so position/rotation driving is unaffected.
        ApplyModelScale(go, shipLen);

        // ── ReplayAISGPS (vGPS-compatible shim) ───────────────────────────
        ReplayAISGPS gps = go.GetComponent<ReplayAISGPS>();
        if (gps == null) gps = go.AddComponent<ReplayAISGPS>();
        gps.Bind(tr);

        vp.Go          = go;
        vp.Transponder = tr;
        vp.GPS         = gps;

        // ── Initial position at the primed segment ────────────────────────
        var recs = vp.Records;
        if (recs.Count > 0)
        {
            int       si     = vp.SegIdx;
            AISRecord r1     = recs[si];
            AISRecord r2     = si + 1 < recs.Count ? recs[si + 1] : r1;
            double    segDur = (r2.Timestamp - r1.Timestamp).TotalSeconds;
            float     t0     = segDur > 0
                ? Mathf.Clamp01((float)((startSimTime - r1.Timestamp).TotalSeconds / segDur))
                : 0f;

            Vector3 p0 = GeoToWorld(si > 0              ? recs[si - 1].Lat : r1.Lat,
                                    si > 0              ? recs[si - 1].Lon : r1.Lon);
            Vector3 p1 = GeoToWorld(r1.Lat, r1.Lon);
            Vector3 p2 = GeoToWorld(r2.Lat, r2.Lon);
            Vector3 p3 = GeoToWorld(si + 2 < recs.Count ? recs[si + 2].Lat : r2.Lat,
                                    si + 2 < recs.Count ? recs[si + 2].Lon : r2.Lon);

            Vector3 initPos = CatmullRom(p0, p1, p2, p3, t0);
            go.transform.position    = initPos;
            go.transform.eulerAngles = new Vector3(0f, r1.CourseDeg, 0f);

            double initLat = r1.Lat + (r2.Lat - r1.Lat) * t0;
            double initLon = r1.Lon + (r2.Lon - r1.Lon) * t0;
            gps.ForcePosition(initLat, initLon);
            tr.latitude   = initLat;
            tr.longitude  = initLon;
            tr.heading    = r1.CourseDeg;
            tr.speedKnots = r1.SpeedKn;
        }
    }

    /// <summary>
    /// Returns a representative real-world length in metres for a given AIS ship type.
    /// These are typical values — the CSV carries no length data so we approximate.
    /// You can override per-vessel via the defaultShipLengthM inspector field.
    /// </summary>
    private float ShipLengthForType(AisShipType type)
    {
        switch (type)
        {
            case AisShipType.Tanker:         return 250f;
            case AisShipType.Cargo:          return 180f;
            case AisShipType.Passenger:      return 200f;
            case AisShipType.HighSpeedCraft: return  40f;
            case AisShipType.Tug:            return  30f;
            case AisShipType.PilotVessel:    return  20f;
            case AisShipType.SearchAndRescue:return  15f;
            case AisShipType.Fishing:        return  25f;
            case AisShipType.Sailing:        return  10f;
            case AisShipType.PleasureCraft:  return  10f;
            case AisShipType.AtoN:           return   5f;
            default:                         return defaultShipLengthM;
        }
    }

    /// <summary>
    /// Scales the named Model child (or root if not found) to the vessel's
    /// real-world length.  Also writes the length back to AISTransponder.ship_lenght.
    /// Root transform stays at (1,1,1) so position/rotation driving is clean.
    /// </summary>
    private void ApplyModelScale(GameObject vesselRoot, float lengthM)
    {
        if (lengthM <= 0f) return;

        Transform target = vesselRoot.transform;
        if (!string.IsNullOrEmpty(modelChildName))
        {
            Transform child = vesselRoot.transform.Find(modelChildName);
            if (child != null)
                target = child;
            else
                Debug.LogWarning($"[AISReplay] Child \"{modelChildName}\" not found on " +
                                 $"{vesselRoot.name} — scaling root instead.");
        }

        target.localScale = Vector3.one * lengthM;
        Debug.Log($"[AISReplay] {vesselRoot.name}  shipLen={lengthM}m  scale applied to '{target.name}'");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Per-frame vessel driving  (Catmull-Rom spline)
    // ─────────────────────────────────────────────────────────────────────────

    private void DriveVessel(VesselPlayback vp, DateTime simNow)
    {
        var recs = vp.Records;
        if (recs.Count == 0 || vp.Go == null) return;

        // ── Advance segment pointer ───────────────────────────────────────
        // Only advance when the NEXT record's time has been reached.
        // The pointer is already primed to the correct segment by LoadAndStart,
        // so on the first frame this loop body will NOT execute (simNow == startSimTime).
        while (vp.SegIdx < recs.Count - 2 &&
               recs[vp.SegIdx + 1].Timestamp <= simNow)
        {
            vp.SegIdx++;
        }

        // Past end of data — hold at last position
        if (vp.SegIdx >= recs.Count - 1)
        {
            var last = recs[recs.Count - 1];
            vp.Go.transform.position = GeoToWorld(last.Lat, last.Lon);
            PushToTransponder(vp, last.Lat, last.Lon, last.SpeedKn, last.CourseDeg);
            return;
        }

        // ── t within current segment ──────────────────────────────────────
        AISRecord r1    = recs[vp.SegIdx];
        AISRecord r2    = recs[vp.SegIdx + 1];
        double    segDur = (r2.Timestamp - r1.Timestamp).TotalSeconds;
        float     t      = segDur > 0.0
            ? Mathf.Clamp01((float)((simNow - r1.Timestamp).TotalSeconds / segDur))
            : 1f;

        // ── Catmull-Rom control points ────────────────────────────────────
        //   p0 = waypoint before r1  (clamp to r1 at track start)
        //   p1 = current segment start  (r1)
        //   p2 = current segment end    (r2)
        //   p3 = waypoint after  r2  (clamp to r2 at track end)
        int     si = vp.SegIdx;
        Vector3 p0 = GeoToWorld(si > 0              ? recs[si - 1].Lat : r1.Lat,
                                si > 0              ? recs[si - 1].Lon : r1.Lon);
        Vector3 p1 = GeoToWorld(r1.Lat, r1.Lon);
        Vector3 p2 = GeoToWorld(r2.Lat, r2.Lon);
        Vector3 p3 = GeoToWorld(si + 2 < recs.Count ? recs[si + 2].Lat : r2.Lat,
                                si + 2 < recs.Count ? recs[si + 2].Lon : r2.Lon);

        // ── Interpolated world position ───────────────────────────────────
        Vector3 splinePos = CatmullRom(p0, p1, p2, p3, t);
        splinePos.y = vp.Go.transform.position.y;   // preserve buoyancy / terrain Y

        // ── Heading from spline tangent ───────────────────────────────────
        Vector3 tangent = CatmullRomTangent(p0, p1, p2, p3, t);
        float heading = tangent.sqrMagnitude > 1e-4f
            ? Mathf.Atan2(tangent.x, tangent.z) * Mathf.Rad2Deg
            : Mathf.LerpAngle(r1.CourseDeg, r2.CourseDeg, t);

        // ── Apply ─────────────────────────────────────────────────────────
        vp.Go.transform.position    = splinePos;
        vp.Go.transform.eulerAngles = new Vector3(0f, heading, 0f);

        double lat = r1.Lat + (r2.Lat - r1.Lat) * t;
        double lon = r1.Lon + (r2.Lon - r1.Lon) * t;
        float  spd = Mathf.Lerp(r1.SpeedKn, r2.SpeedKn, t);
        PushToTransponder(vp, lat, lon, spd, heading);
    }

    private void PushToTransponder(VesselPlayback vp,
                                    double lat, double lon,
                                    float spd, float heading)
    {
        vp.GPS.ForcePosition(lat, lon);
        if (vp.Transponder != null)
        {
            vp.Transponder.latitude   = lat;
            vp.Transponder.longitude  = lon;
            vp.Transponder.speedKnots = spd;
            vp.Transponder.heading    = heading;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Catmull-Rom maths
    // ─────────────────────────────────────────────────────────────────────────

    private static Vector3 CatmullRom(Vector3 p0, Vector3 p1,
                                       Vector3 p2, Vector3 p3, float t)
    {
        float t2 = t  * t;
        float t3 = t2 * t;
        return 0.5f * (
              2f * p1
            + (-p0 + p2)                     * t
            + ( 2f*p0 - 5f*p1 + 4f*p2 - p3) * t2
            + (-p0   + 3f*p1  - 3f*p2 + p3) * t3);
    }

    private static Vector3 CatmullRomTangent(Vector3 p0, Vector3 p1,
                                              Vector3 p2, Vector3 p3, float t)
    {
        float t2 = t * t;
        return 0.5f * (
              (-p0 + p2)
            + ( 2f*p0 - 5f*p1 + 4f*p2 - p3) * (2f * t)
            + (-p0   + 3f*p1  - 3f*p2 + p3) * (3f * t2));
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  CSV parser
    // ─────────────────────────────────────────────────────────────────────────

    private List<AISRecord> ParseCSV(string path)
    {
        var result = new List<AISRecord>();
        var lines  = File.ReadAllLines(path);
        if (lines.Length < 2) return result;

        string[] hdr = lines[0].Split(',');
        int iTs  = ColIdx(hdr, "timestamp");
        int iId  = ColIdx(hdr, "mmsi");
        int iNm  = ColIdx(hdr, "name");
        int iSt  = ColIdx(hdr, "ship_type");
        int iLa  = ColIdx(hdr, "lat");
        int iLo  = ColIdx(hdr, "lon");
        int iSpd = ColIdx(hdr, "speed_kn");
        int iCrs = ColIdx(hdr, "course_deg");

        if (iTs < 0 || iId < 0 || iLa < 0 || iLo < 0)
        {
            Debug.LogError("[AISReplay] CSV missing required columns " +
                           "(need at minimum: timestamp, mmsi, lat, lon).");
            return result;
        }

        int skipped = 0;
        for (int i = 1; i < lines.Length; i++)
        {
            string ln = lines[i].Trim();
            if (string.IsNullOrEmpty(ln)) continue;
            string[] c = ln.Split(',');
            try
            {
                result.Add(new AISRecord
                {
                    Timestamp = DateTime.Parse(c[iTs].Trim(), null,
                                    DateTimeStyles.RoundtripKind).ToUniversalTime(),
                    MMSI      = int.Parse(c[iId].Trim()),
                    Name      = iNm  >= 0 ? c[iNm ].Trim() : "UNKNOWN",
                    ShipType  = iSt  >= 0 ? int.Parse(c[iSt].Trim()) : 0,
                    Lat       = double.Parse(c[iLa].Trim(),  CultureInfo.InvariantCulture),
                    Lon       = double.Parse(c[iLo].Trim(),  CultureInfo.InvariantCulture),
                    SpeedKn   = iSpd >= 0
                                    ? float.Parse(c[iSpd].Trim(), CultureInfo.InvariantCulture)
                                    : 0f,
                    CourseDeg = iCrs >= 0
                                    ? float.Parse(c[iCrs].Trim(), CultureInfo.InvariantCulture)
                                    : 0f,
                });
            }
            catch (Exception ex)
            {
                skipped++;
                if (skipped <= 5)
                    Debug.LogWarning($"[AISReplay] Skipping row {i}: {ex.Message}");
            }
        }

        if (skipped > 0)
            Debug.LogWarning($"[AISReplay] {skipped} total rows skipped (parse errors).");

        return result;
    }

    private static int ColIdx(string[] hdr, string name) =>
        Array.FindIndex(hdr, h => h.Trim().ToLower() == name);

    private static string ResolvePath(string p)
    {
        if (Path.IsPathRooted(p)) return p;
        string s = Path.Combine(Application.streamingAssetsPath, p);
        if (File.Exists(s)) return s;
        return Path.Combine(Application.dataPath, p);
    }
}


// =============================================================================
//  ReplayAISGPS
//
//  vGPS-compatible shim — exposes currentLat / currentLon with the same names
//  as vGPS so AISTransponder.Broadcast() reads correct coordinates.
//  AISReplaySystem calls ForcePosition() every frame with interpolated data.
// =============================================================================
[DisallowMultipleComponent]
public class ReplayAISGPS : MonoBehaviour
{
    /// <summary>Mirrors vGPS.currentLat – read by AISTransponder.Broadcast().</summary>
    public double currentLat { get; private set; }

    /// <summary>Mirrors vGPS.currentLon – read by AISTransponder.Broadcast().</summary>
    public double currentLon { get; private set; }

    private AISTransponder _transponder;

    public void Bind(AISTransponder t) => _transponder = t;

    public void ForcePosition(double lat, double lon)
    {
        currentLat = lat;
        currentLon = lon;
        if (_transponder != null)
        {
            _transponder.latitude  = lat;
            _transponder.longitude = lon;
        }
    }
}