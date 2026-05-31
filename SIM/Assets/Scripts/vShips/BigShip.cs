using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class BigShip : MonoBehaviour
{
    [Header("Spline Waypoints")]
    [Tooltip("Assign empty GameObjects in order to define the path")]
    public List<Transform> waypoints = new List<Transform>();
    public bool            loopPath  = true;

    [Header("Movement")]
    public float maxSpeed         = 6f;       // m/s — large ship ~10 knots
    public float acceleration     = 0.3f;     // Very slow to accelerate
    public float deceleration     = 0.15f;    // Even slower to stop
    public float waypointRadius   = 15f;      // Distance to advance to next waypoint

    [Header("Steering")]
    public float turnSpeed        = 8f;       // Degrees/sec — big ships turn slowly
    public float rudderSmoothing  = 3f;       // Smooths out heading changes

    [Header("Ship Feel")]
    public float rollAmount       = 2f;       // Side tilt when turning
    public float rollSmoothing    = 2f;
    public float pitchAmount      = 0.5f;     // Nose dip when accelerating
    public float pitchSmoothing   = 2f;
    public float wakeTrailOffset  = -5f;      // Z offset for wake effect (if any)

    [Header("Spline Settings")]
    public bool  useCatmullRom    = true;     // Smooth curve vs straight segments
    [Range(1, 20)]
    public int   splineResolution = 10;       // Subdivisions per segment for gizmo

    // ── Runtime state ──────────────────────────────────────
    private int     _currentIndex  = 0;
    private float   _currentSpeed  = 0f;
    private float   _currentRoll   = 0f;
    private float   _currentPitch  = 0f;
    private float   _rudderAngle   = 0f;
    private Rigidbody _rb;

    // AIS transponder — auto-updated from movement
    private AISTransponder _ais;

    void Start()
    {
        _rb  = GetComponent<Rigidbody>();
        _ais = GetComponent<AISTransponder>();

        if (_rb != null)
        {
            _rb.useGravity  = false;
            _rb.constraints = RigidbodyConstraints.FreezePositionY
                            | RigidbodyConstraints.FreezeRotationX
                            | RigidbodyConstraints.FreezeRotationZ;
        }

        if (waypoints.Count > 0)
            AlignToSpline();
    }

    void FixedUpdate()
    {
        if (waypoints.Count < 2) return;

        Vector3 target   = GetCurrentWaypointPosition();
        float   distance = Vector3.Distance(transform.position, target);

        // Advance waypoint
        if (distance < waypointRadius)
            AdvanceWaypoint();

        // How far is the NEXT waypoint — used for speed management
        float lookAheadDist = GetLookAheadDistance();

        UpdateSpeed(distance, lookAheadDist);
        UpdateSteering(target);
        UpdateShipFeel();
        ApplyMovement();
    }

    // ──────────────────────────────────────────────────────
    // Speed — slow in, slow out around waypoints
    // ──────────────────────────────────────────────────────
    private void UpdateSpeed(float distToWaypoint, float lookAhead)
    {
        // Slow down approaching tight turns
        float cornerSpeed = Mathf.Lerp(maxSpeed * 0.4f, maxSpeed,
            Mathf.Clamp01(lookAhead / 60f));

        float targetSpeed = cornerSpeed;

        // Decelerate into waypoint
        if (distToWaypoint < 30f)
            targetSpeed = Mathf.Lerp(maxSpeed * 0.3f, cornerSpeed,
                distToWaypoint / 30f);

        // Smooth acceleration / deceleration
        float rate = targetSpeed > _currentSpeed ? acceleration : deceleration;
        _currentSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed,
            rate * Time.fixedDeltaTime * 10f);
    }

    // ──────────────────────────────────────────────────────
    // Steering — slow yaw toward target
    // ──────────────────────────────────────────────────────
    private void UpdateSteering(Vector3 target)
    {
        Vector3 dir = (target - transform.position);
        dir.y = 0f;
        if (dir.sqrMagnitude < 0.01f) return;

        float targetYaw  = Mathf.Atan2(dir.x, dir.z) * Mathf.Rad2Deg;
        float currentYaw = transform.eulerAngles.y;
        float yawError   = Mathf.DeltaAngle(currentYaw, targetYaw);

        // Rudder angle smoothed — big ships respond sluggishly
        float targetRudder = Mathf.Clamp(yawError * 0.1f, -1f, 1f);
        _rudderAngle = Mathf.MoveTowards(_rudderAngle, targetRudder,
            rudderSmoothing * Time.fixedDeltaTime);

        float yawDelta = _rudderAngle * turnSpeed * Time.fixedDeltaTime;
        transform.Rotate(0f, yawDelta, 0f, Space.World);
    }

    // ──────────────────────────────────────────────────────
    // Roll & pitch — visual feel only
    // ──────────────────────────────────────────────────────
    private void UpdateShipFeel()
    {
        // Roll into turns
        float targetRoll = -_rudderAngle * rollAmount;
        _currentRoll = Mathf.Lerp(_currentRoll, targetRoll,
            rollSmoothing * Time.fixedDeltaTime);

        // Pitch with acceleration
        float accelFactor = (_currentSpeed / Mathf.Max(maxSpeed, 0.01f)) - 0.5f;
        float targetPitch = -accelFactor * pitchAmount;
        _currentPitch = Mathf.Lerp(_currentPitch, targetPitch,
            pitchSmoothing * Time.fixedDeltaTime);

        // Apply — only roll/pitch, yaw handled by steering
        Vector3 euler = transform.eulerAngles;
        transform.eulerAngles = new Vector3(_currentPitch, euler.y, _currentRoll);
    }

    // ──────────────────────────────────────────────────────
    // Move forward
    // ──────────────────────────────────────────────────────
    private void ApplyMovement()
    {
        if (_rb != null)
        {
            Vector3 forward = new Vector3(transform.forward.x, 0f, transform.forward.z)
                .normalized;
            _rb.velocity = forward * _currentSpeed;

        }
        else
        {
            transform.position += transform.forward * _currentSpeed * Time.fixedDeltaTime;
        }
    }

    // ──────────────────────────────────────────────────────
    // Spline helpers
    // ──────────────────────────────────────────────────────
    private Vector3 GetCurrentWaypointPosition()
    {
        if (!useCatmullRom || waypoints.Count < 4)
            return waypoints[_currentIndex].position;

        // Sample spline slightly ahead of current index for smooth look-ahead
        return CatmullRom(
            GetWaypoint(_currentIndex - 1),
            GetWaypoint(_currentIndex),
            GetWaypoint(_currentIndex + 1),
            GetWaypoint(_currentIndex + 2),
            0.5f);
    }

    private float GetLookAheadDistance()
    {
        int next = (_currentIndex + 1) % waypoints.Count;
        if (next >= waypoints.Count) return 999f;
        return Vector3.Distance(
            waypoints[_currentIndex].position,
            waypoints[next].position);
    }

    private void AdvanceWaypoint()
    {
        _currentIndex++;
        if (_currentIndex >= waypoints.Count)
        {
            if (loopPath)
                _currentIndex = 0;
            else
            {
                _currentIndex = waypoints.Count - 1;
                _currentSpeed = 0f;
            }
        }
    }

    // Catmull-Rom spline between p1 and p2
    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        float t2 = t * t;
        float t3 = t2 * t;
        return 0.5f * (
              2f * p1
            + (-p0 + p2) * t
            + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2
            + (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
    }

    private Vector3 GetWaypoint(int index)
    {
        if (loopPath)
            return waypoints[((index % waypoints.Count) + waypoints.Count) % waypoints.Count].position;
        return waypoints[Mathf.Clamp(index, 0, waypoints.Count - 1)].position;
    }

    // Snap ship to spline direction on start
    private void AlignToSpline()
    {
        if (waypoints.Count < 2) return;
        Vector3 dir = (waypoints[1].position - waypoints[0].position).normalized;
        dir.y = 0f;
        if (dir.sqrMagnitude > 0.01f)
            transform.rotation = Quaternion.LookRotation(dir);
    }

    // ──────────────────────────────────────────────────────
    // Gizmos
    // ──────────────────────────────────────────────────────
    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Count < 2) return;

        // Draw spline
        Gizmos.color = Color.cyan;
        int count = loopPath ? waypoints.Count : waypoints.Count - 1;

        for (int i = 0; i < count; i++)
        {
            if (waypoints[i] == null) continue;

            if (!useCatmullRom || waypoints.Count < 4)
            {
                Vector3 a = waypoints[i].position;
                Vector3 b = waypoints[(i + 1) % waypoints.Count].position;
                Gizmos.DrawLine(a, b);
            }
            else
            {
                Vector3 prev = CatmullRom(
                    GetWaypointSafe(i - 1), GetWaypointSafe(i),
                    GetWaypointSafe(i + 1), GetWaypointSafe(i + 2), 0f);

                for (int s = 1; s <= splineResolution; s++)
                {
                    float   t    = (float)s / splineResolution;
                    Vector3 next = CatmullRom(
                        GetWaypointSafe(i - 1), GetWaypointSafe(i),
                        GetWaypointSafe(i + 1), GetWaypointSafe(i + 2), t);
                    Gizmos.DrawLine(prev, next);
                    prev = next;
                }
            }

            // Waypoint spheres
            Gizmos.color = (i == _currentIndex) ? Color.yellow : Color.white;
            Gizmos.DrawWireSphere(waypoints[i].position, waypointRadius * 0.5f);
            Gizmos.color = Color.cyan;
        }

        // Direction arrow on ship
        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, transform.forward * 20f);

#if UNITY_EDITOR
        // Waypoint labels
        GUIStyle style = new GUIStyle();
        style.normal.textColor = Color.yellow;
        for (int i = 0; i < waypoints.Count; i++)
            if (waypoints[i] != null)
                Handles.Label(waypoints[i].position + Vector3.up * 3f,
                    $"WP {i}", style);
#endif
    }

    private Vector3 GetWaypointSafe(int index)
    {
        if (waypoints.Count == 0) return Vector3.zero;
        if (loopPath)
            return waypoints[((index % waypoints.Count) + waypoints.Count)
                % waypoints.Count].position;
        return waypoints[Mathf.Clamp(index, 0, waypoints.Count - 1)].position;
    }
}
