using UnityEngine;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class BuoyZone : MonoBehaviour
{
    public enum BuoyType { North, South, East, West }

    [System.Serializable]
    public class Buoy
    {
        public BuoyType type;
        public Transform transform;
    }

    [Header("Buoys")]
    public List<Buoy> buoys = new List<Buoy>();

    [Header("Zone Line")]
    public Color lineColor = new Color(1f, 0.2f, 0.2f, 1f);
    public float lineWidth = 0.5f;
    public float waterOffset = 0.1f; // Keeps line just above water surface

    private LineRenderer _lineRenderer;

    void Start()
    {
        BuildZone();
    }

    public void BuildZone()
    {
        List<Vector3> hull = GetHullPoints();
        if (hull.Count < 2) return;

        if (_lineRenderer == null)
        {
            _lineRenderer = gameObject.AddComponent<LineRenderer>();
            _lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            _lineRenderer.useWorldSpace = true;
            _lineRenderer.loop = true;
        }

        _lineRenderer.startWidth = lineWidth;
        _lineRenderer.endWidth   = lineWidth;
        _lineRenderer.startColor = lineColor;
        _lineRenderer.endColor   = lineColor;

        _lineRenderer.positionCount = hull.Count;
        for (int i = 0; i < hull.Count; i++)
            _lineRenderer.SetPosition(i, hull[i] + Vector3.up * waterOffset);
    }

    private List<Vector3> GetHullPoints()
    {
        List<Vector3> points = new List<Vector3>();
        foreach (var b in buoys)
            if (b.transform != null)
                points.Add(b.transform.position);

        if (points.Count < 2) return points;

        Vector3 centroid = Vector3.zero;
        foreach (var p in points) centroid += p;
        centroid /= points.Count;

        points.Sort((a, b) =>
        {
            float angleA = Mathf.Atan2(a.z - centroid.z, a.x - centroid.x);
            float angleB = Mathf.Atan2(b.z - centroid.z, b.x - centroid.x);
            return angleB.CompareTo(angleA);
        });

        return points;
    }

    public bool IsInsideForbiddenZone(Vector3 worldPosition)
    {
        if (_lineRenderer == null || _lineRenderer.positionCount < 3) return false;

        Vector2 point = new Vector2(worldPosition.x, worldPosition.z);
        bool inside = false;
        int count = _lineRenderer.positionCount;
        int j = count - 1;

        for (int i = 0; i < count; i++)
        {
            Vector3 pi = _lineRenderer.GetPosition(i);
            Vector3 pj = _lineRenderer.GetPosition(j);
            Vector2 a = new Vector2(pi.x, pi.z);
            Vector2 b = new Vector2(pj.x, pj.z);

            if (((a.y > point.y) != (b.y > point.y)) &&
                (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x))
                inside = !inside;

            j = i;
        }
        return inside;
    }

    void OnDrawGizmos()
    {
        List<Vector3> pts = GetHullPoints();
        if (pts.Count < 2) return;

        Gizmos.color = lineColor;
        for (int i = 0; i < pts.Count; i++)
            Gizmos.DrawLine(pts[i], pts[(i + 1) % pts.Count]);

#if UNITY_EDITOR
        foreach (var b in buoys)
        {
            if (b.transform == null) continue;
            Handles.Label(b.transform.position + Vector3.up * 1.5f, b.type.ToString());
        }
#endif
    }
}