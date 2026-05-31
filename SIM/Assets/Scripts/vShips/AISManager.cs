using UnityEngine;
using System.Collections.Generic;

// Central registry — ships and buoys register here
public static class AISManager
{
    private static Dictionary<int, AISTransponder> _transponders 
        = new Dictionary<int, AISTransponder>();

    public static event System.Action<AISTransponder> OnVesselUpdated;

    public static void Register(AISTransponder t)
    {
        if (!_transponders.ContainsKey(t.mmsi))
            _transponders.Add(t.mmsi, t);
    }

    public static void Unregister(AISTransponder t) 
        => _transponders.Remove(t.mmsi);

    public static void OnBroadcast(AISTransponder t)
        => OnVesselUpdated?.Invoke(t);

    // Query by MMSI
    public static AISTransponder GetByMMSI(int mmsi)
        => _transponders.TryGetValue(mmsi, out var t) ? t : null;

    // Get all vessels within range of a position
    public static List<AISTransponder> GetInRange(Vector3 position, float range)
    {
        List<AISTransponder> result = new List<AISTransponder>();
        float rangeSq = range * range;

        foreach (var t in _transponders.Values)
            if (t.isActive && (t.position - position).sqrMagnitude < rangeSq)
                result.Add(t);

        return result;
    }

    public static IEnumerable<AISTransponder> GetAll() => _transponders.Values;


    public static List<AISTransponder> GetAllList() => new List<AISTransponder>(_transponders.Values);
    

}