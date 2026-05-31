using UnityEngine;
using System.Collections.Generic;

// Attach to each ship that RECEIVES AIS — reads other ships' broadcasts
public class AISReceiver : MonoBehaviour
{
    [Header("Settings")]
    public float receiverRange = 200f;      // Real AIS range: ~20-40 nautical miles

    // Table of what this ship currently knows
    public Dictionary<int, AISTransponder> knownVessels 
        = new Dictionary<int, AISTransponder>();


    [System.Serializable]

    public class AISContactEntry
    {
        public string vesselName;
        public int    mmsi;
        public float  distanceMeters;
        public float  speedKnots;
        public float  heading;
        public float ship_lenght;
        public double latitude;     
        public double longitude;    
        public AisShipType shipType;
    }

    [Header("Contacts (Read Only)")]
    public List<AISContactEntry> contactList = new List<AISContactEntry>();

    void OnEnable()  => AISManager.OnVesselUpdated += OnVesselBroadcast;
    void OnDisable() => AISManager.OnVesselUpdated -= OnVesselBroadcast;

    private void OnVesselBroadcast(AISTransponder transponder)
    {
        if (transponder.gameObject == gameObject) return;

        float rangeSq = receiverRange * receiverRange;
        float dSq     = (transponder.position - transform.position).sqrMagnitude;

        if (dSq < rangeSq)
            knownVessels[transponder.mmsi] = transponder;
        else
            knownVessels.Remove(transponder.mmsi);

        RefreshContactList(); // <- add this
    }

    private void RefreshContactList()
    {
        contactList.Clear();
        foreach (var v in knownVessels.Values)
        {
           contactList.Add(new AISContactEntry
            {
                vesselName     = v.vesselName,
                mmsi           = v.mmsi,
                shipType       = v.shipType,
                distanceMeters = Mathf.Sqrt((v.position - transform.position).sqrMagnitude),
                speedKnots     = v.speedKnots,
                heading        = v.heading,
                ship_lenght    = v.ship_lenght,
                latitude       = v.latitude,   
                longitude      = v.longitude   
            });
        }
    }

    // Nearest vessel of any type
    public AISTransponder GetNearest()
    {
        AISTransponder nearest  = null;
        float          nearestSq = float.MaxValue;

        foreach (var v in knownVessels.Values)
        {
            float dSq = (v.position - transform.position).sqrMagnitude;
            if (dSq < nearestSq) { nearestSq = dSq; nearest = v; }
        }
        return nearest;
    }

    // CPA — Closest Point of Approach, useful for collision avoidance
    public float GetCPA(AISTransponder other)
    {
        if (other == null) return float.MaxValue;

        Rigidbody myRb    = GetComponent<Rigidbody>();
        Rigidbody otherRb = other.GetComponent<Rigidbody>();
        if (myRb == null || otherRb == null) return float.MaxValue;

        Vector3 relPos = other.position        - transform.position;
        Vector3 relVel = otherRb.velocity - myRb.velocity;

        if (relVel.sqrMagnitude < 0.001f) return relPos.magnitude;

        float t = -Vector3.Dot(relPos, relVel) / relVel.sqrMagnitude;
        t = Mathf.Max(0, t);  // Only future CPA

        return (relPos + relVel * t).magnitude;
    }
}