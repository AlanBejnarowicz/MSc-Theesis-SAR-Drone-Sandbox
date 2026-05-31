using UnityEngine;
using System.Collections.Generic;

public class DroneFleetManager : MonoBehaviour
{
    [Header("Spawn")]
    public GameObject  dronePrefab;
    public Transform[] spawnPoints;
    public int         droneCount = 3;

    [Header("UDP Base Ports")]
    public int sendBasePort    = 5000;
    public int receiveBasePort = 6000;

    [Header("C++ Host")]
    public string controllerHost = "127.0.0.1";

    private List<GameObject> _drones = new List<GameObject>();

    void Start() => SpawnFleet();

    private void SpawnFleet()
    {
        for (int i = 0; i < droneCount; i++)
        {
            Transform spawn = spawnPoints[i % spawnPoints.Length];
            GameObject go   = Instantiate(dronePrefab, spawn.position, spawn.rotation);
            go.name         = $"UAV_{i}";

            DroneShipData data = go.GetComponent<DroneShipData>();
            data.ShipId        = i;
            

            DroneUDPBridge bridge = go.GetComponent<DroneUDPBridge>();
            bridge.sendHost       = controllerHost;
            bridge.sendPort       = sendBasePort + i;
            bridge.receivePort    = receiveBasePort + i;
            bridge.enabled = true;


            AISTransponder trs = go.GetComponent<AISTransponder>();
            trs.vesselName = go.name;

            LoRaRadio lrdo = go.GetComponent<LoRaRadio>();
            lrdo.nodeId = i;

            _drones.Add(go);

            Debug.Log($"Spawned {go.name} | MMSI {data.drone_MMSI} | " +
                      $"TX:{sendBasePort + i} RX:{receiveBasePort + i}");
        }
    }

    public void RespawnFleet()
    {
        foreach (var d in _drones) Destroy(d);
        _drones.Clear();
        SpawnFleet();
    }
}
