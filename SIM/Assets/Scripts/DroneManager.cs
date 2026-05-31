using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class DroneManager : MonoBehaviour
{

    [Header("Spawn Settings")]
    public GameObject shipPrefab;      // Drag your ship prefab here
    public int shipCount = 5;          // N ships
    public Transform spawnOrigin;      // The "starting point" transform
    public Vector3 spacingOffset = new Vector3(20f, 0f, 0f); // Distance between each ship

    [Header("Fleet Data")]
    public GameObject[] shipArray;     // The stored array of spawned ships

    void Start()
    {
        SpawnFleet();
    }

    void SpawnFleet()
    {
        // Initialize the array
        shipArray = new GameObject[shipCount];

        for (int i = 0; i < shipCount; i++)
        {
            // Calculate position: Origin + (Offset * Index)
            Vector3 spawnPos = spawnOrigin.position + (spacingOffset * i);
            
            // Spawn the ship
            GameObject newShip = Instantiate(shipPrefab, spawnPos, spawnOrigin.rotation);
            
            // Name it for clarity
            newShip.name = "Ship_" + i;

            // Store it in the array
            shipArray[i] = newShip;
        }

        Debug.Log($"Successfully spawned {shipCount} ships in formation.");
    }
}