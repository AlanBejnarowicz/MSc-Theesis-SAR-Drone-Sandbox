using UnityEngine;
using System.Collections.Generic;

public class BenchmarkManager : MonoBehaviour
{
    [Header("Spawn Settings")]
    public GameObject vesselPrefab;
    public int spawnBatchSize = 10;
    public float spawnInterval = 1.0f; // Time between batches in seconds
    
    [Header("Spread Settings")]
    [Tooltip("Base radius for the first batch.")]
    public float baseSpreadRadius = 20f;
    [Tooltip("How much the spawn radius expands per batch to prevent crowding.")]
    public float radiusExpansionPerBatch = 5f;

    private List<GameObject> activeVessels = new List<GameObject>();
    
    // FPS Tracking Variables
    private float timer;
    private int frameCountInPeriod;
    private float deltaTimeSumInPeriod;
    private int batchCount = 0;

    void Start()
    {
        ResetPeriodCounters();
    }

    void Update()
    {
        // Track performance data for the current frame
        frameCountInPeriod++;
        deltaTimeSumInPeriod += Time.unscaledDeltaTime; // Using unscaled to ensure accuracy even if Time.timeScale changes
        
        timer += Time.unscaledDeltaTime;
        if (timer >= spawnInterval)
        {
            // Calculate true average FPS for this specific period
            float averageFps = frameCountInPeriod / deltaTimeSumInPeriod;
            
            // Output data before spawning the next batch
            Debug.Log($"[BENCHMARK] Vessels: {activeVessels.Count} | Avg FPS over last {spawnInterval}s: {averageFps:F1}");

            // Spawn the new batch and step up the generation metrics
            SpawnBatch();
            batchCount++;
            
            // Reset counters for the next tracking window
            ResetPeriodCounters();
        }
    }

    void SpawnBatch()
    {
        // Dynamic boundary: Radius expands as more batches enter the simulation
        float currentRadius = baseSpreadRadius + (batchCount * radiusExpansionPerBatch);

        for (int i = 0; i < spawnBatchSize; i++)
        {
            // Uniform distribution in a circle
            float angle = Random.Range(0f, Mathf.PI * 2);
            // Using square root for uniform distribution across the circle area (prevents bunching in the center)
            float radius = currentRadius * Mathf.Sqrt(Random.Range(0f, 1f)); 
            
            float x = radius * Mathf.Cos(angle);
            float z = radius * Mathf.Sin(angle);
            
            Vector3 spawnPos = new Vector3(x, 0f, z) + transform.position;
            
            GameObject vessel = Instantiate(vesselPrefab, spawnPos, Quaternion.identity);
            activeVessels.Add(vessel);
        }
    }

    void ResetPeriodCounters()
    {
        timer = 0f;
        frameCountInPeriod = 0;
        deltaTimeSumInPeriod = 0f;
    }
}
