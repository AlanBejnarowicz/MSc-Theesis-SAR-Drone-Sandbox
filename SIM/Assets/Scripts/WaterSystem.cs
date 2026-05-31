using UnityEngine;


[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class WaterSystem : MonoBehaviour
{
    [Header("Settings")]
    public Transform targetCamera;
    public float waveAmplitudeScale = 1.0f;
    public float gridSnap = 1.0f;

    [Header("Shader Properties")]
    public Material waterMaterial;
    private int _secondsID;
    private int _AmplitudeScale;


    [Header("Mesh Settings")]
    public int gridResolution = 100; // Number of vertices per side
    public float spacing = 1.0f;     // Distance between vertices
    private Mesh mesh;



    private void Start()
    {
        _secondsID = Shader.PropertyToID("_Seconds");
        _AmplitudeScale = Shader.PropertyToID("_AmplitudeScale");

        if (targetCamera == null) targetCamera = Camera.main.transform;

        GenerateMesh();
        ApplyMaterial();
    }

    private void Update()
    {
        // Project camera forward onto XZ plane to shift mesh ahead of camera
        Vector3 camForward = targetCamera.forward;
        camForward.y = 0f;
        camForward.Normalize();

        // How far to push the mesh center forward (half the mesh size)
        float meshHalfSize = (gridResolution * spacing) * 0.9f;

        Vector3 newPos = targetCamera.position + camForward * meshHalfSize * 0.5f;
        float x = Mathf.Floor(newPos.x / gridSnap) * gridSnap;
        float z = Mathf.Floor(newPos.z / gridSnap) * gridSnap;
        transform.position = new Vector3(x, 0, z);

        if (waterMaterial != null)
        {
            waterMaterial.SetFloat(_secondsID, Time.time);
            waterMaterial.SetFloat(_AmplitudeScale, waveAmplitudeScale);
        }
    }

    // --- PHYSICS ACCESSIBLE FUNCTIONS ---

    public float GetWaterHeight(Vector3 worldPos)
    {
        return CalculateTotalWaveHeight(worldPos);
    }

    private float CalculateWave(Vector3 pos, float freq, float speed, float amp)
    {
        // Matches your C++ math exactly
        return Mathf.Sin(pos.x * freq + Time.time * speed) * Mathf.Cos(pos.z * freq * 0.8f + Time.time * speed) * amp;
    }

    private float CalculateTotalWaveHeight(Vector3 pos)
    {
        // Large Swells
        float w1 = CalculateWave(pos, 0.02f, 1.2f, 0.8f * waveAmplitudeScale); 
        float w2 = CalculateWave(pos, 0.04f, 1.5f, 0.5f * waveAmplitudeScale);

        // Mid-level Turbulence
        float w3 = CalculateWave(pos, 0.10f, 2.2f, 0.2f * waveAmplitudeScale);
        float w4 = CalculateWave(pos, 0.18f, 1.8f, 0.15f * waveAmplitudeScale);

        // Surface Micro-Ripples
        float w5 = CalculateWave(pos, 0.45f, 3.0f, 0.05f * waveAmplitudeScale);
        float w6 = CalculateWave(pos, 0.80f, 4.2f, 0.02f * waveAmplitudeScale);

        return w1 + w2 + w3 + w4 + w5 + w6;
    }

    public Vector3 GetNormal(Vector3 position)
    {
        float offset = 0.1f;
        float hL = GetWaterHeight(position + Vector3.left * offset);
        float hR = GetWaterHeight(position + Vector3.right * offset);
        float hB = GetWaterHeight(position + Vector3.back * offset);
        float hF = GetWaterHeight(position + Vector3.forward * offset);

        // Calculate the slope based on height differences
        return new Vector3(hL - hR, 2.0f * offset, hB - hF).normalized;
    }

    void GenerateMesh()
    {
        mesh = new Mesh();
        mesh.name = "ProceduralSea";

        int vertexCount = gridResolution * gridResolution;
        Vector3[] vertices = new Vector3[vertexCount];
        Vector2[] uvs = new Vector2[vertexCount];
        int[] triangles = new int[(gridResolution - 1) * (gridResolution - 1) * 6];

        float offset = (gridResolution - 1) * spacing * 0.5f;

        for (int z = 0; z < gridResolution; z++)
        {
            for (int x = 0; x < gridResolution; x++)
            {
                int index = z * gridResolution + x;
                // Center the mesh around its local origin
                vertices[index] = new Vector3(x * spacing - offset, 0, z * spacing - offset);
                uvs[index] = new Vector2((float)x / gridResolution, (float)z / gridResolution);
            }
        }

        int triIndex = 0;
        for (int z = 0; z < gridResolution - 1; z++)
        {
            for (int x = 0; x < gridResolution - 1; x++)
            {
                int root = z * gridResolution + x;
                triangles[triIndex++] = root;
                triangles[triIndex++] = root + gridResolution;
                triangles[triIndex++] = root + 1;

                triangles[triIndex++] = root + 1;
                triangles[triIndex++] = root + gridResolution;
                triangles[triIndex++] = root + gridResolution + 1;
            }
        }

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds(); // Important for culling

        GetComponent<MeshFilter>().mesh = mesh;
    }


    void ApplyMaterial()
    {
        if (waterMaterial != null)
        {
            GetComponent<MeshRenderer>().material = waterMaterial;
        }
        else
        {
            Debug.LogWarning("Water Material not assigned to WaterSystem!");
        }
    }

}