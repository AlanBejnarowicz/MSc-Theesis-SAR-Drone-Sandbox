using UnityEngine;

public class WindSimulator : MonoBehaviour
{
    [Header("Base Wind")]
    public float baseWindStrength = 5f;
    public Vector3 windDirection = Vector3.forward;

    [Header("Gusts")]
    public float gustStrength = 10f;
    public float gustInterval = 3f;
    public float gustDuration = 0.5f;

    [Header("Settings")]
    public LayerMask shipLayer;

    private float _gustTimer = 0f;
    private float _gustActiveTimer = 0f;
    public bool _isGusting = false;
    private Rigidbody[] _shipRigidbodies;

    void Start()
    {
        RefreshShipList();
    }

    void FixedUpdate()
    {
        HandleGustTimer();
        ApplyWind();
    }

    private void HandleGustTimer()
    {
        if (_isGusting)
        {
            _gustActiveTimer -= Time.fixedDeltaTime;
            if (_gustActiveTimer <= 0f)
                _isGusting = false;
        }
        else
        {
            _gustTimer -= Time.fixedDeltaTime;
            if (_gustTimer <= 0f)
            {
                _isGusting = true;
                _gustActiveTimer = gustDuration;
                _gustTimer = gustInterval + Random.Range(-1f, 1f); // slight randomness
                windDirection = Quaternion.Euler(0, Random.Range(-30f, 30f), 0) * windDirection; // shift direction on gust
            }
        }
    }

    private void ApplyWind()
    {
        float strength = (baseWindStrength/100.0f) + (_isGusting ? (gustStrength / 100.0f) : 0f);
        Vector3 force = windDirection.normalized * strength;

        foreach (Rigidbody rb in _shipRigidbodies)
        {
            if (rb != null)
                rb.AddForce(force, ForceMode.Acceleration);
        }
    }

    // Call this if ships are spawned/destroyed at runtime
    public void RefreshShipList()
    {
        GameObject[] shipObjects = GameObject.FindObjectsByType<GameObject>(FindObjectsSortMode.None);
        System.Collections.Generic.List<Rigidbody> rbs = new();

        foreach (GameObject go in shipObjects)
        {
            if (((1 << go.layer) & shipLayer) != 0)
            {
                Rigidbody rb = go.GetComponent<Rigidbody>();
                if (rb != null) rbs.Add(rb);
            }
        }

        _shipRigidbodies = rbs.ToArray();
    }
}