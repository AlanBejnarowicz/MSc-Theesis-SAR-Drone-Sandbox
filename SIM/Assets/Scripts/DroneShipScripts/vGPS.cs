using UnityEngine;

public class vGPS : MonoBehaviour
{
    [Header("References")]
    public Transform shipTransform;
    public Transform originTransform; // The GPS_CORD_SYSTEM object

    [Header("Origin Geodetic Coordinates")]
    public double originLatitude = 54.3718;  // Example: Gdansk, Poland
    public double originLongitude = 18.6127;

    [Header("Sensor Settings")]
    public float updateRate = 5f; // GPS is typically slow (5-10Hz)
    public float positionalNoiseSigma = 0.5f; // meters

    [Header("Current Reading (Output)")]
    public double currentLat;
    public double currentLon;

    private float _timer;
    private const double EarthRadius = 6378137.0; // WGS84 Major Axis in meters


    void Start()
    {
        shipTransform = this.gameObject.transform;

        originTransform = GameObject.Find("GPS_CORD_ORIGIN").transform;

    }
    

    void FixedUpdate()
    {
        _timer += Time.fixedDeltaTime;
        if (_timer >= (1f / updateRate))
        {
            UpdateGPS();
            _timer = 0f;
        }
    }

    private void UpdateGPS()
    {
        // 1. Calculate Cartesian offset from the GPS_CORD_SYSTEM origin
        Vector3 offset = shipTransform.position - originTransform.position;

        // 2. Add GPS Accuracy Noise
        offset.x += RandomGaussian(0, positionalNoiseSigma);
        offset.z += RandomGaussian(0, positionalNoiseSigma);

        // 3. Convert Meters to Decimal Degrees
        // Latitude: 1 degree is approx 111,111 meters
        double deltaLat = offset.z / 111111.0;

        // Longitude: Degree length shrinks as you move away from the equator
        double latInRadians = originLatitude * Mathf.Deg2Rad;
        double metersPerDegreeLon = 111111.0 * System.Math.Cos(latInRadians);
        double deltaLon = offset.x / metersPerDegreeLon;

        // 4. Update Final Coordinates
        currentLat = originLatitude + deltaLat;
        currentLon = originLongitude + deltaLon;
    }

    private float RandomGaussian(float mean, float stdDev)
    {
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;
        return mean + stdDev * Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
    }
}