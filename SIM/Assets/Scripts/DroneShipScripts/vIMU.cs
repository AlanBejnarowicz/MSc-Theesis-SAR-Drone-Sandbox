using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vIMU : MonoBehaviour
{
[Header("References")]
    public Rigidbody shipRigidbody;
    
    [Header("Sensor Parameters")]
    public Vector3 accelerometerNoiseSigma = new Vector3(0.05f, 0.05f, 0.05f);
    public Vector3 gyroNoiseSigma = new Vector3(0.01f, 0.01f, 0.01f);
    
    public Vector3 AccelReadings { get; private set; }
    public Vector3 GyroReadings { get; private set; }

    private Vector3 _lastVelocity;
    private float _timer;

    void FixedUpdate()
    {

            UpdateIMU();

    }

    private void UpdateIMU()
    {
        // 1. Calculate Linear Acceleration in World Space
        // a = (v_current - v_last) / dt
        Vector3 currentVelocity = shipRigidbody.velocity;
        Vector3 worldAcceleration = (currentVelocity - _lastVelocity) / Time.fixedDeltaTime;
        _lastVelocity = currentVelocity;

        // 2. Account for "Proper Acceleration" (Add Gravity)
        // Accelerometers measure acceleration relative to free-fall
        Vector3 specificForceWorld = worldAcceleration - Physics.gravity;

        // 3. Transform to Local Sensor Frame
        // This simulates the orientation of the physical IMU chip
        Vector3 localAccel = transform.InverseTransformDirection(specificForceWorld);
        Vector3 localGyro = transform.InverseTransformDirection(shipRigidbody.angularVelocity);

        // 4. Apply Stochastic Noise (Gaussian/White Noise)
        AccelReadings = AddNoise(localAccel, accelerometerNoiseSigma);
        GyroReadings = AddNoise(localGyro, gyroNoiseSigma);
    }

    private Vector3 AddNoise(Vector3 signal, Vector3 sigma)
    {
        return new Vector3(
            signal.x + RandomGaussian(0, sigma.x),
            signal.y + RandomGaussian(0, sigma.y),
            signal.z + RandomGaussian(0, sigma.z)
        );
    }

    private float RandomGaussian(float mean, float stdDev)
    {
        // Box-Muller transform for generating normal distribution
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }
}