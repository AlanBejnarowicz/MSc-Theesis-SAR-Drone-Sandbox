using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vCOMPASS : MonoBehaviour
{
[Header("Settings")]
    public Transform shipTransform;
    public float samplingRate = 20f; // Compasses are usually slower than IMUs
    public float headingNoiseSigma = 1.5f; // Degrees of jitter

    [Header("Output")]
    [Tooltip("0 = North (+Z), 90 = East (+X), 180 = South (-Z), 270 = West (-X)")]
    public float currentHeading;

    private float _timer;



    void Start()
    {
        if (shipTransform == null)
            shipTransform = this.transform;
    }

    void FixedUpdate()
    {
        _timer += Time.fixedDeltaTime;
        if (_timer >= (1f / samplingRate))
        {
            UpdateCompass();
            _timer = 0f;
        }
    }

    private void UpdateCompass()
    {
        Vector3 forward = shipTransform.forward;
        float angle = Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
        if (angle < 0) angle += 360f;

        // Add noise then re-wrap to keep 0-360
        float noisy = AddGaussianNoise(angle, headingNoiseSigma);
        currentHeading = (noisy % 360f + 360f) % 360f;
    }

    private float AddGaussianNoise(float value, float sigma)
    {
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return value + (sigma * randStdNormal);
    }
}