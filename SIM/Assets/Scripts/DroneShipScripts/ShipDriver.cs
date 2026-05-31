using System;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class AdvancedShipController : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public Transform rudderTransform;
    public Transform propellerMount;

    [Header("Engine & Rudder")]
    public float motorPower = 5000f;
    public float maxRudderAngle = 35f;

    [Header("Hydrodynamics")]
    [Tooltip("How much the prop blast affects the rudder even at low speeds.")]
    public float propWashEfficiency = 0.6f;
    [Tooltip("The 'Bite' of the rudder.")]
    public float liftCoefficient = 1.5f;
    [Tooltip("How much the ship slows down during a turn.")]
    public float dragCoefficient = 0.3f;

    public float currentRudderAngle = 0f;


    public float throttle = 0.0f;
    public float steerInput = 0.0f;

    void FixedUpdate()
    {
        // float throttle = Input.GetAxis("Vertical");
        // float steerInput = Input.GetAxis("Horizontal");

        // 1. Rudder Rotation (Physical Speed)
        float targetAngle = -steerInput * maxRudderAngle;
        currentRudderAngle = targetAngle;
        rudderTransform.localRotation = Quaternion.Euler(0, currentRudderAngle, 0);

        // 2. Propulsion
        Vector3 propulsionForce = propellerMount.forward * throttle * motorPower;
        rb.AddForceAtPosition(propulsionForce, propellerMount.position, ForceMode.Force);

        // 3. Advanced Rudder Physics
        ApplyRudderForces(throttle);
    }

    void ApplyRudderForces(float throttle)
    {
        // Get the velocity of the water at the rudder's position
        Vector3 localVelocity = transform.InverseTransformDirection(rb.GetPointVelocity(rudderTransform.position));
        
        // Prop Wash: Increase 'effective' water speed if the engine is spinning
        float engineSpeedFactor = Mathf.Max(0, throttle) * (motorPower / rb.mass) * propWashEfficiency;
        float effectiveForwardSpeed = localVelocity.z + engineSpeedFactor;

       

        if (Mathf.Abs(effectiveForwardSpeed) < 0.1f) return;

        // Lift Calculation: Force = Speed^2 * LiftCoeff * Sin(Angle)
        // We use the rudder's right vector to determine the direction of the push
        float rudderAngleRad = currentRudderAngle * Mathf.Deg2Rad;
        
        // LIFT (The turning force)
        float liftMagnitude = (effectiveForwardSpeed * effectiveForwardSpeed) * liftCoefficient * Mathf.Sin(rudderAngleRad) * Math.Sign(effectiveForwardSpeed);
        Vector3 liftForce = rudderTransform.right * liftMagnitude;

        // DRAG (The slowing force - induced by the rudder being sideways)
        float dragMagnitude = (effectiveForwardSpeed * effectiveForwardSpeed) * dragCoefficient * Mathf.Abs(Mathf.Sin(rudderAngleRad)) * Math.Sign(effectiveForwardSpeed);
        Vector3 dragForce = -transform.forward * dragMagnitude * Mathf.Sign(effectiveForwardSpeed);

        // Apply forces to the Rigidbody
        rb.AddForceAtPosition(liftForce + dragForce, rudderTransform.position, ForceMode.Force);
    }
}