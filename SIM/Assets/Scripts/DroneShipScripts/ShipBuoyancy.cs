using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ShipBuoyancy : MonoBehaviour
{
    [Header("References")]

    public WaterSystem water;

    public Transform[] floaters;
    public Transform centerOfMass;
    public Transform scalableObj;

    [Header("Buoyancy Settings")]
    public float buoyancyForce = 20f;
    public float buoyancyDamper = 5f; // Stops the "bouncing"
    public float normalDragForce = 5f; 

    public float forceLimiter = 10.0f;

    public float verticalWaterDrag = 1.0f;


    [Header("Stability")]
    public float waterDrag = 1f; // Stops the "sliding"
    public float waterAngularDrag = 2f; // Stops the "tipping/rolling"

    public Vector3 CM = Vector3.zero;


    [Header("Keel Settings (Sideways Resistance)")]
    [Tooltip("X-axis: 0 (Forward) to 1 (90 degrees sideways). Y-axis: Drag Multiplier.")]
    public AnimationCurve keelDragCurve = AnimationCurve.Linear(0, 0.1f, 1, 1.0f);
    public float keelStrength = 5.0f; // Overall multiplier for the sideways grip


    [Header("Debug")]
    public bool renderFloaters = true;
    public bool drawGizmos = true;

    private Rigidbody rb;


    [HideInInspector] public bool managedExternally = true;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        

        water = Object.FindAnyObjectByType<WaterSystem>();

        if (water == null)
        {
            Debug.LogWarning("WaterSystem component not found in the scene.");
        }

        float scale = scalableObj.transform.localScale.x;
        CM.y = -0.3f * scale; 
        CM.z = -scale / 2.0f;

        rb.automaticCenterOfMass = false;
        rb.centerOfMass = CM;
        
    }



    void FixedUpdate()
    {
        if (!managedExternally){
            BuoyancyPhysicsUpdate();
        }
    }



    public void BuoyancyPhysicsUpdate()
    { 
        if (water == null || floaters.Length == 0) return;

       


        int submergedCount = 0;

        float maxDepth = 1.0f * scalableObj.localScale.y; 
        

        foreach (Transform floater in floaters)
        {
            float seaHeight = water.GetWaterHeight(floater.position);
            float depth = seaHeight - floater.position.y;

            if (depth > 0)
            {
                submergedCount++;
                
                // Use a fixed maxDepth for all floaters to ensure symmetry
                float actualMaxDepth = 0.3f; 
                float submergence = Mathf.Clamp01(depth / actualMaxDepth);
                
                // BUOYANCY: Clean linear push
                // Removed displacement * submergence (which was depth^2)
                Vector3 buoyancy = Vector3.up * (buoyancyForce * submergence);

                // DAMPING: Only applied to the vertical velocity of the floater
                Vector3 velocityAtPoint = rb.GetPointVelocity(floater.position);
                float verticalVelo = velocityAtPoint.y;
                Vector3 damping = Vector3.down * (verticalVelo * buoyancyDamper * submergence);

               // 2. IMPROVED NORMAL (SURFING) FORCE
                // Get the direction the water surface is "tilting"
                Vector3 surfaceNormal = water.GetNormal(floater.position);
                
                // We project the gravity vector onto the water plane to find the "Downhill" direction
                // This makes the ship naturally want to slide down the face of a wave.
                Vector3 gravityProjection = Vector3.ProjectOnPlane(Vector3.down, surfaceNormal);
                Vector3 waveSlideForce = -gravityProjection * normalDragForce * submergence;

                Vector3 totalForce = buoyancy + damping + waveSlideForce;

                // Apply clamping to prevent physics 'explosions'
                totalForce = Vector3.ClampMagnitude(totalForce, forceLimiter);
                
                rb.AddForceAtPosition(totalForce, floater.transform.position, ForceMode.Force);
            }
        }

        // 3. Global Stability: Apply drag to the whole body if touching water
        if (submergedCount > 0)
        {
            // Counter-act movement and rotation proportional to the ship's mass/volume
            rb.AddForce(-rb.velocity * waterDrag , ForceMode.Force);
            rb.AddTorque(-rb.angularVelocity * waterAngularDrag , ForceMode.Force);

            // vertical water drag when submerged to prevent jumping from water
            Vector3 v = rb.velocity;
            if(v.y > 0)
            {
                Vector3 verticalDrag = new Vector3(0, -v.y, 0) * verticalWaterDrag;
                rb.AddForce(verticalDrag , ForceMode.Force);
            }
        }


       // ApplyKeelDrag();


        



    }



    public void ApplyKeelDrag()
    {
        // 1. Get velocity in local space
        Vector3 localVel = transform.InverseTransformDirection(rb.velocity);
        
        // Ignore vertical movement for keel calculation
        Vector3 planarVel = new Vector3(localVel.x, 0, localVel.z);
        if (planarVel.sqrMagnitude < 0.01f) return;

        // 2. Calculate the 'Slip Angle' (0 degrees = forward, 90 degrees = full sideways)
        float dot = Vector3.Dot(planarVel.normalized, Vector3.forward);
        float slipFactor = 1f - Mathf.Abs(dot); // 0 at forward, 1 at sideways

        // 3. Evaluate the curve for the 'Keel Grip'
        float dragMultiplier = keelDragCurve.Evaluate(slipFactor);

        // 4. Calculate forces
        // We apply a massive force against the sideways component of velocity
        Vector3 sidewaysVelocity = transform.right * localVel.x;
        Vector3 keelForce = -sidewaysVelocity * dragMultiplier * keelStrength;

        // Apply as Acceleration to keep it "stiff" regardless of mass
        rb.AddForce(keelForce, ForceMode.Acceleration);
    }

    private void OnDrawGizmos()
    {
        if (rb != null && drawGizmos)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(transform.TransformPoint(rb.centerOfMass), 0.2f * transform.localScale.x);
        }
    }
}