using UnityEngine;

public class FleetCamera : MonoBehaviour
{
    [Header("Targeting")]
    public Transform currentTarget;
    public LayerMask shipLayer;
    public LayerMask waterLayer;
    
    [Header("Dynamic Rotation")]
    public float minPitch = 20f;  // Angle when zoomed in (looking at horizon)
    public float maxPitch = 85f;  // Angle when zoomed out (top-down)
    public float pitchSmoothness = 5f;

    [Header("Movement")]
    public float keyboardSpeed = 30f;
    public float rotationSpeed = 120f;
    public float scrollSpeed = 20f;
    public float smoothTime = 0.15f;

    [Header("Zoom")]
    public float minZoom = 5f;
    public float maxZoom = 150f;

    [Header("Drag Settings")]
    public float dragSensitivity = 1.5f;
    private Vector3 _lastMousePosition;

    private float _currentZoom = 30f;
    private float _currentYaw = 0f;    // Horizontal rotation (Y-axis)
    private float _targetPitch = 30f;  // Vertical rotation (X-axis)
    private Vector3 _panOffset = Vector3.zero;
    private Vector3 _moveVelocity = Vector3.zero;

    void LateUpdate()
    {
        // 1. Inputs (Click, Rotate, Pan)
        HandleInputs();

        // 2. Calculate Dynamic Pitch based on Zoom
        // Normalize zoom (0 to 1) and map it to our pitch range
        float zoomPercent = Mathf.InverseLerp(minZoom, maxZoom, _currentZoom);
        _targetPitch = Mathf.Lerp(minPitch, maxPitch, zoomPercent);

        // 3. Position Calculation
        if (currentTarget != null || _panOffset != Vector3.zero)
        {
            // Create rotation from Yaw (User) and Pitch (Calculated)
            Quaternion rotation = Quaternion.Euler(_targetPitch, _currentYaw, 0);
            Vector3 zoomVector = new Vector3(0, 0, -_currentZoom);
            
            Vector3 focalPoint = (currentTarget != null) ? currentTarget.position + _panOffset : _panOffset;
            Vector3 desiredPosition = focalPoint + rotation * zoomVector;
            
            transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref _moveVelocity, smoothTime);
            transform.LookAt(focalPoint + Vector3.up * 1.5f);
        }
    }



    private void HandleInputs()
    {
        // --- 1. LEFT CLICK (Select Ship or Detach via Water) ---
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, 10000f))
            {
                if (hit.transform.CompareTag("Ship"))
                {
                    currentTarget = hit.transform;
                    _panOffset = Vector3.zero; // Snap to ship center
                }
                else if (((1 << hit.transform.gameObject.layer) & waterLayer) != 0)
                {
                    DetachCamera();
                }
            }
        }



        // --- 2. KEYBOARD PANNING (WSAD) ---
        float h = Input.GetAxis("Horizontal"); // A, D
        float v = Input.GetAxis("Vertical");   // W, S

        if (Mathf.Abs(h) > 0.01f || Mathf.Abs(v) > 0.01f)
        {
            // Scale speed smoothly between min and max zoom
            float zoomPercent = Mathf.InverseLerp(minZoom, maxZoom, _currentZoom);
            float dynamicSpeed = Mathf.Lerp(keyboardSpeed, keyboardSpeed * 5f, zoomPercent);

            Quaternion yawRotation = Quaternion.Euler(0f, _currentYaw, 0f);
            Vector3 forward = yawRotation * Vector3.forward;
            Vector3 right = yawRotation * Vector3.right;

            _panOffset += (forward * v + right * h) * dynamicSpeed * Time.deltaTime;
        }

        // --- 3. RIGHT CLICK (Rotate Yaw) ---
        if (Input.GetMouseButton(1))
        {
            _currentYaw += Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
        }

        // --- 4. SCROLL (Zoom) ---
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        _currentZoom = Mathf.Clamp(_currentZoom - scroll * scrollSpeed * 100f * Time.deltaTime, minZoom, maxZoom);
    }

    
    private void DetachCamera()
    {
        if (currentTarget != null)
        {
            // Maintain position so camera doesn't jump to (0,0,0)
            _panOffset += currentTarget.position;
            currentTarget = null;
            Debug.Log("Camera Detached via Water Layer click.");
        }
    }
  
}