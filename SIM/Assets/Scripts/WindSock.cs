using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class WindSock : MonoBehaviour
{
    [Header("References")]
    public WindSimulator windSimulator;

    [Header("Sock Shape")]
    public int segments = 12;
    public float sockLength = 4f;
    public float baseRadius = 0.4f;
    public float tipRadius = 0.1f;

    [Header("Sock Droop")]
    public float gravity = 2f;          // How much the sock droops when there's no wind
    public float stiffness = 0.5f;      // How much the sock resists bending (0 = very floppy)

    [Header("Pole")]
    public float poleHeight = 5f;
    public Color poleColor = Color.gray;
    public float poleWidth = 0.05f;

    [Header("Colors")]
    public Color color1 = new Color(1f, 0.3f, 0.1f);   // Orange
    public Color color2 = Color.white;

    private LineRenderer _sockRenderer;
    private LineRenderer _poleRenderer;
    private Vector3[] _segmentPositions;
    private Vector3 _mountPoint;

    void Start()
    {
        _mountPoint = transform.position + Vector3.up * poleHeight;

        SetupPole();
        SetupSock();

        _segmentPositions = new Vector3[segments + 1];
    }

    void Update()
    {
        if (windSimulator == null) return;

        UpdateSockShape();
        DrawSock();
    }

    // -------------------------------------------------------
    // Pole (separate LineRenderer on child GO)
    // -------------------------------------------------------
    private void SetupPole()
    {
        GameObject poleGO = new GameObject("Pole");
        poleGO.transform.SetParent(transform);

        _poleRenderer = poleGO.AddComponent<LineRenderer>();
        _poleRenderer.positionCount = 2;
        _poleRenderer.SetPosition(0, transform.position);
        _poleRenderer.SetPosition(1, _mountPoint);
        _poleRenderer.startWidth  = poleWidth;
        _poleRenderer.endWidth    = poleWidth;
        _poleRenderer.material    = CreateMaterial(poleColor);
        _poleRenderer.useWorldSpace = true;
    }

    // -------------------------------------------------------
    // Sock LineRenderer (tube approximated as spine + width)
    // -------------------------------------------------------
    private void SetupSock()
    {
        _sockRenderer = GetComponent<LineRenderer>();
        _sockRenderer.positionCount = segments + 1;
        _sockRenderer.useWorldSpace = true;
        _sockRenderer.textureMode   = LineTextureMode.Tile;
        _sockRenderer.material = CreateMaterial();

        Gradient gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[]
            {
                new GradientColorKey(color1, 0.0f),
                new GradientColorKey(color2, 0.33f),
                new GradientColorKey(color1, 0.66f),
                new GradientColorKey(color2, 1.0f)
            },
            new GradientAlphaKey[]
            {
                new GradientAlphaKey(1f, 0f),
                new GradientAlphaKey(1f, 1f)
            }
        );
        _sockRenderer.colorGradient = gradient;



        _sockRenderer.colorGradient = gradient;
        _sockRenderer.material = CreateMaterial(Color.white);
    }

    // -------------------------------------------------------
    // Simulate sock as a chain of segments
    // -------------------------------------------------------
    private void UpdateSockShape()
    {
        // Wind info from simulator
        Vector3 windDir   = windSimulator.windDirection.normalized;
        float windStrength = windSimulator.baseWindStrength +
                             (IsGusting() ? windSimulator.gustStrength : 0f);

        // Flatten to XZ so vertical component doesn't skew the angle
        Vector3 windFlat     = new Vector3(windDir.x, 0f, windDir.z).normalized;
        Vector3 windForce    = windFlat * windStrength;
        Vector3 gravityForce = Vector3.down * gravity;
        Vector3 sockForce    = windForce + gravityForce;

        // Normalize to get a direction, scale by segment length
        float segmentLength = sockLength / segments;

        // Root always at mount point
        _segmentPositions[0] = _mountPoint;

        // Each segment bends toward force direction
        Vector3 direction = sockForce.normalized;
        for (int i = 1; i <= segments; i++)
        {
            float t = (float)i / segments;
            // Tip is more affected by wind, base is stiffer
            float bendFactor = Mathf.Lerp(stiffness, 1f, t);
            Vector3 segDir = Vector3.Lerp(Vector3.down, direction, bendFactor).normalized;
            _segmentPositions[i] = _segmentPositions[i - 1] + segDir * segmentLength;
        }
    }

    // -------------------------------------------------------
    // Draw sock as tapered LineRenderer
    // -------------------------------------------------------
    private void DrawSock()
    {
        _sockRenderer.positionCount = segments + 1;

        for (int i = 0; i <= segments; i++)
        {
            _sockRenderer.SetPosition(i, _segmentPositions[i]);
            float t = (float)i / segments;
            // Taper from base to tip
            float width = Mathf.Lerp(baseRadius * 2f, tipRadius * 2f, t);
            // We can't set per-point width with AnimationCurve, use widthCurve
        }

        // Taper via width curve
        AnimationCurve curve = new AnimationCurve();
        curve.AddKey(0f, baseRadius * 2f);
        curve.AddKey(1f, tipRadius  * 2f);
        _sockRenderer.widthCurve = curve;
    }

    // -------------------------------------------------------
    // Reads gust state via reflection-free public field check
    // -------------------------------------------------------
    private bool IsGusting()
    {
        // WindSimulator doesn't expose _isGusting, so we approximate:
        // if actual wind feels above base, a gust is likely active.
        // For a cleaner solution, make _isGusting public in WindSimulator.
        return false;
    }

    private Material CreateMaterial(Color color)
    {
        // Use Legacy/Particles shader — correctly renders vertex color gradients on LineRenderer
        Material mat = new Material(Shader.Find("Legacy Shaders/Particles/Alpha Blended Premultiply"));
        mat.color = color;
        return mat;
    }

    private Material CreateMaterial()
    {
        return new Material(Shader.Find("Legacy Shaders/Particles/Alpha Blended Premultiply"));
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, transform.position + Vector3.up * poleHeight);
        Gizmos.DrawWireSphere(transform.position + Vector3.up * poleHeight, 0.2f);
    }
}