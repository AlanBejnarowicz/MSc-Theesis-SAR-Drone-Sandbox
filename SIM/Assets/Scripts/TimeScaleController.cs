using UnityEngine;

public class TimeScaleController : MonoBehaviour
{
    [Header("Settings")]
    [SerializeField] private KeyCode toggleKey = KeyCode.T;

    private readonly float[] _scales      = { 1f, 2f, 5f, 10f, 25f };
    private readonly string[] _labels     = { "1×", "2×", "5×", "10×", "25×" };
    private int   _selected               = 0;
    private bool  _visible                = true;

    // Layout
    private const float PanelW  = 320f;
    private const float PanelH  = 80f;
    private const float Padding = 12f;
    private const float BtnH    = 44f;
    private float       BtnW    => (PanelW - Padding * 2 - 4 * 8) / 5f;

    // Styles (built once in OnGUI)
    private GUIStyle _panelStyle;
    private GUIStyle _btnNormal;
    private GUIStyle _btnActive;
    private GUIStyle _labelStyle;
    private bool     _stylesReady;

    // ------------------------------------------------------------------ //

    void Start() => ApplyScale(_selected);

    void Update()
    {
        if (Input.GetKeyDown(toggleKey))
            _visible = !_visible;

        // Keyboard shortcuts: 1-4
        for (int i = 0; i < _scales.Length; i++)
            if (Input.GetKeyDown(KeyCode.Alpha1 + i))
                SelectScale(i);
    }

    // ------------------------------------------------------------------ //

    void OnGUI()
    {
        if (!_visible) return;
        BuildStyles();

        float x = Screen.width  - PanelW - 12f;
        float y = 12f;

        // Panel shadow
        GUI.color = new Color(0, 0, 0, 0.35f);
        GUI.Box(new Rect(x + 3, y + 3, PanelW, PanelH), GUIContent.none, _panelStyle);
        GUI.color = Color.white;

        // Panel background
        GUI.Box(new Rect(x, y, PanelW, PanelH), GUIContent.none, _panelStyle);

        // "SIM SPEED" label
        GUI.Label(new Rect(x + Padding, y + 4f, 80f, 16f), "SIM SPEED", _labelStyle);

        // Buttons
        float bx = x + Padding;
        float by = y + PanelH - BtnH - Padding * 0.5f;

        for (int i = 0; i < _scales.Length; i++)
        {
            GUIStyle style = (i == _selected) ? _btnActive : _btnNormal;
            if (GUI.Button(new Rect(bx, by, BtnW, BtnH), _labels[i], style))
                SelectScale(i);

            bx += BtnW + 8f;
        }
    }

    // ------------------------------------------------------------------ //

    private void SelectScale(int index)
    {
        _selected = index;
        ApplyScale(index);
    }

    private void ApplyScale(int index)
    {
        float s = _scales[index];
        Time.timeScale      = s;
        Time.fixedDeltaTime = 0.01f; // always 50Hz, Unity runs more steps automatically
        Debug.Log($"[TimeScale] {s}×");
    }

    // ------------------------------------------------------------------ //

    private void BuildStyles()
    {
        if (_stylesReady) return;
        _stylesReady = true;

        // Panel
        _panelStyle            = new GUIStyle(GUI.skin.box);
        _panelStyle.normal.background = MakeTex(4, 4, new Color(0.08f, 0.08f, 0.10f, 0.92f));

        // Label
        _labelStyle            = new GUIStyle(GUI.skin.label);
        _labelStyle.fontSize   = 11;
        _labelStyle.fontStyle  = FontStyle.Bold;
        _labelStyle.normal.textColor = new Color(0.5f, 0.7f, 1f, 0.85f);

        // Normal button
        _btnNormal                       = new GUIStyle(GUI.skin.button);
        _btnNormal.fontSize              = 16;
        _btnNormal.fontStyle             = FontStyle.Bold;
        _btnNormal.normal.background     = MakeTex(4, 4, new Color(0.18f, 0.20f, 0.26f, 1f));
        _btnNormal.hover.background      = MakeTex(4, 4, new Color(0.25f, 0.28f, 0.38f, 1f));
        _btnNormal.normal.textColor      = new Color(0.75f, 0.82f, 1f, 1f);
        _btnNormal.hover.textColor       = Color.white;
        _btnNormal.border                = new RectOffset(3, 3, 3, 3);

        // Active button
        _btnActive                       = new GUIStyle(_btnNormal);
        _btnActive.normal.background     = MakeTex(4, 4, new Color(0.25f, 0.55f, 1f, 1f));
        _btnActive.hover.background      = MakeTex(4, 4, new Color(0.30f, 0.62f, 1f, 1f));
        _btnActive.normal.textColor      = Color.white;
        _btnActive.hover.textColor       = Color.white;
    }

    private static Texture2D MakeTex(int w, int h, Color col)
    {
        var pix = new Color[w * h];
        for (int i = 0; i < pix.Length; i++) pix[i] = col;
        var t = new Texture2D(w, h);
        t.SetPixels(pix);
        t.Apply();
        return t;
    }
}