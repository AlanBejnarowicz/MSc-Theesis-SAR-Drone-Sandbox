using System;
using System.Collections.Generic;
using System.Net;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Attach to any persistent GameObject (e.g. GameManager).
/// Starts a lightweight HTTP server on localhost:8765.
/// 
/// Endpoints:
///   GET  /vessels   → JSON array of all registered AISTransponders
///   GET  /health    → {"status":"ok"}
///   OPTIONS *       → CORS preflight (needed by browser fetch)
/// </summary>
public class AISHttpServer : MonoBehaviour
{
    [Header("Server Settings")]
    public int    port           = 8765;
    public float  broadcastHz    = 2f;   // How often Unity pushes fresh data (matches AISTransponder.broadcastInterval)

    private HttpListener  _listener;
    private Thread        _thread;
    private volatile bool _running;

    // Snapshot rebuilt on the Unity main thread, served from the HTTP thread
    private volatile string _jsonSnapshot = "[]";
    private float _snapshotTimer;

    // ───────────────────────────────────────────────
    void Start()
    {
        _listener = new HttpListener();
        _listener.Prefixes.Add($"http://localhost:{port}/");
        _listener.Prefixes.Add($"http://127.0.0.1:{port}/");
        _listener.Start();

        _running = true;
        _thread  = new Thread(ServeLoop) { IsBackground = true };
        _thread.Start();

        Debug.Log($"[AISHttpServer] Listening on http://localhost:{port}/");
    }

    void Update()
    {
        // Rebuild JSON snapshot on the main thread (safe to access Unity objects)
        _snapshotTimer += Time.deltaTime;
        if (_snapshotTimer >= 1f / broadcastHz)
        {
            _snapshotTimer = 0f;
            _jsonSnapshot  = BuildSnapshot();
        }
    }

    void OnDestroy()
    {
        _running = false;
        _listener?.Stop();
        _thread?.Join(500);
    }

    // ───────────────────────────────────────────────
    //  HTTP THREAD
    // ───────────────────────────────────────────────
    private void ServeLoop()
    {
        while (_running)
        {
            HttpListenerContext ctx;
            try   { ctx = _listener.GetContext(); }
            catch { break; }

            try   { HandleRequest(ctx); }
            catch (Exception e) { Debug.LogWarning("[AISHttpServer] " + e.Message); }
        }
    }

    private void HandleRequest(HttpListenerContext ctx)
    {
        var req  = ctx.Request;
        var resp = ctx.Response;

        // CORS — allow the HTML file opened from disk (file://) or any localhost origin
        resp.Headers.Add("Access-Control-Allow-Origin",  "*");
        resp.Headers.Add("Access-Control-Allow-Methods", "GET, OPTIONS");
        resp.Headers.Add("Access-Control-Allow-Headers", "Content-Type");

        if (req.HttpMethod == "OPTIONS")
        {
            resp.StatusCode = 204;
            resp.Close();
            return;
        }

        string path = req.Url.AbsolutePath.TrimEnd('/');
        string body;
        string mime;

        if (path == "/vessels" || path == "")
        {
            body = _jsonSnapshot;
            mime = "application/json";
        }
        else if (path == "/health")
        {
            body = "{\"status\":\"ok\"}";
            mime = "application/json";
        }
        else
        {
            resp.StatusCode = 404;
            body = "{\"error\":\"not found\"}";
            mime = "application/json";
        }

        byte[] buf = Encoding.UTF8.GetBytes(body);
        resp.ContentType     = mime + "; charset=utf-8";
        resp.ContentLength64 = buf.Length;
        resp.OutputStream.Write(buf, 0, buf.Length);
        resp.OutputStream.Close();
    }

    // ───────────────────────────────────────────────
    //  SNAPSHOT BUILDER  (main thread)
    // ───────────────────────────────────────────────
    private string BuildSnapshot()
    {
        var vessels = AISManager.GetAllList();   // see note below
        if (vessels == null || vessels.Count == 0)
            return "[]";

        var sb = new StringBuilder("[");
        for (int i = 0; i < vessels.Count; i++)
        {
            var v = vessels[i];
            if (i > 0) sb.Append(',');
            sb.Append('{');
            sb.Append($"\"mmsi\":{v.mmsi},");
            sb.Append($"\"name\":\"{EscapeJson(v.vesselName)}\",");
            sb.Append($"\"ship_type\":{(int)v.shipType},");
            sb.Append($"\"lat\":{v.latitude.ToString("F6", System.Globalization.CultureInfo.InvariantCulture)},");
            sb.Append($"\"lon\":{v.longitude.ToString("F6", System.Globalization.CultureInfo.InvariantCulture)},");
            sb.Append($"\"speed\":{v.speedKnots.ToString("F2", System.Globalization.CultureInfo.InvariantCulture)},");
            sb.Append($"\"course\":{v.heading.ToString("F1", System.Globalization.CultureInfo.InvariantCulture)},");
            sb.Append($"\"is_drone\":true,");
            sb.Append($"\"timestamp\":\"{DateTime.UtcNow:o}\"");
            sb.Append('}');
        }
        sb.Append(']');
        return sb.ToString();
    }

    private static string EscapeJson(string s)
        => s?.Replace("\\", "\\\\").Replace("\"", "\\\"") ?? "";
}
