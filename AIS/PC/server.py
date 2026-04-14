# server.py — runs on your PC
import os
import asyncio, json, sqlite3, websockets, socket
from datetime import datetime, timezone, timedelta
from pyais import decode
from aiohttp import web

TCP_PORT  = 5001
WS_PORT   = 8080
HTTP_PORT = 8000

connected_browsers = set()

db = sqlite3.connect("ships.db", check_same_thread=False, isolation_level=None)
db.execute("PRAGMA journal_mode=WAL")
db.execute("PRAGMA cache_size=-64000")

db.execute("""CREATE TABLE IF NOT EXISTS ship_registry (
    mmsi        TEXT PRIMARY KEY,
    name        TEXT,
    ship_type   INTEGER,
    last_lat    REAL,
    last_lon    REAL,
    last_speed  REAL,
    last_course REAL,
    last_seen   TEXT,
    first_seen  TEXT
)""")

db.execute("""CREATE TABLE IF NOT EXISTS positions (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts        TEXT,
    mmsi      TEXT,
    lat       REAL,
    lon       REAL,
    speed     REAL,
    course    REAL
)""")

db.execute("CREATE INDEX IF NOT EXISTS idx_positions_ts   ON positions(ts)")
db.execute("CREATE INDEX IF NOT EXISTS idx_positions_mmsi ON positions(mmsi)")

fragment_buffer = {}

MAX_DB_BYTES = 4 * 1024 * 1024 * 1024  # 4GB
MAX_AGE_DAYS = 14


def cleanup_db():
    cutoff = (datetime.now(timezone.utc) - timedelta(days=MAX_AGE_DAYS)).isoformat()
    deleted = db.execute("DELETE FROM positions WHERE ts < ?", (cutoff,)).rowcount
    db.execute("VACUUM")

    size = os.path.getsize("ships.db")
    if size > MAX_DB_BYTES:
        total = db.execute("SELECT COUNT(*) FROM positions").fetchone()[0]
        keep  = int(total * 0.9)
        db.execute("""DELETE FROM positions WHERE id NOT IN (
            SELECT id FROM positions ORDER BY ts DESC LIMIT ?
        )""", (keep,))
        db.execute("VACUUM")
        print(f"[DB] Size limit hit — trimmed to {keep} rows")

    print(f"[DB] Cleanup done. Deleted {deleted} old rows. Size: {size/1024/1024:.1f} MB")


async def load_known_ships():
    rows = db.execute("""
        SELECT mmsi, name, ship_type, last_lat, last_lon, last_speed, last_course, last_seen
        FROM ship_registry
    """).fetchall()
    print(f"[DB] Loaded {len(rows)} known ships from registry")
    return rows


async def broadcast(msg):
    global connected_browsers
    dead = set()
    for ws in connected_browsers:
        try:
            await ws.send(msg)
        except:
            dead.add(ws)
    connected_browsers -= dead


async def ws_handler(websocket):
    connected_browsers.add(websocket)
    try:
        rows = db.execute("""
            SELECT mmsi, name, ship_type, last_lat, last_lon, last_speed, last_course
            FROM ship_registry
        """).fetchall()
        for r in rows:
            payload = {
                "mmsi": r[0], "name": r[1] or "", "type": r[2] or 0,
                "lat": r[3], "lon": r[4], "speed": r[5], "course": r[6],
                "ts": "", "historic": True
            }
            await websocket.send(json.dumps(payload))
        await websocket.wait_closed()
    finally:
        connected_browsers.discard(websocket)


async def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", TCP_PORT))
    srv.listen(1)
    srv.setblocking(False)
    loop = asyncio.get_event_loop()
    print(f"TCP listening on :{TCP_PORT}")
    while True:
        conn, addr = await loop.sock_accept(srv)
        print(f"RPi connected from {addr}")
        asyncio.create_task(handle_rpi(conn))


async def handle_rpi(conn):
    loop = asyncio.get_event_loop()
    buf = b""
    while True:
        try:
            data = await loop.sock_recv(conn, 4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                await process_nmea(line.decode("ascii", errors="ignore").strip())
        except:
            break


async def process_nmea(sentence):
    if sentence.startswith("!AIVDM") or sentence.startswith("!AIVDO"):
        try:
            parts = sentence.split(',')
            total_frags = int(parts[1])
            frag_num    = int(parts[2])
            seq_id      = parts[3]
            channel     = parts[4]

            if total_frags == 1:
                sentences_to_decode = [sentence]
            else:
                key = (seq_id, channel)
                if key not in fragment_buffer:
                    fragment_buffer[key] = {}
                fragment_buffer[key][frag_num] = sentence

                if len(fragment_buffer[key]) < total_frags:
                    return

                sentences_to_decode = [fragment_buffer[key][i] for i in sorted(fragment_buffer[key])]
                del fragment_buffer[key]

            msg = decode(*sentences_to_decode).asdict()
            print(f"[DECODED] type:{msg.get('msg_type')} mmsi:{msg.get('mmsi')} name:{msg.get('shipname','')}")

            def safe(val):
                if isinstance(val, set): return None
                return val

            payload = {
                "mmsi":   str(msg.get("mmsi", "")),
                "lat":    safe(msg.get("lat")),
                "lon":    safe(msg.get("lon")),
                "speed":  safe(msg.get("speed")),
                "course": safe(msg.get("course")),
                "name":   safe(msg.get("shipname", "")),
                "type":   safe(msg.get("ship_type", 0)),
                "ts":     datetime.now(timezone.utc).isoformat()
            }

            if (payload["lat"] and payload["lon"] and
                abs(payload["lat"]) <= 90 and
                abs(payload["lon"]) <= 180):

                now = datetime.now(timezone.utc).isoformat()

                db.execute("""INSERT INTO ship_registry
                    (mmsi, name, ship_type, last_lat, last_lon, last_speed, last_course, last_seen, first_seen)
                    VALUES (?,?,?,?,?,?,?,?,?)
                    ON CONFLICT(mmsi) DO UPDATE SET
                        name        = COALESCE(NULLIF(excluded.name,''), name),
                        ship_type   = COALESCE(excluded.ship_type, ship_type),
                        last_lat    = excluded.last_lat,
                        last_lon    = excluded.last_lon,
                        last_speed  = excluded.last_speed,
                        last_course = excluded.last_course,
                        last_seen   = excluded.last_seen
                """, (payload["mmsi"], payload["name"], payload["type"],
                      payload["lat"], payload["lon"], payload["speed"],
                      payload["course"], now, now))

                db.execute("INSERT INTO positions (ts,mmsi,lat,lon,speed,course) VALUES (?,?,?,?,?,?)",
                    (now, payload["mmsi"], payload["lat"], payload["lon"],
                     payload["speed"], payload["course"]))

                await broadcast(json.dumps(payload))

            elif payload["name"] and payload["mmsi"]:
                db.execute("""INSERT INTO ship_registry (mmsi, name, ship_type, last_seen, first_seen)
                    VALUES (?,?,?,?,?)
                    ON CONFLICT(mmsi) DO UPDATE SET
                        name      = COALESCE(NULLIF(excluded.name,''), name),
                        ship_type = COALESCE(excluded.ship_type, ship_type),
                        last_seen = excluded.last_seen
                """, (payload["mmsi"], payload["name"], payload["type"],
                      datetime.now(timezone.utc).isoformat(),
                      datetime.now(timezone.utc).isoformat()))

                await broadcast(json.dumps(payload))

        except Exception as e:
            print(f"[ERROR] {e} — sentence: {sentence}")


# ── HTTP handlers ──

async def handle_index(request):
    return web.FileResponse('./index.html')


async def handle_options(request):
    return web.Response(headers={
        'Access-Control-Allow-Origin':  '*',
        'Access-Control-Allow-Methods': 'GET, OPTIONS',
        'Access-Control-Allow-Headers': '*'
    })


async def handle_export(request):
    period = request.query.get('period', '24h')
    fmt    = request.query.get('format', 'csv')
    mmsi   = request.query.get('mmsi', None)

    periods = {
        '1h':  1,  '3h':  3,  '6h':  6,
        '12h': 12, '24h': 24, '3d':  72, '7d': 168, '14d': 336
    }
    hours  = periods.get(period, 24)
    cutoff = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat()

    query = """
        SELECT p.ts, p.mmsi, r.name, r.ship_type, p.lat, p.lon, p.speed, p.course
        FROM positions p
        LEFT JOIN ship_registry r ON p.mmsi = r.mmsi
        WHERE p.ts >= ?
    """
    params = [cutoff]
    if mmsi:
        query += " AND p.mmsi = ?"
        params.append(mmsi)
    query += " ORDER BY p.ts ASC"

    rows = db.execute(query, params).fetchall()

    if fmt == 'csv':
        lines = ["timestamp,mmsi,name,ship_type,lat,lon,speed_kn,course_deg"]
        for r in rows:
            lines.append(','.join(str(v) if v is not None else '' for v in r))
        body     = '\n'.join(lines)
        filename = f"ais_{period}_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.csv"
        return web.Response(
            body=body,
            content_type='text/csv',
            headers={
                'Content-Disposition':        f'attachment; filename="{filename}"',
                'Access-Control-Allow-Origin': '*'
            }
        )
    else:
        data = [{"ts": r[0], "mmsi": r[1], "name": r[2], "ship_type": r[3],
                 "lat": r[4], "lon": r[5], "speed": r[6], "course": r[7]} for r in rows]
        return web.Response(
            text=json.dumps(data),
            content_type='application/json',
            headers={'Access-Control-Allow-Origin': '*'}
        )


async def handle_track(request):
    mmsi   = request.match_info['mmsi']
    hours  = int(request.query.get('hours', 24))
    cutoff = (datetime.now(timezone.utc) - timedelta(hours=hours)).isoformat()

    rows = db.execute("""
        SELECT ts, lat, lon, speed, course FROM positions
        WHERE mmsi = ? AND ts >= ?
        ORDER BY ts ASC
    """, (mmsi, cutoff)).fetchall()

    data = [{"ts": r[0], "lat": r[1], "lon": r[2], "speed": r[3], "course": r[4]} for r in rows]
    return web.Response(
        text=json.dumps(data),
        content_type='application/json',
        headers={'Access-Control-Allow-Origin': '*'}
    )


async def main():
    cleanup_db()
    await load_known_ships()

    async def daily_cleanup():
        while True:
            await asyncio.sleep(86400)
            cleanup_db()

    app = web.Application()
    app.router.add_route('OPTIONS', '/{path_info:.*}', handle_options)
    app.router.add_get('/',             handle_index)
    app.router.add_get('/index.html',   handle_index)
    app.router.add_get('/export',       handle_export)
    app.router.add_get('/track/{mmsi}', handle_track)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', HTTP_PORT)
    await site.start()
    print(f"HTTP listening on :{HTTP_PORT}")

    await asyncio.gather(
        tcp_server(),
        websockets.serve(ws_handler, "0.0.0.0", WS_PORT),
        daily_cleanup()
    )


asyncio.run(main())
