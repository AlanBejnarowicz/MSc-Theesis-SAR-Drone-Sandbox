# server.py — runs on your PC
import asyncio, json, sqlite3, websockets, socket
from datetime import datetime, timezone
from pyais import decode

TCP_PORT = 5001       # receives from RPi
WS_PORT  = 8080       # serves to browser
HTTP_PORT = 8000      # serves index.html (use: python -m http.server 8000)

connected_browsers = set()
db = sqlite3.connect("ships.db")
db.execute("""CREATE TABLE IF NOT EXISTS ships (
    ts TEXT, mmsi TEXT, lat REAL, lon REAL,
    speed REAL, course REAL, name TEXT, type INTEGER
)""")
db.commit()

async def broadcast(msg):
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
            msg = decode(sentence).asdict()
            payload = {
                "mmsi":   str(msg.get("mmsi", "")),
                "lat":    msg.get("lat"),
                "lon":    msg.get("lon"),
                "speed":  msg.get("speed"),
                "course": msg.get("course"),
                "name":   msg.get("shipname", ""),
                "type":   msg.get("ship_type", 0),
                "ts": datetime.now(timezone.utc).isoformat()
            }
            if payload["lat"] and payload["lon"]:
                db.execute("INSERT INTO ships VALUES (?,?,?,?,?,?,?,?)",
                    (payload["ts"], payload["mmsi"], payload["lat"], payload["lon"],
                    payload["speed"], payload["course"], payload["name"], payload["type"]))
                db.commit()
                await broadcast(json.dumps(payload))
        except Exception as e:
            pass  # partial/multi-part messages are common, ignore silently

async def main():
    await asyncio.gather(
        tcp_server(),
        websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    )

asyncio.run(main())