#!/usr/bin/env python3
"""ble_cdp_probe.py — 045 F2 / R2 remote BLE download measurement. Needs Chromium started with --remote-debugging-port=9222
--enable-experimental-web-platform-features on the page http://127.0.0.1:8765/flight_logger.html (python3 -m http.server 8765 in
xiao/web) and websocket-client. Usage: ble_cdp_probe.py [seconds] [out.json].
Drive flight_logger.html in a CDP-controlled Chromium: connect to XIAO-GP, list logs, sample a download for N seconds with
external timing probes on the page's Web Bluetooth calls, then disconnect. Only connect()/listFiles()/downloadFile()/disconnect()
are ever invoked (never erase)."""
import json, re, sys, time, urllib.request
import websocket

SAMPLE_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/ble_probe.json"
ALLOWED_CALLS = {"connect()", "listFiles()", "disconnect()"}

PROBE_JS = r"""(()=>{ if (window.__probe) return 'already';
 const P = window.__probe = {t0: performance.now(), ev: []};
 const now = () => +(performance.now() - P.t0).toFixed(1);
 const C = BluetoothRemoteGATTCharacteristic.prototype;
 for (const m of ['writeValue','writeValueWithResponse','writeValueWithoutResponse']) {
   const f = C[m]; if (!f) continue;
   C[m] = function(v) { const u = this.uuid.slice(4,8); let s=''; try { s = new TextDecoder().decode(v); } catch(e) {}
     P.ev.push(['w', now(), u, s.slice(0,24)]);
     return f.apply(this, arguments).then(r => { P.ev.push(['wd', now(), u]); return r; },
                                          e => { P.ev.push(['we', now(), u, String(e)]); throw e; }); };
 }
 const ael = C.addEventListener;
 C.addEventListener = function(type, cb, o) {
   if (type === 'characteristicvaluechanged') { const u = this.uuid.slice(4,8);
     const w = function(e) { const dv = e.target.value; let s='';
       if (u !== '6002') { try { s = new TextDecoder().decode(dv); } catch(_) {} }
       P.ev.push(['n', now(), u, dv.byteLength, s.slice(0,80)]); return cb.call(this, e); };
     return ael.call(this, type, w, o); }
   return ael.apply(this, arguments); };
 const rd = navigator.bluetooth.requestDevice.bind(navigator.bluetooth);
 navigator.bluetooth.requestDevice = (opts) =>
   rd({acceptAllDevices: true, optionalServices: (opts && opts.optionalServices) || []});
 return 'ok'; })()"""


class CDP:
    def __init__(self, url):
        self.ws = websocket.create_connection(url, timeout=1); self.i = 0; self.events = []

    def call(self, method, params=None, timeout=15):
        self.i += 1; mid = self.i
        self.ws.send(json.dumps({"id": mid, "method": method, "params": params or {}}))
        t0 = time.time()
        while time.time() - t0 < timeout:
            try: m = json.loads(self.ws.recv())
            except websocket.WebSocketTimeoutException: continue
            if m.get("id") == mid: return m
            self.events.append(m)
        raise TimeoutError(method)

    def pump(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            try: self.events.append(json.loads(self.ws.recv()))
            except websocket.WebSocketTimeoutException: pass

    def js(self, expr, gesture=False, await_promise=False):
        r = self.call("Runtime.evaluate", {"expression": expr, "userGesture": gesture, "awaitPromise": await_promise,
                                           "returnByValue": True})
        return r.get("result", {}).get("result", {}).get("value")


def page_ws():
    for _ in range(40):
        try:
            for t in json.load(urllib.request.urlopen("http://127.0.0.1:9222/json")):
                if t.get("type") == "page" and "flight_logger" in t.get("url", ""): return t["webSocketDebuggerUrl"]
        except Exception: pass
        time.sleep(0.5)
    raise SystemExit("no flight_logger page on :9222")


def main():
    c = CDP(page_ws())
    for m in ("Runtime.enable", "Page.enable", "DeviceAccess.enable"): c.call(m)
    c.call("Page.reload", {"ignoreCache": True}); c.pump(3)
    print("probe:", c.js(PROBE_JS))
    assert "connect()" in ALLOWED_CALLS
    c.js("connect()", gesture=True)                      # opens the device chooser
    picked = None; t0 = time.time()
    while not picked and time.time() - t0 < 30:
        c.pump(0.5)
        for e in list(c.events):
            if e.get("method") == "DeviceAccess.deviceRequestPrompted":
                p = e["params"]; devs = p.get("devices", [])
                hit = [d for d in devs if d.get("id") == "C0:73:67:87:59:36" or d.get("name") == "XIAO-GP"]
                if hit:
                    c.call("DeviceAccess.selectPrompt", {"id": p["id"], "deviceId": hit[0]["id"]}); picked = hit[0]; break
        c.events = [e for e in c.events if e.get("method") != "DeviceAccess.deviceRequestPrompted"] if not picked else c.events
    print("chooser:", picked)
    if not picked: raise SystemExit("XIAO-GP not offered in the chooser")
    for _ in range(40):
        if "Connected" in (c.js("document.getElementById('connectionStatus').textContent") or ""): break
        c.pump(0.5)
    print("status:", c.js("document.getElementById('status').textContent"))
    c.js("listFiles()", gesture=True); c.pump(4)
    html = c.js("document.getElementById('fileList').innerHTML") or ""
    files = re.findall(r"downloadFile\('([^']+)'\)", html)
    print("files:", files)
    if not files: raise SystemExit("no files listed")
    target = files[-1]
    c.js(f"downloadFile({json.dumps(target)})", gesture=True)
    print(f"downloading {target} for {SAMPLE_S:.0f} s ...", flush=True)
    t_dl = time.time()
    while time.time() - t_dl < SAMPLE_S:   # stop early once the page reports the download finished or failed
        c.pump(1.0)
        st = c.js("document.getElementById('status').textContent") or ""
        if time.time() - t_dl > 3 and ("verified" in st or "CRC mismatch" in st or "Error" in st or "Disconnected" in st):
            break
    print(f"download phase {time.time() - t_dl:.1f} s", flush=True)
    probe = c.js("JSON.stringify(window.__probe.ev)")
    page_log = c.js("typeof dlLog !== 'undefined' ? dlLog.join('\\n') : ''") or ""   # 045 F2 page instrumentation
    status = c.js("document.getElementById('status').textContent")
    c.js("disconnect()", gesture=True); c.pump(2)
    json.dump({"target": target, "files": files, "status_at_end": status, "page_log": page_log,
               "events": json.loads(probe or "[]")},
              open(OUT, "w"))
    ev = json.loads(probe or "[]")
    data = [e for e in ev if e[0] == "n" and e[2] == "6002"]
    stat = [e for e in ev if e[0] == "n" and e[2] != "6002"]
    print("status msgs:", [e[4] for e in stat][:6])
    if data:
        sizes = sorted({e[3] for e in data}); tot = sum(e[3] for e in data); span = (data[-1][1] - data[0][1]) / 1000
        gaps = sorted(b[1] - a[1] for a, b in zip(data, data[1:]))
        print(f"data notifications {len(data)}  sizes {sizes}  bytes {tot}  span {span:.1f} s  rate {tot / max(span, 1e-3) / 1000:.2f} kB/s"
              f"  gap median {gaps[len(gaps) // 2] if gaps else 0:.1f} ms  p90 {gaps[int(.9 * len(gaps))] if gaps else 0:.1f} ms")
    print("page log tail:", *page_log.splitlines()[-6:], sep="\n  ")
    print("end status:", status, "| saved", OUT)


if __name__ == "__main__":
    main()
