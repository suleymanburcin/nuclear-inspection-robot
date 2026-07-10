#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rover_gcs.py  —  Outdoor GPS Rover Yer Kontrol (TEK DOSYA)
==========================================================
Gemi GCS mantigi: SOLDA uydu haritasi (tiklayarak waypoint koy, rota cizilir),
SAGDA telemetri + kontrol. "Baslat" -> ESP32'den (USB) gelen nav JSON'i okunur,
pure-pursuit ile Ackermann arac icin direksiyon+gaz komutu uretilir ve canli
gosterilir. (Servo cikisi opsiyonel: 'cmd gonder' kutusu.)

Zincir:   GPS + BNO055 --> ESP32 --USB--> Jetson (bu uygulama)
nav satiri (ESP32, ~10 Hz):
  {"nav":1,"fix":1,"sats":9,"lat":36.15919,"lon":33.56161,
   "spd":0.8,"cog":142.3,"yaw":138.7,"cal":3}

Kurulum:
  sudo apt install python3-pyqt5 python3-pyqt5.qtwebengine
  pip3 install pyserial
Calistir:
  python3 rover_gcs.py
"""

import sys, json, math

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSlot, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel

# ======================================================================
# AYARLAR
# ======================================================================
# Test alani ankraji (Buyukeceli / Akkuyu NGS - dogrulanmis) = harita merkezi
ANCHOR_LAT = 36.1591933
ANCHOR_LON = 33.5616058
START_ZOOM = 17

# Serpantin yedek alani (haritada waypoint cizmezsen kullanilir)
FIELD_W, FIELD_H, LANE_M = 30.0, 20.0, 4.0

# Arac / navigasyon (benzinli RC, ~1.20 x 1.00 m, Ackermann)
WHEELBASE   = 0.80    # m  on-arka dingil mesafesi <-- olcup gir
MAX_STEER   = 30.0    # derece direksiyon limiti
LOOKAHEAD   = 3.0     # m  pure pursuit bakis mesafesi
WP_RADIUS   = 1.5     # m  ulasildi esigi
CRUISE      = 0.35    # 0-1 duz seyir gaz
SLOW        = 0.20    # 0-1 donuste/yakinda gaz
SPD_FUSE    = 0.5     # m/s ustu -> GPS cog, alti -> BNO yaw
DECLINATION = 5.5     # derece E (Akkuyu ~ +5.5, kesin degeri dogrula)
GEOFENCE    = 5.0     # m  alan disi tasma emniyeti

SERIAL_BAUD = 115200
MISSION_FILE = "mission.json"

M_PER_DEG_LAT = 111320.0
def m_per_deg_lon(lat): return 111320.0 * math.cos(math.radians(lat))

# ======================================================================
# GEOMETRI + NAVIGASYON
# ======================================================================
def local_offset(lat, lon):
    return ((lon - ANCHOR_LON) * m_per_deg_lon(ANCHOR_LAT),
            (lat - ANCHOR_LAT) * M_PER_DEG_LAT)

def dist_bearing(clat, clon, tlat, tlon):
    dE = (tlon - clon) * m_per_deg_lon(clat)
    dN = (tlat - clat) * M_PER_DEG_LAT
    return math.hypot(dE, dN), math.degrees(math.atan2(dE, dN)) % 360.0

def wrap180(a): return (a + 180.0) % 360.0 - 180.0

def make_lawnmower(lat0, lon0, w, h, lane):
    loc, y, flip = [], 0.0, False
    while y <= h + 1e-6:
        loc += [(0.0, y), (w, y)] if not flip else [(w, y), (0.0, y)]
        flip = not flip; y += lane
    dlat, dlon = 1.0 / M_PER_DEG_LAT, 1.0 / m_per_deg_lon(lat0)
    return [(lat0 + n * dlat, lon0 + e * dlon) for (e, n) in loc]

def fused_heading(nav):
    spd = nav.get("spd") or 0.0
    cog, yaw = nav.get("cog"), nav.get("yaw")
    if spd >= SPD_FUSE and cog is not None and cog >= 0:
        return cog
    if yaw is not None:
        return (yaw + DECLINATION) % 360.0
    return None

def in_geofence(lat, lon):
    e, n = local_offset(lat, lon)
    return -GEOFENCE <= e <= FIELD_W + GEOFENCE and -GEOFENCE <= n <= FIELD_H + GEOFENCE

class PurePursuit:
    def __init__(self, waypoints):
        self.wps, self.i, self.done = waypoints, 0, False

    def step(self, nav):
        if self.done or self.i >= len(self.wps):
            self.done = True; return 0.0, 0.0, {"state": "DONE"}
        lat, lon = nav.get("lat"), nav.get("lon")
        if not nav.get("fix") or lat is None:
            return 0.0, 0.0, {"state": "NO_FIX", "sats": nav.get("sats", 0)}
        if not in_geofence(lat, lon):
            return 0.0, 0.0, {"state": "GEOFENCE_STOP"}
        hd = fused_heading(nav)
        if hd is None:
            return 0.0, 0.0, {"state": "NO_HEADING"}
        tlat, tlon = self.wps[self.i]
        d, brg = dist_bearing(lat, lon, tlat, tlon)
        if d <= WP_RADIUS:
            self.i += 1
            if self.i >= len(self.wps):
                self.done = True; return 0.0, 0.0, {"state": "DONE"}
            tlat, tlon = self.wps[self.i]
            d, brg = dist_bearing(lat, lon, tlat, tlon)
        alpha = math.radians(wrap180(brg - hd))
        Ld = max(LOOKAHEAD, d)
        steer = max(-MAX_STEER, min(MAX_STEER,
                    math.degrees(math.atan2(2.0 * WHEELBASE * math.sin(alpha), Ld))))
        thr = SLOW if (abs(math.degrees(alpha)) > 25 or d < 2 * WP_RADIUS) else CRUISE
        return steer, thr, {"state": "RUN", "wp": self.i + 1, "n": len(self.wps),
                            "dist": round(d, 1), "brg": round(brg, 1),
                            "hd": round(hd, 1), "sats": nav.get("sats", 0)}

# ======================================================================
# NAVIGASYON THREAD'I  (seri oku -> pure pursuit -> telemetri sinyali)
# ======================================================================
class NavThread(QThread):
    telem = pyqtSignal(object)

    def __init__(self, port, waypoints, send_cmd=False):
        super().__init__()
        self.port, self.send_cmd = port, send_cmd
        self.pp = PurePursuit(waypoints)
        self._run = True

    def run(self):
        try:
            import serial
        except Exception:
            self.telem.emit({"state": "NO_PYSERIAL"}); return
        try:
            ser = serial.Serial(self.port, SERIAL_BAUD, timeout=1)
        except Exception as e:
            self.telem.emit({"state": "PORT_FAIL", "msg": str(e)}); return

        self.telem.emit({"state": "LINK", "msg": self.port})
        while self._run:
            raw = ser.readline().decode("utf-8", "ignore").strip()
            if not raw.startswith("{") or '"nav"' not in raw:
                continue
            try:
                nav = json.loads(raw)
            except json.JSONDecodeError:
                continue
            steer, thr, info = self.pp.step(nav)
            info["steer"], info["thr"] = round(steer, 1), round(thr, 2)
            self.telem.emit(info)
            if self.send_cmd:                    # Jetson -> ESP32 servo komutu
                try:
                    ser.write((json.dumps({"cmd": 1, "steer": round(steer, 1),
                                           "thr": round(thr, 2)}) + "\n").encode())
                except Exception:
                    pass
            if self.pp.done:
                self.telem.emit({"state": "DONE"}); break
        try: ser.close()
        except Exception: pass

    def stop(self):
        self._run = False
        self.wait(1500)

# ======================================================================
# JS <-> PYTHON KOPRUSU
# ======================================================================
class Bridge(QObject):
    def __init__(self):
        super().__init__(); self.waypoints = []

    @pyqtSlot(str)
    def sync(self, js):
        try: self.waypoints = json.loads(js)
        except Exception: self.waypoints = []

    @pyqtSlot()
    def commit(self):
        with open(MISSION_FILE, "w") as f:
            json.dump(self.waypoints, f)
        print(f"[MISSION] {len(self.waypoints)} waypoint -> {MISSION_FILE}")

# ======================================================================
# HARITA HTML (Leaflet)
# ======================================================================
HTML = """
<!DOCTYPE html><html><head><meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<style>
 html,body,#map{height:100%;margin:0;background:#000;font-family:Arial}
 #bar{position:absolute;z-index:1000;top:10px;left:50px;right:10px;display:flex;gap:8px;pointer-events:none}
 #bar>*{pointer-events:auto}
 .btn{background:#05080e;color:#dff6ff;border:1px solid #00e5ff;border-radius:8px;padding:7px 12px;font-size:13px;font-weight:bold;cursor:pointer}
 .btn:hover{background:#0e2233}
 #cnt{background:#05080e;color:#00e5ff;border:1px solid #0e2233;border-radius:8px;padding:7px 12px;font-size:13px;font-weight:bold}
 .wp{background:#00e5ff;color:#000;border:2px solid #000;border-radius:50%;width:24px;height:24px;line-height:22px;text-align:center;font-weight:bold;font-size:12px}
 .anchor{background:#ff9f1c;border:2px solid #000;border-radius:4px;width:16px;height:16px}
</style></head><body>
<div id="bar">
 <div id="cnt">0 waypoint</div>
 <button class="btn" onclick="undo()">Geri al</button>
 <button class="btn" onclick="clearAll()">Temizle</button>
 <button class="btn" onclick="save()">Kaydet</button>
</div>
<div id="map"></div>
<script>
var LAT=__LAT__,LON=__LON__,Z=__Z__;
var map=L.map('map').setView([LAT,LON],Z);
var sat=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:20,attribution:'Esri'});
var osm=L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'OSM'});
sat.addTo(map);
L.control.layers({'Uydu':sat,'Sokak':osm},null,{position:'bottomleft'}).addTo(map);
L.marker([LAT,LON],{icon:L.divIcon({className:'anchor',html:'',iconSize:[16,16]})}).addTo(map).bindTooltip('Ankraj');
var pts=[],markers=[],line=L.polyline([],{color:'#00e5ff',weight:3,opacity:.9}).addTo(map);
function redraw(){
 markers.forEach(m=>map.removeLayer(m));markers=[];
 pts.forEach((p,i)=>{markers.push(L.marker(p,{icon:L.divIcon({className:'wp',html:(i+1),iconSize:[24,24]})}).addTo(map));});
 line.setLatLngs(pts);
 document.getElementById('cnt').textContent=pts.length+' waypoint';
 if(window.bridge)bridge.sync(JSON.stringify(pts));
}
map.on('click',e=>{pts.push([e.latlng.lat,e.latlng.lng]);redraw();});
function undo(){pts.pop();redraw();}
function clearAll(){pts=[];redraw();}
function save(){if(window.bridge)bridge.commit();}
new QWebChannel(qt.webChannelTransport,ch=>{window.bridge=ch.objects.bridge;});
</script></body></html>
"""

# ======================================================================
# ANA PENCERE  (SOL harita · SAG telemetri/kontrol)
# ======================================================================
C_BG, C_ACC, C_TXT, C_MUTE = "#000000", "#00e5ff", "#dff6ff", "#5b7a90"
C_GRN, C_ORG, C_RED = "#00ff9c", "#ff9f1c", "#ff2e63"

class RoverGCS(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rover GCS · Outdoor GPS · Akkuyu")
        self.resize(1200, 760)
        self.nav = None

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central); h.setContentsMargins(0, 0, 0, 0); h.setSpacing(0)

        # --- SOL: harita ---
        self.view = QWebEngineView()
        self.bridge = Bridge()
        ch = QWebChannel(); ch.registerObject("bridge", self.bridge)
        self.view.page().setWebChannel(ch)
        html = (HTML.replace("__LAT__", repr(ANCHOR_LAT))
                    .replace("__LON__", repr(ANCHOR_LON)).replace("__Z__", str(START_ZOOM)))
        self.view.setHtml(html, QtCore.QUrl("https://local/"))
        h.addWidget(self.view, 1)

        # --- SAG: kontrol + telemetri ---
        panel = QtWidgets.QWidget(); panel.setFixedWidth(320)
        panel.setStyleSheet(f"background:{C_BG};border-left:1px solid {C_ACC};")
        v = QtWidgets.QVBoxLayout(panel); v.setContentsMargins(12, 12, 12, 12); v.setSpacing(8)

        v.addWidget(self._h("NAVIGASYON"))
        self.port = QtWidgets.QLineEdit("/dev/ttyUSB0"); self.port.setStyleSheet(self._css_in())
        v.addWidget(self.port)
        self.cmd = QtWidgets.QCheckBox("cmd gonder (ESP32 servo)")
        self.cmd.setStyleSheet(f"color:{C_MUTE};font-size:12px;")
        v.addWidget(self.cmd)
        row = QtWidgets.QHBoxLayout()
        self.b_start = QtWidgets.QPushButton("BASLAT"); self.b_start.setStyleSheet(self._css_btn(C_GRN))
        self.b_stop = QtWidgets.QPushButton("DURDUR"); self.b_stop.setStyleSheet(self._css_btn(C_RED))
        self.b_start.clicked.connect(self.start_nav); self.b_stop.clicked.connect(self.stop_nav)
        self.b_stop.setEnabled(False)
        row.addWidget(self.b_start); row.addWidget(self.b_stop); v.addLayout(row)

        v.addSpacing(6); v.addWidget(self._h("TELEMETRI"))
        self.t = {}
        for key, lbl in [("state", "DURUM"), ("wp", "WAYPOINT"), ("dist", "MESAFE"),
                         ("brg", "HEDEF YON"), ("hd", "HEADING"),
                         ("steer", "DIREKSIYON"), ("thr", "GAZ"), ("sats", "UYDU")]:
            v.addWidget(self._row(key, lbl))
        v.addStretch(1)
        h.addWidget(panel)

    # --- yardimci stil/olusturucular ---
    def _h(self, txt):
        l = QtWidgets.QLabel(txt); l.setStyleSheet(f"color:{C_ACC};font-size:13px;font-weight:bold;letter-spacing:2px;")
        return l
    def _css_in(self):
        return f"background:#05080e;color:{C_TXT};border:1px solid #0e2233;border-radius:6px;padding:6px;font-size:13px;"
    def _css_btn(self, col):
        return (f"background:#05080e;color:{col};border:1px solid {col};border-radius:8px;"
                f"padding:8px;font-size:13px;font-weight:bold;")
    def _row(self, key, lbl):
        w = QtWidgets.QWidget(); r = QtWidgets.QHBoxLayout(w); r.setContentsMargins(0, 0, 0, 0)
        a = QtWidgets.QLabel(lbl); a.setStyleSheet(f"color:{C_MUTE};font-size:12px;letter-spacing:1px;")
        b = QtWidgets.QLabel("--"); b.setStyleSheet(f"color:{C_TXT};font-size:14px;font-weight:bold;")
        b.setAlignment(QtCore.Qt.AlignRight)
        r.addWidget(a); r.addStretch(1); r.addWidget(b); self.t[key] = b
        return w

    # --- nav baslat/durdur ---
    def start_nav(self):
        wps = self.bridge.waypoints or make_lawnmower(ANCHOR_LAT, ANCHOR_LON, FIELD_W, FIELD_H, LANE_M)
        if not wps:
            self.t["state"].setText("WAYPOINT YOK"); return
        self.nav = NavThread(self.port.text().strip(), wps, self.cmd.isChecked())
        self.nav.telem.connect(self.on_telem)
        self.nav.start()
        self.b_start.setEnabled(False); self.b_stop.setEnabled(True)

    def stop_nav(self):
        if self.nav:
            self.nav.stop(); self.nav = None
        self.b_start.setEnabled(True); self.b_stop.setEnabled(False)
        self.t["state"].setText("DURDU"); self.t["state"].setStyleSheet(f"color:{C_MUTE};font-size:14px;font-weight:bold;")

    def on_telem(self, d):
        st = d.get("state", "")
        col = {"RUN": C_GRN, "DONE": C_ACC, "LINK": C_ACC}.get(st, C_RED)
        self.t["state"].setText(st)
        self.t["state"].setStyleSheet(f"color:{col};font-size:14px;font-weight:bold;")
        if "wp" in d:   self.t["wp"].setText(f"{d['wp']}/{d.get('n','?')}")
        if "dist" in d: self.t["dist"].setText(f"{d['dist']} m")
        if "brg" in d:  self.t["brg"].setText(f"{d['brg']}\u00b0")
        if "hd" in d:   self.t["hd"].setText(f"{d['hd']}\u00b0")
        if "steer" in d:self.t["steer"].setText(f"{d['steer']:+}\u00b0")
        if "thr" in d:  self.t["thr"].setText(f"{d['thr']}")
        if "sats" in d: self.t["sats"].setText(str(d["sats"]))
        if st == "DONE":
            self.b_start.setEnabled(True); self.b_stop.setEnabled(False)

    def closeEvent(self, e):
        if self.nav: self.nav.stop()
        e.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = RoverGCS(); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
