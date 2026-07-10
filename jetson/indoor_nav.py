#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
indoor_nav.py  —  INDOOR OTONOM DRIVE PLAN (navigasyon planlayici)
===================================================================
Onceden cizilmis 3 kat haritasi (nukleer santral). Bir odaya tikla = baslangic,
baska odaya tikla = hedef. Sistem A* ile koridor+kapilardan gecen en kisa yolu
bulur, robot ikonu yolda ilerler, FORWARD/BACK/LEFT/RIGHT + mesafe komutlarini
ve ok yonunu gosterir. RETURN butonu: hedefte bekler, sonra geri doner.

NOT (durust cerceve):
  Bu bir PLANLAYICI/SIMULATOR. Robot GERCEKTE kendi yerini bilmez (localization
  yok - o ayri bir is: odometri/marker/SLAM). Burada konum varsayilir; arac
  rotayi planlar, komutlari uretir, haritada gosterir. Uretilen komutlar
  (ileri/sag/sol + metre) gercektir ve ileride robota UDP/UART ile baglanabilir.

Bagimsiz calisir (mevcut gas dock'a dokunmaz):
    python3 indoor_nav.py

Gereksinim: PyQt5, numpy
"""

import sys, heapq, math
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

# ----------------------------------------------------------------------
# TEMA
# ----------------------------------------------------------------------
C_BG    = "#0a0c0e"
C_PANEL = "#101417"
C_WALL  = "#2a3138"
C_FREE  = "#12181c"
C_ACC   = "#00e6c8"
C_GREEN = "#00e678"
C_RED   = "#f04646"
C_ORANGE= "#ff9a3c"
C_MUTE  = "#6e787d"
C_TEXT  = "#e6efef"
C_PATH  = "#00e6c8"

FONT = "DejaVu Sans"

# ----------------------------------------------------------------------
# KAT HARITALARI  (birim: METRE, origin sol-ust, x saga, y asagi)
#   room: (isim, x, y, w, h, kategori)   -> serbest dikdortgen (oda ici)
#   door: (x, y, w, h)                   -> duvardaki 1.5m bosluk (serbest kopru)
#   robot sadece oda MERKEZINE gider (tiklayinca o odanin merkezi hedef olur)
# odalarin en kisa kenari >= 7 m, kapilar 1.5 m.
# ----------------------------------------------------------------------

# kategori -> renk (nukleer santral temasi)
CAT_COL = {
    "reactor":  "#5a2530",   # reaktor - koyu kirmizi
    "control":  "#1f4a4a",   # kontrol - teal
    "cooling":  "#20364f",   # sogutma - mavi
    "power":    "#4a3f1f",   # elektrik/jenerator - amber
    "safety":   "#3a2350",   # guvenlik/radyasyon - mor
    "corridor": "#161c20",   # koridor
    "open":     "#151b1f",   # acik alan
}

def _room(name, x, y, w, h, cat):
    return {"name": name, "x": x, "y": y, "w": w, "h": h, "cat": cat,
            "cx": x + w / 2.0, "cy": y + h / 2.0}

def _door(x, y, w, h):
    return {"x": x, "y": y, "w": w, "h": h}

# ---- KAT 1: koridor + odalar (ust 3 oda, alt 2 oda) ----
KAT1 = {
    "name": "KAT 1  ·  Reaktor Seviyesi",
    "rooms": [
        _room("Reaktor Kontrol",   1,  1,  8, 8, "control"),
        _room("Turbin Salonu",    11,  1,  9, 8, "power"),
        _room("Jenerator Odasi",  22,  1,  9, 8, "power"),
        _room("Sogutma Pompasi",   4, 15, 10, 7, "cooling"),
        _room("Radyasyon Izleme", 18, 15, 11, 7, "safety"),
        _room("KORIDOR",           1, 10, 30, 3, "corridor"),
    ],
    "doors": [
        _door(4.5,  9, 1.5, 1),    # Reaktor Kontrol <-> koridor
        _door(15,   9, 1.5, 1),    # Turbin <-> koridor
        _door(26,   9, 1.5, 1),    # Jenerator <-> koridor
        _door(8.5, 13, 1.5, 2),    # Sogutma <-> koridor
        _door(23,  13, 1.5, 2),    # Radyasyon <-> koridor
    ],
    # asansor: koridorun sol ucunda; onu koridora acilir
    "elevator": {"x": 1.0, "y": 10.5, "w": 2.0, "h": 2.0},
    "start": (4.0, 11.5),          # asansor agzindan 1m onde (koridor ici)
    # SABIT ANA HAT (omurga) - koridor orta cizgisi, degismez
    "spine": [(3.0, 11.5), (29.0, 11.5)],   # yatay orta cizgi
}

# ---- KAT 2: merkez hac koridor + 2 yan + ust/alt odalar ----
KAT2 = {
    "name": "KAT 2  ·  Yardimci Sistemler",
    "rooms": [
        # yatay + dikey koridor (hac) - kesisir
        _room("KORIDOR-Y",         2, 13, 30, 3, "corridor"),   # yatay y=13..16
        _room("KORIDOR-D",       15.5, 1, 3, 27, "corridor"),   # dikey x=15.5..18.5 y=1..28
        # ust 2 oda (y=1..9, dikey koridora yandan baglanir)
        _room("Kontrol Merkezi",   2,  1, 12, 8, "control"),    # x=2..14
        _room("Yakit Havuzu",     20,  1, 12, 8, "cooling"),    # x=20..32
        # alt 2 oda (y=18..26, yatay koridora ustten baglanir)
        _room("Elektrik Panosu",   2, 18, 11, 8, "power"),      # x=2..13
        _room("Atik Depolama",    21, 18, 11, 8, "safety"),     # x=21..32
    ],
    "doors": [
        # ust odalar -> DIKEY koridora yandan (odalarin ic kenari ile x=15.5..18.5 arasi)
        _door(14,   4, 1.5, 1.5),   # Kontrol Merkezi (sag kenar x=14) -> dikey koridor (x=15.5)
        _door(18.5, 4, 1.5, 1.5),   # Yakit Havuzu (sol kenar x=20) -> dikey koridor (x=18.5)
        # alt odalar -> YATAY koridora ustten (oda ust kenari y=18, koridor alt y=16)
        _door(7,   16, 1.5, 2),     # Elektrik Panosu -> yatay koridor
        _door(25,  16, 1.5, 2),     # Atik Depolama -> yatay koridor
    ],
    # asansor: yatay koridorun sol ucunda
    "elevator": {"x": 0.0, "y": 13.5, "w": 2.0, "h": 2.0},
    "start": (3.0, 14.5),          # asansor agzindan 1m onde (yatay koridor)
    # SABIT ANA HAT (omurga) - hac koridor orta cizgileri
    "spine": [(3.0, 14.5), (31.0, 14.5),    # yatay orta cizgi
              (17.0, 2.0), (17.0, 27.0)],   # dikey orta cizgi (x=17)
}

# ---- KAT 3: acik alan (dumduz) + birkac isaretli bolge ----
KAT3 = {
    "name": "KAT 3  ·  Acik Denetim Alani",
    "rooms": [
        _room("ACIK ALAN",         1,  1, 30, 22, "open"),
    ],
    # acik alanda hedef noktalari (sanal bolgeler) - tiklanabilir merkezler
    "zones": [
        {"name": "Giris Noktasi",   "cx": 5,  "cy": 5},
        {"name": "Denetim-A",       "cx": 15, "cy": 6},
        {"name": "Denetim-B",       "cx": 26, "cy": 8},
        {"name": "Numune Alani",    "cx": 9,  "cy": 18},
        {"name": "Cikis Noktasi",   "cx": 25, "cy": 18},
    ],
    "doors": [],
    # asansor: acik alanin sol ust kosesinde
    "elevator": {"x": 1.0, "y": 1.0, "w": 2.0, "h": 2.0},
    "start": (4.0, 2.0),           # asansor agzindan 1m onde
    # acik alan: omurga yok (serbest gidis), ama tutarlilik icin ana giris hatti
    "spine": [],
}

FLOORS = [KAT1, KAT2, KAT3]

# ----------------------------------------------------------------------
# OLCEK: tum harita 2x buyutulur (robot ayni kalir -> bol manevra alani)
#   odalar 2x, koridor 2x, kapilar 3m (sabit, genis gecis)
# ----------------------------------------------------------------------
MAP_SCALE = 2.0          # harita buyutme faktoru
DOOR_WIDTH = 3.0         # kapi genisligi (m) - sabit, robot rahat gecsin

def _scale_floor(fl):
    s = MAP_SCALE
    for r in fl["rooms"]:
        for k in ("x", "y", "w", "h", "cx", "cy"):
            r[k] *= s
    for d in fl.get("doors", []):
        # konumu olcekle; kapi acikligini 3m'ye sabitle (dar kenarda)
        d["x"] *= s; d["y"] *= s; d["w"] *= s; d["h"] *= s
        # kapinin dar boyutunu (gecis genisligi) 3m yap
        if d["w"] < d["h"]:      # dikey kapi -> genislik w
            d["x"] += (d["w"] - DOOR_WIDTH)/2; d["w"] = DOOR_WIDTH
        else:                    # yatay kapi -> genislik h... aslinda gecis w
            d["x"] += (d["w"] - DOOR_WIDTH)/2 if d["w"] > DOOR_WIDTH else 0
            d["w"] = max(d["w"], DOOR_WIDTH)
    ev = fl.get("elevator")
    if ev:
        for k in ("x", "y", "w", "h"): ev[k] *= s
    if "start" in fl:
        fl["start"] = (fl["start"][0]*s, fl["start"][1]*s)
    fl["spine"] = [(x*s, y*s) for (x, y) in fl.get("spine", [])]
    for z in fl.get("zones", []):
        z["cx"] *= s; z["cy"] *= s
    return fl

for _fl in FLOORS:
    _scale_floor(_fl)

# ----------------------------------------------------------------------
# OCCUPANCY GRID + A*
# ----------------------------------------------------------------------
GRID_RES = 0.25          # metre/hucre

# ---- ARAC KINEMATIGI (Ackermann / direksiyonlu go-kart) — GERCEK OLCULER ----
#   Arkadan itis, on tekerlekler yone kirilir, YERINDE DONEMEZ, GERI VITES YOK.
#   Olculen: dingil 0.70m, maks direksiyon 45deg, robot boyu 1.2m
#   -> donus yaricapi R = dingil/tan(45) = 0.70m (donus capi 1.40m)
WHEELBASE   = 0.70                          # dingil mesafesi (on aks-arka aks), OLCULDU
STEER_MAX   = math.radians(45)              # maks direksiyon acisi, OLCULDU
MIN_RADIUS  = WHEELBASE / math.tan(STEER_MAX)   # = 0.70m (otomatik)
ROBOT_LEN   = 1.2                           # robot boyu (m), gorsel + manevra
LOOKAHEAD   = 1.6                            # pure pursuit ileri-bakis (m)

class GridMap:
    def __init__(self, floor):
        self.floor = floor
        # sinirlar
        xs, ys = [], []
        for r in floor["rooms"]:
            xs += [r["x"], r["x"] + r["w"]]; ys += [r["y"], r["y"] + r["h"]]
        self.x0, self.y0 = min(xs) - 1, min(ys) - 1
        self.x1, self.y1 = max(xs) + 1, max(ys) + 1
        self.W = int(math.ceil((self.x1 - self.x0) / GRID_RES))
        self.H = int(math.ceil((self.y1 - self.y0) / GRID_RES))
        # 1 = dolu (duvar), 0 = serbest
        self.grid = np.ones((self.H, self.W), dtype=np.uint8)
        for r in floor["rooms"]:
            self._carve(r["x"], r["y"], r["w"], r["h"])
        for d in floor.get("doors", []):
            self._carve(d["x"], d["y"], d["w"], d["h"])
        # duvara uzaklik haritasi (merkez hat tercihi icin)
        self._build_distmap()

    def _build_distmap(self):
        # her serbest hucrenin en yakin duvara uzakligi (hucre cinsinden)
        # basit 2-gecisli chamfer distance transform
        INF = 1e6
        d = np.where(self.grid == 0, INF, 0.0)   # duvar=0, serbest=INF
        H, W = d.shape
        # ileri gecis
        for y in range(H):
            for x in range(W):
                if d[y, x] == 0: continue
                m = d[y, x]
                if x > 0:      m = min(m, d[y, x-1] + 1)
                if y > 0:      m = min(m, d[y-1, x] + 1)
                if x > 0 and y > 0:   m = min(m, d[y-1, x-1] + 1.414)
                if x < W-1 and y > 0: m = min(m, d[y-1, x+1] + 1.414)
                d[y, x] = m
        # geri gecis
        for y in range(H-1, -1, -1):
            for x in range(W-1, -1, -1):
                if d[y, x] == 0: continue
                m = d[y, x]
                if x < W-1:    m = min(m, d[y, x+1] + 1)
                if y < H-1:    m = min(m, d[y+1, x] + 1)
                if x < W-1 and y < H-1: m = min(m, d[y+1, x+1] + 1.414)
                if x > 0 and y < H-1:   m = min(m, d[y+1, x-1] + 1.414)
                d[y, x] = m
        self.distmap = d   # buyuk = duvardan uzak (koridor ortasi)

    def _carve(self, x, y, w, h):
        c0 = self.w2c(x, y); c1 = self.w2c(x + w, y + h)
        cx0, cy0 = c0; cx1, cy1 = c1
        cx0, cx1 = sorted((max(0, cx0), min(self.W, cx1)))
        cy0, cy1 = sorted((max(0, cy0), min(self.H, cy1)))
        self.grid[cy0:cy1, cx0:cx1] = 0

    def w2c(self, x, y):
        return (int((x - self.x0) / GRID_RES), int((y - self.y0) / GRID_RES))

    def c2w(self, cx, cy):
        return (self.x0 + (cx + 0.5) * GRID_RES, self.y0 + (cy + 0.5) * GRID_RES)

    def free(self, cx, cy):
        return 0 <= cx < self.W and 0 <= cy < self.H and self.grid[cy, cx] == 0

    # A* (8-yonlu)
    def astar(self, start_w, goal_w):
        s = self.w2c(*start_w); g = self.w2c(*goal_w)
        s = self._nearest_free(s); g = self._nearest_free(g)
        if s is None or g is None:
            return None
        nbrs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        openh = [(0, s)]
        came = {}; gsc = {s: 0}
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == g:
                path = [cur]
                while cur in came:
                    cur = came[cur]; path.append(cur)
                return [self.c2w(*c) for c in reversed(path)]
            cx, cy = cur
            for dx, dy in nbrs:
                nx, ny = cx + dx, cy + dy
                if not self.free(nx, ny):
                    continue
                if dx and dy and not (self.free(cx+dx, cy) and self.free(cx, cy+dy)):
                    continue    # kose kesmeyi engelle
                step = 1.414 if (dx and dy) else 1.0
                # MERKEZ HAT CEZASI: duvara yakinsa pahali (ortadan gitsin)
                clear = self.distmap[ny, nx]
                wall_pen = max(0.0, 5.0 - clear) * 1.2   # 5 hucreye kadar ceza (guclu)
                ng = gsc[cur] + step + wall_pen
                nc = (nx, ny)
                if nc not in gsc or ng < gsc[nc]:
                    gsc[nc] = ng
                    f = ng + math.hypot(g[0]-nx, g[1]-ny)
                    heapq.heappush(openh, (f, nc))
                    came[nc] = cur
        return None

    def _nearest_free(self, c):
        cx, cy = c
        if self.free(cx, cy):
            return c
        for rad in range(1, 12):
            for dx in range(-rad, rad+1):
                for dy in range(-rad, rad+1):
                    if self.free(cx+dx, cy+dy):
                        return (cx+dx, cy+dy)
        return None

    # gorus hatti (Bresenham) - iki nokta arasi tum hucreler serbest mi
    def line_free(self, w0, w1, min_clear=0.0):
        """Iki nokta arasi tum hucreler serbest mi.
        min_clear > 0 ise, cizgi boyunca duvara uzaklik da bu esigin uzerinde olmali
        (boylece capraz kisayollar duvar dibinden gecemez -> robot ortadan gider)."""
        c0 = self.w2c(*w0); c1 = self.w2c(*w1)
        x0, y0 = c0; x1, y1 = c1
        dx = abs(x1-x0); dy = abs(y1-y0)
        sx = 1 if x0 < x1 else -1; sy = 1 if y0 < y1 else -1
        err = dx - dy
        clear_cells = min_clear / GRID_RES
        while True:
            if not self.free(x0, y0):
                return False
            if clear_cells > 0 and self.distmap[y0, x0] < clear_cells:
                return False        # duvara cok yakin -> bu kisayol gecersiz
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2*err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 <  dx: err += dx; y0 += sy

    # yol noktalarini koridor/gecit ORTASINA cek (distmap tepesine dogru)
    def centerize(self, path):
        if not path or len(path) < 3:
            return path
        out = [path[0]]
        for k in range(1, len(path)-1):
            cx, cy = self.w2c(*path[k])
            best = (cx, cy); bestd = self.distmap[cy, cx]
            # genis komsulukta en yuksek clearance'a kaydir (koridor tam ortasi)
            for dx in range(-6, 7):
                for dy in range(-6, 7):
                    nx, ny = cx+dx, cy+dy
                    if 0 <= nx < self.W and 0 <= ny < self.H and self.grid[ny, nx] == 0:
                        if self.distmap[ny, nx] > bestd:
                            bestd = self.distmap[ny, nx]; best = (nx, ny)
            out.append(self.c2w(*best))
        out.append(path[-1])
        # DUZ HIZALAMA: ardisik noktalar neredeyse ayni x veya y'deyse tam hizala
        # (koridorda dumduz gitsin, hafif egim olmasin)
        for k in range(1, len(out)-1):
            px, py = out[k-1]; x, y = out[k]
            if abs(x - px) < 0.6:   out[k] = (px, y)   # dikey segment -> x'i sabitle
            elif abs(y - py) < 0.6: out[k] = (x, py)   # yatay segment -> y'yi sabitle
        return out

    # yol sadelestirme (string pulling) - merkez hatti koruyarak
    def simplify(self, path):
        if not path or len(path) < 3:
            return path
        path = self.centerize(path)      # once merkez hatta cek
        # kisayol ancak duvardan guvenli mesafedeyse kabul (robot ortadan gitsin,
        # ama duz koridoru tek segmentte birlestirebilsin diye esik olcülü)
        CLEAR = 0.55     # metre - capraz duvar kesmeyi onler, duz gidisi bozmaz
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.line_free(path[i], path[j], min_clear=CLEAR):
                    break
                j -= 1
            out.append(path[j]); i = j
        return out


# ----------------------------------------------------------------------
# TURN-BY-TURN KOMUT URETIMI
# ----------------------------------------------------------------------
def heading(a, b):
    return math.atan2(b[1]-a[1], b[0]-a[0])

def dist(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def classify_turn(h_prev, h_new):
    d = h_new - h_prev
    while d > math.pi:  d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    deg = math.degrees(d)
    if abs(deg) < 25:      return "FORWARD"
    if abs(deg) > 155:     return "BACK"
    # y-down ekranda: pozitif = saat yonu = SAG
    return "RIGHT" if deg > 0 else "LEFT"

def build_commands(waypoints):
    """waypoints -> [(action, meters), ...]  ok yonu + mesafe icin."""
    cmds = []
    if len(waypoints) < 2:
        return [("ARRIVED", 0)]
    h_prev = heading(waypoints[0], waypoints[1])
    cmds.append(("FORWARD", dist(waypoints[0], waypoints[1])))
    for i in range(1, len(waypoints)-1):
        h = heading(waypoints[i], waypoints[i+1])
        turn = classify_turn(h_prev, h)
        if turn != "FORWARD":
            cmds.append((turn, 0))
        cmds.append(("FORWARD", dist(waypoints[i], waypoints[i+1])))
        h_prev = h
    cmds.append(("ARRIVED", 0))
    return cmds


# ----------------------------------------------------------------------
# HARITA WIDGET'I  (cizim + tiklama + robot animasyonu)
# ----------------------------------------------------------------------
class MapWidget(QtWidgets.QWidget):
    log_msg = QtCore.pyqtSignal(str)
    hud_msg = QtCore.pyqtSignal(str, str, float)   # action, renk, mesafe

    def __init__(self):
        super().__init__()
        self.setMinimumSize(760, 560)
        self.floor_idx = 0
        self.gm = GridMap(FLOORS[0])
        self.start = None          # (x,y) world
        self.goal = None
        self.start_name = ""
        self.goal_name = ""
        self.waypoints = []        # sadelestirilmis yol
        self.robot = None          # (x,y) robot anlik konum
        self.robot_hdg = 0.0
        self.seg_i = 0             # anlik hedef waypoint index
        self.seg_t = 0.0
        self.steer = 0.0           # anlik direksiyon acisi (rad, gorsel + HUD)
        self.moving = False
        self.returning = False
        self.wait_left = 0.0       # hedefte bekleme (RETURN)
        self.speed = 3.0           # m/s (simulasyon hizi)
        self.cmds = []

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)       # ~30 fps

        # ilk kat: robotu asansor onune yerlestir (acilista gorunur olsun)
        fl0 = FLOORS[0]
        self.start = fl0.get("start", None)
        self._home = self.start            # RETURN hedefi = asansor onu
        self._park_exit_dir = None         # park varisinda burnu bu yone doner (kapiya)
        self.start_name = "Asansor Onu"
        self.robot = self.start
        self.robot_hdg = 0.0

    # ---- kat degistir ----
    def set_floor(self, idx):
        self.floor_idx = idx
        self.gm = GridMap(FLOORS[idx])
        fl = FLOORS[idx]
        # varsayilan baslangic = asansor onu
        self.start = fl.get("start", None)
        self._home = self.start            # RETURN hedefi = asansor onu
        self.start_name = "Asansor Onu"
        self.goal = None; self.goal_name = ""
        self.waypoints = []
        self.robot = self.start
        self.robot_hdg = 0.0
        self.moving = self.returning = False
        self.cmds = []
        self.log_msg.emit(f"[KAT] {fl['name']} yuklendi.")
        self.log_msg.emit(f"[BASLANGIC] Asansor onu (varsayilan). "
                          f"SOL TIK = hedef,  SAG TIK = baslangic degistir.")
        self.update()

    # ---- world <-> screen ----
    def _fit(self):
        pad = 16
        span_x = self.gm.x1 - self.gm.x0
        span_y = self.gm.y1 - self.gm.y0
        sc = min((self.width()-2*pad)/span_x, (self.height()-2*pad)/span_y)
        ox = (self.width() - span_x*sc)/2 - self.gm.x0*sc
        oy = (self.height() - span_y*sc)/2 - self.gm.y0*sc
        return sc, ox, oy

    def w2s(self, x, y):
        sc, ox, oy = self._fit()
        return QtCore.QPointF(x*sc + ox, y*sc + oy)

    def s2w(self, px, py):
        sc, ox, oy = self._fit()
        return ((px - ox)/sc, (py - oy)/sc)

    # ---- tiklama: hangi oda/bolge? ----
    def _hit(self, wx, wy):
        fl = FLOORS[self.floor_idx]
        # acik alan bolgeleri
        for z in fl.get("zones", []):
            if math.hypot(wx - z["cx"], wy - z["cy"]) < 2.2:
                return z["name"], (z["cx"], z["cy"])
        # odalar (koridor haric hedef alinamaz ama baslangic olabilir)
        for r in fl["rooms"]:
            if r["x"] <= wx <= r["x"]+r["w"] and r["y"] <= wy <= r["y"]+r["h"]:
                if "KORIDOR" in r["name"]:
                    continue
                return r["name"], (r["cx"], r["cy"])
        return None, None

    def mousePressEvent(self, e):
        # sadece HAREKET halindeyken engelle (varista serbest)
        if self.moving or self.returning == "wait":
            self.log_msg.emit("[!] Robot hareket halinde, bekleyin.")
            return
        wx, wy = self.s2w(e.x(), e.y())
        name, center = self._hit(wx, wy)
        if not name:
            return

        # SAG TIK = baslangic degistir
        if e.button() == QtCore.Qt.RightButton:
            self.start = center; self.start_name = name
            self.robot = center; self.robot_hdg = 0.0
            self.goal = None; self.goal_name = ""
            self.waypoints = []; self.cmds = []
            self.returning = False
            self.log_msg.emit(f"[BASLANGIC] {name} olarak degistirildi (sag tik).")
            self.update()
            return

        # SOL TIK = hedef sec ve git
        # robot bir yere varmissa, oradan devam et (yeni baslangic = robot konumu)
        if self.robot is not None:
            self.start = self.robot
        if center == self.start or (self.robot and
                abs(center[0]-self.robot[0]) < 0.5 and abs(center[1]-self.robot[1]) < 0.5):
            self.log_msg.emit("[!] Zaten bu noktadasin. Baska hedef sec.")
            return
        self.goal = center; self.goal_name = name
        self.returning = False
        self.log_msg.emit(f"[HEDEF] {name}  -> yol hesaplaniyor...")
        self._plan()
        self.update()

    # ---- omurga (spine) uzerinde nokta projeksiyonu ----
    def _project_to_spine(self, pt, spine_segs):
        """pt'yi en yakin omurga segmentine dik indir -> (omurga_uzeri_nokta, seg_index, t)."""
        best = None; bestd = 1e9
        for si in range(len(spine_segs)):
            a, b = spine_segs[si]
            ax, ay = a; bx, by = b
            dx, dy = bx-ax, by-ay
            L2 = dx*dx + dy*dy
            if L2 < 1e-9: continue
            t = ((pt[0]-ax)*dx + (pt[1]-ay)*dy) / L2
            t = max(0.0, min(1.0, t))
            px, py = ax+t*dx, ay+t*dy
            d = math.hypot(pt[0]-px, pt[1]-py)
            if d < bestd:
                bestd = d; best = ((px, py), si, t)
        return best

    # iki dogru parcasinin kesisim noktasi (yoksa None)
    def _seg_intersection(self, s1, s2):
        (x1,y1),(x2,y2) = s1; (x3,y3),(x4,y4) = s2
        den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(den) < 1e-9: return None
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
        u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / den
        # kesisim her iki segment uzerinde mi (biraz tolerans)
        if -0.05 <= t <= 1.05 and -0.05 <= u <= 1.05:
            return (x1 + t*(x2-x1), y1 + t*(y2-y1))
        return None

    # ---- omurga tabanli yol: start -> omurga -> hedef kapisi -> oda ----
    def _plan_spine(self, spine_pairs):
        segs = spine_pairs
        s_proj = self._project_to_spine(self.start, segs)
        g_proj = self._project_to_spine(self.goal, segs)
        if not s_proj or not g_proj:
            return None
        s_on, s_si, s_t = s_proj
        g_on, g_si, g_t = g_proj

        wps = [self.start, s_on]                  # asansor onu -> omurgaya cik

        # omurga uzerinde yuru
        if s_si == g_si:
            pass                                  # ayni segment: direkt
        else:
            # farkli segment: KESISIM noktasindan gec (hac merkezi)
            inter = self._seg_intersection(segs[s_si], segs[g_si])
            if inter:
                wps.append(inter)                 # kesisimden don (koridorun ucuna gitme!)
            else:
                # kesismiyorsa segment uclarindan (nadir durum)
                if s_si < g_si: wps.append(segs[s_si][1])
                else:           wps.append(segs[s_si][0])
        wps.append(g_on)                          # hedef kapisi hizasi (omurga uzeri)
        # PARK: kapidan iceri+saga gir. Park noktasi oda ici, sagda.
        dx = self.goal[0] - g_on[0]; dy = self.goal[1] - g_on[1]
        dd = math.hypot(dx, dy)
        if dd > 1e-6:
            ux, uy = dx/dd, dy/dd                 # kapidan odaya birim vektor (iceri)
            rx_, ry_ = -uy, ux                     # SAG yan vektor
            depth = dd * 0.80                      # oda ICINE gir
            side  = min(2.5, dd * 0.20)            # saga kayma
            p_in  = (g_on[0] + ux*depth + rx_*side,
                     g_on[1] + uy*depth + ry_*side)
            p_park = (g_on[0] + ux*(depth*0.78) + rx_*side,
                      g_on[1] + uy*(depth*0.78) + ry_*side)
            wps.extend([p_in, p_park])
            # PARK ACISI: park noktasindan KAPIYA (g_on) dogru = cikisa donuk.
            # Robot varinca burnu bu yone doner -> RETURN'de duz cikar.
            self._park_exit_dir = math.atan2(g_on[1]-p_park[1], g_on[0]-p_park[0])
        else:
            wps.append(self.goal)
            self._park_exit_dir = None

        clean = [wps[0]]
        for w in wps[1:]:
            if dist(clean[-1], w) > 0.3:
                clean.append(w)
        return self._round_corners(clean, radius=MIN_RADIUS)

    def _round_corners(self, pts, radius=1.2):
        if len(pts) < 3:
            return pts
        out = [pts[0]]
        for i in range(1, len(pts)-1):
            a, b, c = pts[i-1], pts[i], pts[i+1]
            # b kosesine giren ve cikan yon vektorleri
            v1 = (b[0]-a[0], b[1]-a[1]); l1 = math.hypot(*v1)
            v2 = (c[0]-b[0], c[1]-b[1]); l2 = math.hypot(*v2)
            if l1 < 1e-6 or l2 < 1e-6:
                out.append(b); continue
            r = min(radius, l1*0.45, l2*0.45)
            # koseye r once ve r sonra nokta
            p_in  = (b[0]-v1[0]/l1*r, b[1]-v1[1]/l1*r)
            p_out = (b[0]+v2[0]/l2*r, b[1]+v2[1]/l2*r)
            out.append(p_in)
            out.append(b)        # kose tepesi (yay ortasi)
            out.append(p_out)
        out.append(pts[-1])
        return out

    # ---- yol planla ----
    def _plan(self, keep_heading=False):
        fl = FLOORS[self.floor_idx]
        spine = fl.get("spine", [])
        wps = None
        self._park_exit_dir = None                # varsayilan: park acisi yok

        # RETURN (home'a donus) ise park acisi uygulanmaz (asansor onu, kapi yok)
        return_trip = keep_heading

        if spine and len(spine) >= 2:
            segs = [(spine[i], spine[i+1]) for i in range(0, len(spine)-1, 2)]
            wps = self._plan_spine(segs)

        if not wps:
            raw = self.gm.astar(self.start, self.goal)
            if not raw:
                self.log_msg.emit("[HATA] Yol bulunamadi.")
                self.goal = None; self.goal_name = ""
                return
            wps = self.gm.simplify(raw)

        # PARK ACISI: robot varinca burnu CIKISA doner. A* yolu (Kat3) icin son
        # segment tersi; omurga yolu (Kat1/2) icin plan_spine icinde g_on ile
        # hesaplanip _park_exit_dir'e yazildi. Burada sadece A* fallback:
        if not return_trip and self._park_exit_dir is None and len(wps) >= 2:
            px, py = wps[-1]; qx, qy = wps[-2]
            self._park_exit_dir = math.atan2(qy-py, qx-px)
        if return_trip:
            self._park_exit_dir = None

        self.waypoints = wps
        self.cmds = build_commands(self.waypoints)
        total = sum(d for a, d in self.cmds if a == "FORWARD")
        self.log_msg.emit(f"[YOL] {len(self.waypoints)} nokta (ana hat), ~{total:.1f} m")
        for a, d in self.cmds:
            if a == "FORWARD":  self.log_msg.emit(f"    ↑ FORWARD {d:.1f} m")
            elif a == "LEFT":   self.log_msg.emit(f"    ← TURN LEFT")
            elif a == "RIGHT":  self.log_msg.emit(f"    → TURN RIGHT")
            elif a == "BACK":   self.log_msg.emit(f"    ↓ BACK")
            elif a == "ARRIVED":self.log_msg.emit(f"    ● VARILDI")
        # animasyonu baslat
        # keep_heading=True (RETURN): robot ISINLANMAZ - gercek konum ve acisinda
        # kalir, pure pursuit onu gercek Ackermann fizigiyle (yay cizerek) yola sokar.
        if not keep_heading:
            self.robot = self.waypoints[0]        # normal git: robot yol basina
            if len(self.waypoints) >= 2:
                self.robot_hdg = heading(self.waypoints[0], self.waypoints[1])
        # RETURN: self.robot ve self.robot_hdg DEGISMEZ (park noktasi/acisi korunur)
        self.seg_i = 0                            # yolun basindan takip et
        self.steer = 0.0
        self.moving = True; self.returning = False
        self.update()

    # ---- RETURN: hedefte bekle, sonra geri don ----
    def start_return(self, wait_s=3.0):
        if not self.waypoints or self.moving:
            self.log_msg.emit("[!] Once bir hedefe varilmali.")
            return
        if self.robot is None:
            return
        self.wait_left = wait_s
        self.returning = "wait"
        self.log_msg.emit(f"[RETURN] Hedefte {wait_s:.0f} sn beklenecek, sonra geri donulecek.")

    # robottan ileri, bir sonraki KESKIN donusun (kose) mesafesi
    def _dist_to_next_corner(self, rx, ry):
        wps = self.waypoints
        if not wps or self.seg_i >= len(wps) - 1:
            return 99.0
        # seg_i'den itibaren, yon degisiminin buyuk oldugu ilk nokta = kose
        acc = dist((rx, ry), wps[self.seg_i])
        for i in range(self.seg_i, len(wps) - 1):
            a = wps[i-1] if i > 0 else (rx, ry)
            b = wps[i]; c = wps[i+1]
            h1 = math.atan2(b[1]-a[1], b[0]-a[0])
            h2 = math.atan2(c[1]-b[1], c[0]-b[0])
            dh = abs((h2 - h1 + math.pi) % (2*math.pi) - math.pi)
            if dh > math.radians(30):              # 30 dereceden fazla donus = kose
                return acc
            if i + 1 < len(wps):
                acc += dist(wps[i], wps[i+1])
        return acc

    # yol uzerinde robottan 'la' mesafede ileri-bakis noktasi (pure pursuit)
    def _lookahead_point(self, rx, ry, la):
        wps = self.waypoints
        if not wps:
            return (rx, ry)
        # robot yaklastigi ara noktalari gec
        while (self.seg_i < len(wps) - 1 and
               dist((rx, ry), wps[self.seg_i]) < 0.5):
            self.seg_i += 1
        # robottan baslayarak yol boyunca la kadar ilerle, o noktayi dondur
        prev = (rx, ry)
        acc = 0.0
        for i in range(self.seg_i, len(wps)):
            seg_d = dist(prev, wps[i])
            if acc + seg_d >= la:
                remain = la - acc
                t = remain / seg_d if seg_d > 1e-6 else 0.0
                return (prev[0] + (wps[i][0]-prev[0])*t,
                        prev[1] + (wps[i][1]-prev[1])*t)
            acc += seg_d
            prev = wps[i]
        return wps[-1]

    # park noktasi: robotu ODA ICINE sokma. Kapi agzinda (g_on) durdur.
    # Robot koridora/omurgaya donuk kalir -> RETURN'de duz cikis, U-manevra YOK.
    # (Gercekte robot kapida durup gozlem/olcum yapar; iceri girmesi gerekmez.)
    def _park_near_door(self, g_on, room_center):
        # cok hafif oda tarafina (kapiyi gecmis gorunsun ama omurgaya yakin)
        dx = room_center[0] - g_on[0]; dy = room_center[1] - g_on[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return g_on
        depth = 1.0                               # kapidan sadece 1m iceri
        return (g_on[0] + dx/d*depth, g_on[1] + dy/d*depth)

    # PARK MANEVRASI: robot kapidan (g_on) girer, oda icinde bir yay cizerek
    # burnu CIKISA (kapiya) donuk park eder. Ackermann ileri yay ile doner,
    # yerinde donmez. RETURN'de robot zaten cikisa donuk -> duz cikar.
    def _park_maneuver(self, g_on, room_center):
        dx = room_center[0] - g_on[0]; dy = room_center[1] - g_on[1]
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return [g_on]
        ux, uy = dx/d, dy/d                        # kapidan odaya birim vektor
        px, py = -uy, ux                           # dik (yan) vektor
        R = MIN_RADIUS                             # donus yaricapi (0.7m)
        # robot iceri girer (2R derinlik), yana kayar, geri doner:
        # yarim daire yay -> burnu tam tersine (cikisa) doner
        depth = min(2.0*R + 0.5, d*0.7)            # oda icine girme derinligi
        side  = 1.6*R                              # yanal kayma (yay yaricapi)
        # yay noktalari: giris -> ic -> yan -> cikisa donuk park
        p1 = (g_on[0] + ux*depth,          g_on[1] + uy*depth)           # iceri gir
        p2 = (p1[0] + px*side,             p1[1] + py*side)              # yana kay (yay)
        p3 = (g_on[0] + ux*depth*0.5 + px*side, g_on[1] + uy*depth*0.5 + py*side)  # cikisa donuk
        # park noktasi: kapi yaninda, cikisa donuk. son iki nokta cikis yonu verir.
        p4 = (g_on[0] + px*side*0.8, g_on[1] + py*side*0.8)             # kapi yani, cikisa donuk
        return [p1, p2, p3, p4]

    # ---- animasyon adimi (Ackermann / pure pursuit, yerinde donmez, geri yok) ----
    def _tick(self):
        dt = 0.033
        if self.returning == "wait":
            self.wait_left -= dt
            self.hud_msg.emit("BEKLIYOR", C_ORANGE, max(0, self.wait_left))
            if self.wait_left <= 0:
                # GERI DONUS: park noktasindan asansor onune (_home) yeni yol.
                self.start = self.robot
                self.goal = self._home
                self._plan(keep_heading=True)      # robot konum/aci korunur
                self.returning = True
                # Robot park acisinda (kapiya donuk). RETURN yolu da kapidan/cikistan
                # basladigi icin robot zaten dogru yone yakin bakar -> duz cikar,
                # pure pursuit kalan farki yay ile duzeltir (tik donme yok).
                self.log_msg.emit("[RETURN] Baslangica donus (gercek fizik).")
            self.update(); return

        if not self.moving or not self.waypoints:
            return

        rx, ry = self.robot
        rth = self.robot_hdg
        goal = self.waypoints[-1]

        # hedefe yeterince yakin mi? -> PARK
        if dist((rx, ry), goal) < 0.45:
            self.moving = False
            self.robot = goal                     # hedefe tam otur
            self.steer = 0.0
            # PARK ACISI: robotu burnu KAPIYA (cikisa) donuk cevir. Robot durmus
            # durumda, bu guvenli bir yonelme (spin/carpma riski yok).
            # Kapi yonu = park noktasindan omurgaya (cikisa) dogru.
            if self._park_exit_dir is not None:
                self.robot_hdg = self._park_exit_dir
            if self.returning is True:
                self.hud_msg.emit("BASLANGICTA", C_GREEN, 0)
                self.log_msg.emit("[RETURN] Baslangica donuldu. ● PARK")
                self.returning = False
            else:
                self.hud_msg.emit("PARK", C_GREEN, 0)
                self.log_msg.emit("[VARIS] Park (burnu kapiya donuk). ● PARK")
            self.update(); return

        # --- PURE PURSUIT (basit, sabit lookahead) ---
        # Sabit kucuk lookahead: robot waypoint'leri sirayla, sadik takip eder.
        # Kose yumusatma noktalari zaten yayi olusturur; ekstra adaptasyon spin yapar.
        d_goal = dist((rx, ry), goal)
        LA = 1.5 if d_goal > 1.5 else max(0.6, d_goal)
        target = self._lookahead_point(rx, ry, LA)

        dx = target[0] - rx; dy = target[1] - ry
        ld = math.hypot(dx, dy)
        if ld < 1e-6:
            self.update(); return
        ly = math.sin(-rth) * dx + math.cos(-rth) * dy
        curv = 2.0 * ly / (ld * ld)
        delta = math.atan(WHEELBASE * curv)
        delta = max(-STEER_MAX, min(STEER_MAX, delta))
        self.steer = delta

        # --- BISIKLET MODELI kinematigi (arkadan itis, ileri) ---
        v = self.speed
        rth += (v / WHEELBASE) * math.tan(delta) * dt
        rx  += v * math.cos(rth) * dt
        ry  += v * math.sin(rth) * dt
        self.robot = (rx, ry)
        self.robot_hdg = rth

        # --- HUD: FORWARD / STEER LEFT / STEER RIGHT + kalan mesafe ---
        remain = dist((rx, ry), goal)
        deg = math.degrees(delta)
        if abs(deg) < 6:
            act, col = "FORWARD", C_ACC
        elif deg > 0:                       # y-down ekranda pozitif = saga kirilma
            act, col = "STEER RIGHT", C_ORANGE
        else:
            act, col = "STEER LEFT", C_ORANGE
        self.hud_msg.emit(act, col, remain)
        self.update()

    # ---- cizim ----
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(C_BG))
        sc, ox, oy = self._fit()
        fl = FLOORS[self.floor_idx]

        # odalar
        for r in fl["rooms"]:
            tl = self.w2s(r["x"], r["y"])
            w = r["w"]*sc; h = r["h"]*sc
            rect = QtCore.QRectF(tl.x(), tl.y(), w, h)
            col = QtGui.QColor(CAT_COL.get(r["cat"], C_FREE))
            p.setBrush(col)
            p.setPen(QtGui.QPen(QtGui.QColor(C_WALL), 2))
            p.drawRect(rect)
            # isim: odanin UST kismina (park alaninin ustundeki bosluga)
            if "KORIDOR" not in r["name"]:
                p.setPen(QtGui.QColor(C_TEXT))
                f = QtGui.QFont(FONT); f.setPixelSize(max(9, int(0.85*sc))); f.setBold(True)
                p.setFont(f)
                name_rect = QtCore.QRectF(tl.x(), tl.y()+4, w, max(16, 1.4*sc))
                p.drawText(name_rect, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
                           r["name"])
            else:
                # koridor ismi ortada, soluk
                p.setPen(QtGui.QColor(C_MUTE))
                f = QtGui.QFont(FONT); f.setPixelSize(max(8, int(0.7*sc))); p.setFont(f)
                p.drawText(rect, QtCore.Qt.AlignCenter, r["name"])
            # YESIL PARK ALANI (1m x 1.5m, oda merkezinde) - koridor haric
            if "KORIDOR" not in r["name"]:
                self._draw_park(p, r["cx"], r["cy"], sc)

        # acik alan bolgeleri icin de park alani
        for z in fl.get("zones", []):
            self._draw_park(p, z["cx"], z["cy"], sc)

        # kapilar (bosluk vurgusu)
        for d in fl.get("doors", []):
            tl = self.w2s(d["x"], d["y"])
            p.setBrush(QtGui.QColor(C_FREE)); p.setPen(QtCore.Qt.NoPen)
            p.drawRect(QtCore.QRectF(tl.x(), tl.y(), d["w"]*sc, d["h"]*sc))
            # esik cizgisi
            p.setPen(QtGui.QPen(QtGui.QColor(C_ACC), 1, QtCore.Qt.DotLine))
            p.drawLine(QtCore.QPointF(tl.x(), tl.y()),
                       QtCore.QPointF(tl.x()+d["w"]*sc, tl.y()))

        # SABIT ANA HAT (omurga) - degismez robot koridoru, soluk sari cizgi
        spine = fl.get("spine", [])
        if spine and len(spine) >= 2:
            segs = [(spine[i], spine[i+1]) for i in range(0, len(spine)-1, 2)]
            pen = QtGui.QPen(QtGui.QColor(255, 210, 60, 90), 3)
            pen.setDashPattern([3, 5])
            p.setPen(pen)
            for a, b in segs:
                p.drawLine(self.w2s(*a), self.w2s(*b))

        # ASANSOR (baslangic referansi)
        ev = fl.get("elevator")
        if ev:
            tl = self.w2s(ev["x"], ev["y"])
            r = QtCore.QRectF(tl.x(), tl.y(), ev["w"]*sc, ev["h"]*sc)
            p.setBrush(QtGui.QColor("#243038"))
            p.setPen(QtGui.QPen(QtGui.QColor(C_ACC), 2))
            p.drawRect(r)
            # asansor ikonu (yukari/asagi ok)
            p.setPen(QtGui.QPen(QtGui.QColor(C_ACC), 2))
            cxp, cyp = r.center().x(), r.center().y()
            p.drawLine(QtCore.QPointF(cxp, cyp-6), QtCore.QPointF(cxp, cyp+6))
            p.drawLine(QtCore.QPointF(cxp-4, cyp-2), QtCore.QPointF(cxp, cyp-6))
            p.drawLine(QtCore.QPointF(cxp+4, cyp-2), QtCore.QPointF(cxp, cyp-6))
            p.drawLine(QtCore.QPointF(cxp-4, cyp+2), QtCore.QPointF(cxp, cyp+6))
            p.drawLine(QtCore.QPointF(cxp+4, cyp+2), QtCore.QPointF(cxp, cyp+6))
            f = QtGui.QFont(FONT); f.setPixelSize(8); f.setBold(True); p.setFont(f)
            p.setPen(QtGui.QColor(C_ACC))
            p.drawText(QtCore.QRectF(r.x()-10, r.bottom(), r.width()+20, 12),
                       QtCore.Qt.AlignHCenter, "ASANSOR")

        # acik alan bolgeleri (hedef noktalari)
        for z in fl.get("zones", []):
            c = self.w2s(z["cx"], z["cy"])
            p.setPen(QtGui.QPen(QtGui.QColor(C_MUTE), 1, QtCore.Qt.DashLine))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(c, 2.2*sc, 2.2*sc)
            p.setPen(QtGui.QColor(C_TEXT))
            f = QtGui.QFont(FONT); f.setPixelSize(max(9, int(0.8*sc))); p.setFont(f)
            p.drawText(QtCore.QRectF(c.x()-4*sc, c.y()+2.2*sc, 8*sc, 2*sc),
                       QtCore.Qt.AlignHCenter, z["name"])

        # ROBOT HATTI (beyaz serit — robotun izleyecegi yol)
        if self.waypoints and len(self.waypoints) > 1:
            pts = [self.w2s(*wp) for wp in self.waypoints]
            # 1) kalin yari-saydam beyaz taban (serit genisligi)
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 55), 12,
                                QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
            for i in range(len(pts)-1):
                p.drawLine(pts[i], pts[i+1])
            # 2) orta beyaz kesikli cizgi (yol seridi hissi)
            pen = QtGui.QPen(QtGui.QColor(255, 255, 255, 230), 2.2)
            pen.setDashPattern([6, 6])
            p.setPen(pen)
            for i in range(len(pts)-1):
                p.drawLine(pts[i], pts[i+1])
            # 3) donus noktalari (beyaz kucuk daire)
            for c in pts:
                p.setBrush(QtGui.QColor(255, 255, 255, 200)); p.setPen(QtCore.Qt.NoPen)
                p.drawEllipse(c, 2.5, 2.5)

        # baslangic / hedef isaretleri
        if self.start:
            c = self.w2s(*self.start)
            p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 2)); p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(c, 9, 9)
            p.setPen(QtGui.QColor(C_GREEN)); f = QtGui.QFont(FONT); f.setBold(True); f.setPixelSize(11); p.setFont(f)
            p.drawText(QtCore.QRectF(c.x()-30, c.y()-24, 60, 14), QtCore.Qt.AlignCenter, "BASLANGIC")
        if self.goal:
            c = self.w2s(*self.goal)
            p.setPen(QtGui.QPen(QtGui.QColor(C_RED), 2)); p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(c, 9, 9)
            p.drawLine(QtCore.QPointF(c.x()-12, c.y()), QtCore.QPointF(c.x()+12, c.y()))
            p.drawLine(QtCore.QPointF(c.x(), c.y()-12), QtCore.QPointF(c.x(), c.y()+12))
            p.setPen(QtGui.QColor(C_RED)); f = QtGui.QFont(FONT); f.setBold(True); f.setPixelSize(11); p.setFont(f)
            p.drawText(QtCore.QRectF(c.x()-30, c.y()+12, 60, 14), QtCore.Qt.AlignCenter, "HEDEF")

        # robot (go-kart bozmasi, ustten gorunum)
        if self.robot:
            self._draw_robot(p, self.robot, self.robot_hdg, sc)

    def _draw_park(self, p, cx, cy, sc):
        # 1m x 1.5m yesil park/durma alani (oda merkezinde)
        pw, ph = 1.0*sc, 1.5*sc
        c = self.w2s(cx, cy)
        rect = QtCore.QRectF(c.x()-pw/2, c.y()-ph/2, pw, ph)
        # robot bu alanda mi? (varista dolsun)
        here = (self.robot is not None and not self.moving and
                abs(self.robot[0]-cx) < 0.6 and abs(self.robot[1]-cy) < 0.8)
        if here:
            p.setBrush(QtGui.QColor(0, 230, 120, 140))   # dolu yesil (varildi)
            p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 2))
        else:
            p.setBrush(QtGui.QColor(0, 230, 120, 40))     # soluk yesil
            p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 1, QtCore.Qt.DashLine))
        p.drawRect(rect)
        # kose isaretleri (park cebi hissi)
        p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 2))
        k = min(pw, ph) * 0.3
        for corner in [(rect.left(), rect.top(), 1, 1), (rect.right(), rect.top(), -1, 1),
                       (rect.left(), rect.bottom(), 1, -1), (rect.right(), rect.bottom(), -1, -1)]:
            x0, y0, sx, sy = corner
            p.drawLine(QtCore.QPointF(x0, y0), QtCore.QPointF(x0+sx*k, y0))
            p.drawLine(QtCore.QPointF(x0, y0), QtCore.QPointF(x0, y0+sy*k))

    def _draw_robot(self, p, pos, hdg, sc):
        c = self.w2s(*pos)
        p.save()
        p.translate(c); p.rotate(math.degrees(hdg))
        L = max(16, 1.4*sc); Wd = max(10, 0.9*sc)   # govde
        steer_deg = math.degrees(self.steer)
        # tekerlekler: arka duz, ON tekerlekler direksiyon acisiyla kirik
        p.setBrush(QtGui.QColor("#111")); p.setPen(QtCore.Qt.NoPen)
        wheels = [(-L*0.3, -Wd*0.62, 0),      (-L*0.3, Wd*0.62, 0),        # arka (duz)
                  ( L*0.3, -Wd*0.62, steer_deg), (L*0.3, Wd*0.62, steer_deg)]  # on (kirik)
        for dx, dy, wsteer in wheels:
            p.save()
            p.translate(dx, dy); p.rotate(wsteer)
            p.drawRoundedRect(QtCore.QRectF(-L*0.14, -Wd*0.16, L*0.28, Wd*0.32), 2, 2)
            p.restore()
        # govde
        p.setBrush(QtGui.QColor(C_ACC)); p.setPen(QtGui.QPen(QtGui.QColor("#053"), 1))
        p.drawRoundedRect(QtCore.QRectF(-L*0.5, -Wd*0.5, L, Wd), 3, 3)
        # yon ucu (on)
        p.setBrush(QtGui.QColor(C_GREEN)); p.setPen(QtCore.Qt.NoPen)
        tri = QtGui.QPolygonF([QtCore.QPointF(L*0.5, 0),
                               QtCore.QPointF(L*0.28, -Wd*0.32),
                               QtCore.QPointF(L*0.28,  Wd*0.32)])
        p.drawPolygon(tri)
        p.restore()

        # hareket yonu oku (robotun onunde)
        if self.moving:
            ax = pos[0] + math.cos(hdg)*1.6
            ay = pos[1] + math.sin(hdg)*1.6
            a = self.w2s(*pos); b = self.w2s(ax, ay)
            p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 2))
            p.drawLine(a, b)


# ----------------------------------------------------------------------
# ANA PENCERE
# ----------------------------------------------------------------------
class NavWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("INDOOR OTONOM DRIVE PLAN — Nukleer Santral")
        self.resize(1180, 700)
        self.setStyleSheet(f"background:{C_BG}; color:{C_TEXT}; font-family:{FONT};")

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10); root.setSpacing(10)

        # SOL: harita
        left = QtWidgets.QVBoxLayout()
        self.map = MapWidget()
        # kat butonlari
        floor_bar = QtWidgets.QHBoxLayout()
        self.floor_btns = []
        for i, fl in enumerate(FLOORS):
            b = QtWidgets.QPushButton(fl["name"].split("·")[0].strip())
            b.setCheckable(True)
            b.clicked.connect(lambda _, idx=i: self._pick_floor(idx))
            b.setStyleSheet(self._btn_qss())
            floor_bar.addWidget(b); self.floor_btns.append(b)
        self.floor_btns[0].setChecked(True)
        left.addLayout(floor_bar)
        left.addWidget(self.map, 1)
        root.addLayout(left, 3)

        # SAG: kontrol + log + HUD
        right = QtWidgets.QVBoxLayout(); right.setSpacing(8)

        title = QtWidgets.QLabel("◈ NAVIGASYON PLANLAYICI")
        title.setStyleSheet(f"color:{C_ACC}; font-size:15px; font-weight:700; letter-spacing:2px;")
        right.addWidget(title)

        info = QtWidgets.QLabel("1) Baslangic odasina tikla\n2) Hedef odaya tikla\n"
                                "→ Robot rotayi planlar ve gider.\nTekrar tikla = yeni tur.")
        info.setStyleSheet(f"color:{C_MUTE}; font-size:11px;")
        right.addWidget(info)

        # HUD: buyuk anlik komut + ok + mesafe
        self.hud = QtWidgets.QLabel("HAZIR")
        self.hud.setAlignment(QtCore.Qt.AlignCenter)
        self.hud.setFixedHeight(90)
        self.hud.setStyleSheet(f"background:{C_PANEL}; border:1px solid {C_WALL}; "
                               f"border-radius:10px; color:{C_ACC}; "
                               f"font-size:24px; font-weight:800; letter-spacing:2px;")
        right.addWidget(self.hud)

        # butonlar
        btns = QtWidgets.QHBoxLayout()
        self.btn_return = QtWidgets.QPushButton("↩ RETURN (bekle+don)")
        self.btn_return.clicked.connect(lambda: self.map.start_return(3.0))
        self.btn_return.setStyleSheet(self._btn_qss())
        self.btn_reset = QtWidgets.QPushButton("⟳ SIFIRLA")
        self.btn_reset.clicked.connect(lambda: self.map.set_floor(self.map.floor_idx))
        self.btn_reset.setStyleSheet(self._btn_qss())
        btns.addWidget(self.btn_return); btns.addWidget(self.btn_reset)
        right.addLayout(btns)

        # hiz ayari
        spd = QtWidgets.QHBoxLayout()
        spd.addWidget(QtWidgets.QLabel("Hiz (m/s):"))
        self.spd_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.spd_slider.setMinimum(1); self.spd_slider.setMaximum(8); self.spd_slider.setValue(3)
        self.spd_slider.valueChanged.connect(lambda v: setattr(self.map, "speed", float(v)))
        spd.addWidget(self.spd_slider)
        right.addLayout(spd)

        # komut/log
        right.addWidget(QtWidgets.QLabel("KOMUT GUNLUGU:"))
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet(f"background:{C_PANEL}; border:1px solid {C_WALL}; "
                               f"border-radius:8px; color:{C_TEXT}; "
                               f"font-family:monospace; font-size:11px;")
        right.addWidget(self.log, 1)

        root.addLayout(right, 2)

        # sinyaller
        self.map.log_msg.connect(self._log)
        self.map.hud_msg.connect(self._hud)
        self._log(f"[HAZIR] {FLOORS[0]['name']}. Baslangic odasini secin.")

    def _pick_floor(self, idx):
        for i, b in enumerate(self.floor_btns):
            b.setChecked(i == idx)
        self.map.set_floor(idx)

    def _log(self, msg):
        self.log.appendPlainText(msg)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _hud(self, action, color, meters):
        arrow = {"FORWARD": "↑", "BACK": "↓", "STEER LEFT": "↰", "STEER RIGHT": "↱",
                 "LEFT": "←", "RIGHT": "→", "BEKLIYOR": "⏸",
                 "PARK": "◉", "VARILDI": "●", "BASLANGICTA": "●"}.get(action, "•")
        if action == "FORWARD":
            txt = f"{arrow} {action}  {meters:.1f} m"
        elif action in ("STEER LEFT", "STEER RIGHT"):
            txt = f"{arrow} {action}  {meters:.1f} m"
        elif action in ("LEFT", "RIGHT", "BACK"):
            txt = f"{arrow} {action}"
        elif action == "BEKLIYOR":
            txt = f"{arrow} BEKLIYOR  {meters:.0f} s"
        else:
            txt = f"{arrow} {action}"
        self.hud.setText(txt)
        self.hud.setStyleSheet(f"background:{C_PANEL}; border:1px solid {color}; "
                               f"border-radius:10px; color:{color}; "
                               f"font-size:24px; font-weight:800; letter-spacing:2px;")

    def _btn_qss(self):
        return (f"QPushButton{{background:{C_PANEL}; color:{C_TEXT}; "
                f"border:1px solid {C_WALL}; border-radius:8px; padding:8px 10px; "
                f"font-size:12px; font-weight:600;}}"
                f"QPushButton:checked{{border:1px solid {C_ACC}; color:{C_ACC};}}"
                f"QPushButton:hover{{border:1px solid {C_ACC};}}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = NavWindow(); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
