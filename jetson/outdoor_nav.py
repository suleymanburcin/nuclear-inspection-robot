#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
outdoor_nav.py  —  Jetson tarafi outdoor GPS navigasyon (pure pursuit)
======================================================================
Akkuyu / Buyukeceli test alani icin serpantin (lawnmower) tarama.
ESP32'den USB seri ile gelen nav JSON'ini okur, Ackermann arac icin
direksiyon acisi (derece) + gaz (0-1) uretir.

Beklenen JSON satiri (1. ESP32 -> Jetson, ~1 Hz):
  {"ts":..,"joyX":..,...,"heading":138.7,"imuCal":3,
   "lat":36.15919,"lon":33.56161,"sats":9}
    lat/lon : konum (null ise fix yok)   sats: uydu sayisi
    heading : BNO055 yaw (manyetik kuzey)  imuCal: mag kalibrasyon 0-3
    (spd/cog GPS hiz/yon: su an yok; Leonardo'ya eklenince heading fuzyonu devreye girer)

Heading fuzyonu:
    spd >= SPD_FUSE (GPS hizi varsa) -> GPS cog (true north)
    aksi halde -> BNO heading + DECLINATION (manyetik -> true north)
    Su an spd yok -> hep BNO heading kullanilir (calisir; GPS cog sonra eklenir).

KURU TEST (servo baglamadan): araca sadece GPS+BNO tak, araci elde
gezdir, komut ciktilarini ekranda izle:
    python3 outdoor_nav.py /dev/ttyUSB0
Sadece waypoint listesini gormek (haritaya/mission'a gomek icin):
    python3 outdoor_nav.py
"""

import sys, json, math

# ----------------------------------------------------------------------
# TEST ALANI  (Buyukeceli / Akkuyu NGS - DOGRULANMIS ankraj koordinati)
#   ANCHOR = alanin SOL-ALT (guney-bati) kosesi.
#   Sahada gercek duz test noktana bu iki degeri guncelle, gerisi otomatik.
# ----------------------------------------------------------------------
ANCHOR_LAT = 36.1591933
ANCHOR_LON = 33.5616058

FIELD_W = 30.0        # m  tarama yonu (serit boyu, +Dogu)
FIELD_H = 20.0        # m  seritlere dik toplam mesafe (+Kuzey)
LANE_M  = 4.0         # m  serit araligi (GPS gurultusunun belirgin ustunde)

# ----------------------------------------------------------------------
# ARAC / NAVIGASYON PARAMETRELERI   (benzinli RC, ~1.20 x 1.00 m, Ackermann)
# ----------------------------------------------------------------------
WHEELBASE   = 0.80    # m  on-arka dingil mesafesi  <-- araci olcup gir
MAX_STEER   = 30.0    # derece  fiziksel direksiyon limiti
LOOKAHEAD   = 3.0     # m  pure pursuit bakis mesafesi (artir=yumusak / azalt=agresif)
WP_RADIUS   = 1.5     # m  bu mesafeye girince waypoint'e ulasti say
CRUISE      = 0.35    # 0-1 duz seyir gaz
SLOW        = 0.20    # 0-1 donuste / hedefe yakinken gaz
SPD_FUSE    = 0.5     # m/s ustunde heading = GPS cog, altinda = BNO yaw
DECLINATION = 5.5     # derece E  (Akkuyu ~ +5.5; KESIN degeri dogrula)
GEOFENCE    = 5.0     # m  alan disina bu kadar tasarsa DUR (emniyet)

M_PER_DEG_LAT = 111320.0
def m_per_deg_lon(lat_deg):
    return 111320.0 * math.cos(math.radians(lat_deg))

# ----------------------------------------------------------------------
# GEOMETRI  (kucuk alan -> duzlem/equirectangular yaklasimi yeterli)
# ----------------------------------------------------------------------
def local_offset(lat, lon):
    """Ankraja gore yerel metre ofseti -> (Dogu, Kuzey)."""
    dE = (lon - ANCHOR_LON) * m_per_deg_lon(ANCHOR_LAT)
    dN = (lat - ANCHOR_LAT) * M_PER_DEG_LAT
    return dE, dN

def dist_bearing(cur_lat, cur_lon, tgt_lat, tgt_lon):
    """Iki nokta arasi mesafe (m) ve yon (derece, 0=Kuzey, saat yonu)."""
    dE = (tgt_lon - cur_lon) * m_per_deg_lon(cur_lat)
    dN = (tgt_lat - cur_lat) * M_PER_DEG_LAT
    d = math.hypot(dE, dN)
    brg = math.degrees(math.atan2(dE, dN)) % 360.0
    return d, brg

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

# ----------------------------------------------------------------------
# WAYPOINT URETICI  (serpantin / lawnmower)  -> [(lat, lon), ...]
#   Koordinatlar HESAPLANIR (elle yazilmaz) -> tutarli, hatasiz.
# ----------------------------------------------------------------------
def make_lawnmower(lat0, lon0, width_m, height_m, lane_m):
    pts_local, y, flip = [], 0.0, False
    while y <= height_m + 1e-6:
        if not flip:
            pts_local += [(0.0, y), (width_m, y)]
        else:
            pts_local += [(width_m, y), (0.0, y)]
        flip = not flip
        y += lane_m
    dlat = 1.0 / M_PER_DEG_LAT
    dlon = 1.0 / m_per_deg_lon(lat0)
    return [(lat0 + n * dlat, lon0 + e * dlon) for (e, n) in pts_local]

WAYPOINTS = make_lawnmower(ANCHOR_LAT, ANCHOR_LON, FIELD_W, FIELD_H, LANE_M)

# ----------------------------------------------------------------------
# HEADING FUZYONU  ve  GEOFENCE
# ----------------------------------------------------------------------
def fused_heading(nav):
    # Bizim 1. ESP32 formati: heading (BNO yaw), imuCal. spd/cog GPS'ten
    # gelmiyor (Leonardo eklenince gelecek) -> yoksa BNO heading kullanilir.
    spd = nav.get("spd") or 0.0
    cog = nav.get("cog")                            # GPS gidis yonu (yoksa None)
    yaw = nav.get("heading")                        # BNO055 heading (bizde "heading")
    if spd >= SPD_FUSE and cog is not None and cog >= 0:
        return cog                                  # GPS true-north (hizliyken)
    if yaw is not None:
        return (yaw + DECLINATION) % 360.0          # BNO manyetik -> true north
    return None

def in_geofence(lat, lon):
    e, n = local_offset(lat, lon)
    return (-GEOFENCE <= e <= FIELD_W + GEOFENCE and
            -GEOFENCE <= n <= FIELD_H + GEOFENCE)

# ----------------------------------------------------------------------
# PURE PURSUIT  (Ackermann)  ->  step(nav) = (steer_deg, throttle, info)
#   steer_deg : - sol / + sag  (direksiyon servosuna maplenecek)
#   throttle  : 0-1            (gaz servosuna/ESC'ye maplenecek)
# ----------------------------------------------------------------------
class PurePursuit:
    def __init__(self, waypoints):
        self.wps = waypoints
        self.i = 0
        self.done = False

    def step(self, nav):
        if self.done or self.i >= len(self.wps):
            self.done = True
            return 0.0, 0.0, {"state": "DONE"}

        lat, lon = nav.get("lat"), nav.get("lon")
        # Bizim formatta "fix" alani yok: lat/lon null degilse fix var demektir.
        if lat is None or lon is None:
            return 0.0, 0.0, {"state": "NO_FIX", "sats": nav.get("sats", 0)}

        if not in_geofence(lat, lon):               # emniyet: alan disi -> DUR
            return 0.0, 0.0, {"state": "GEOFENCE_STOP"}

        hd = fused_heading(nav)
        if hd is None:
            return 0.0, 0.0, {"state": "NO_HEADING"}

        tgt_lat, tgt_lon = self.wps[self.i]
        d, brg = dist_bearing(lat, lon, tgt_lat, tgt_lon)

        if d <= WP_RADIUS:                          # waypoint'e ulasildi -> sonraki
            self.i += 1
            if self.i >= len(self.wps):
                self.done = True
                return 0.0, 0.0, {"state": "DONE"}
            tgt_lat, tgt_lon = self.wps[self.i]
            d, brg = dist_bearing(lat, lon, tgt_lat, tgt_lon)

        # --- pure pursuit direksiyon acisi ---
        alpha = math.radians(wrap180(brg - hd))     # heading ile hedef arasi aci
        Ld = max(LOOKAHEAD, d)                       # hedef yakinsa d kullan
        steer = math.degrees(math.atan2(2.0 * WHEELBASE * math.sin(alpha), Ld))
        steer = max(-MAX_STEER, min(MAX_STEER, steer))

        # buyuk aci veya hedefe yakin -> yavasla
        throttle = SLOW if (abs(math.degrees(alpha)) > 25 or d < 2 * WP_RADIUS) else CRUISE

        return steer, throttle, {
            "state": "RUN", "wp": self.i + 1, "dist": round(d, 1),
            "brg": round(brg, 1), "hd": round(hd, 1), "steer": round(steer, 1),
        }

# ----------------------------------------------------------------------
# KURU TEST DONGUSU  —  ESP32 nav satirlarini oku, komut ciktisini yazdir
#   (Servo YOK; sadece navigasyon mantigini sahada dogrulamak icin.)
# ----------------------------------------------------------------------
def dry_run(port):
    import serial
    ser = serial.Serial(port, 115200, timeout=2)
    pp = PurePursuit(WAYPOINTS)
    print(f"[NAV] {port} · {len(WAYPOINTS)} waypoint · kuru test (servo yok)")
    while True:
        raw = ser.readline().decode("utf-8", "ignore").strip()
        # Bizim 1. ESP32 JSON'u {"ts":..,...,"lat":..} formatinda.
        # lat iceren gecerli JSON satirlarini al (kalibrasyon/debug satirlarini atla).
        if not raw.startswith("{") or '"lat"' not in raw:
            continue
        try:
            nav = json.loads(raw)
        except json.JSONDecodeError:
            continue
        steer, throttle, info = pp.step(nav)
        st = info.get("state")
        if st == "RUN":
            print(f"WP{info['wp']:>2} d={info['dist']:>5}m  hedef={info['brg']:>5}  "
                  f"heading={info['hd']:>5}  -> direksiyon={steer:+5.1f}  gaz={throttle:.2f}")
        else:
            print(f"[{st}]  sats={info.get('sats','-')}  direksiyon={steer:+.1f} gaz={throttle:.2f}")
        if pp.done:
            print("[NAV] tarama tamam.")
            break

def print_waypoints():
    print(f"# Akkuyu/Buyukeceli test alani  {FIELD_W:.0f}x{FIELD_H:.0f} m  "
          f"serit {LANE_M:.0f} m  ->  {len(WAYPOINTS)} waypoint")
    print(f"# ankraj (SW kose): {ANCHOR_LAT:.7f}, {ANCHOR_LON:.7f}")
    for k, (la, lo) in enumerate(WAYPOINTS, 1):
        e, n = local_offset(la, lo)
        print(f"WP{k:>2}: {la:.7f}, {lo:.7f}   (E={e:5.1f} m, N={n:5.1f} m)")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        dry_run(sys.argv[1])
    else:
        print_waypoints()
