#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jetson_gas_dock.py  —  Futuristik gaz sensor gosterge paneli (kadranli)
=======================================================================
ESP32'den (USB seri) gelen JSON'i, ekranin SAG kenarina dikey "dock"
olarak yerlesen, her sensoru ARABA GOSTERGESI gibi 0-100 kadranla
gosteren siyah temali futuristik panel.

Kadran renkleri (guvenlik endeksi 0-100):
    YESIL = guvenli  ·  TURUNCU = dikkat  ·  KIRMIZI = tehlike

Zincir:  Leonardo --UART--> ESP32 --USB--> Jetson (bu uygulama)

JSON (saniyede bir):
  {"mq2":2233,"mq8":3199,"mq135":3695,"mq7":2937,"mics":807,
   "temp":27.9,"pres":1005.7,"hum":65.0}

Kurulum (Jetson Orin Nano):
  sudo apt install python3-pyqt5 fonts-orbitron
  pip3 install pyserial
Calistir:
  python3 jetson_gas_dock.py [/dev/ttyUSB0]
Cikis: Esc / Q
=======================================================================
"""

import sys, json, glob, math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import serial
import cv2
import numpy as np
try:
    import pyrealsense2 as rs
    HAS_RS = True
except Exception:
    HAS_RS = False
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except Exception:
    HAS_YOLO = False
from PyQt5 import QtCore, QtGui, QtWidgets

# ----------------------------------------------------------------------
# AYARLAR
# ----------------------------------------------------------------------
BAUD = 115200
PORT_CANDIDATES = ["/dev/ttyTHS1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
BASELINE_SAMPLES = 10          # acilista temiz-hava referansi icin ornek sayisi

# ---- DIREKSIYON ACI SINIRLARI (2.ESP32 firmware'i ile AYNI olmali) ----
STEER_STEPS_PER_REV = 400.0   # surucunun mikroadim ayari
STEER_GEAR          = 1.0     # motor mili -> direksiyon mili orani
STEER_DEG_RIGHT     = 15.0    # saga azami aci
STEER_DEG_LEFT      = 30.0    # sola azami aci
STEER_STEPS_RIGHT = int(STEER_DEG_RIGHT * STEER_STEPS_PER_REV * STEER_GEAR / 360.0 + 0.5)   # 17
STEER_STEPS_LEFT  = int(STEER_DEG_LEFT  * STEER_STEPS_PER_REV * STEER_GEAR / 360.0 + 0.5)   # 33

# Kamera (RealSense RGB node = /dev/video4)
CAM_INDEX = 4                  # v4l2 node; RGB burada dogrulandi (ON kamera)
CAM_REAR_INDEX = 6             # ARKA kamera (A4Tech USB - /dev/video6)
CAM_W, CAM_H = 1280, 720       # yakalama cozunurlugu (islem yuku icin makul)

# YOLO nesne tanima (sov)
YOLO_MODEL = "yolov8n.pt"      # nano: en hafif, Jetson icin uygun (ilk calistirmada indirilir)
YOLO_EVERY = 3                 # her N frame'de bir detection (performans icin)
YOLO_CONF  = 0.45             # min guven skoru

# MJPEG yayin (panel tamami -> PC tarayici)
STREAM_ENABLE = True
STREAM_PORT   = 8080          # PC'de: http://<jetson-ip>:8080
STREAM_W      = 800          # yayin genisligi (800x480 ekran = panel genisligi)
STREAM_QUALITY = 55          # JPEG kalite 1-100 (dusur -> az veri)
STREAM_FPS    = 12           # yayin fps ust siniri (agi bogmamak icin)

# Renkler
C_BG     = "#000000"
C_TILE   = "#05080e"
C_EDGE   = "#0e2233"
C_ACCENT = "#00e5ff"
C_TEXT   = "#dff6ff"
C_MUTED  = "#5b7a90"
C_GREEN  = "#00ff9c"
C_ORANGE = "#ff9f1c"
C_RED    = "#ff2e63"
C_TRACK  = "#101922"
C_NEEDLE = "#ffffff"

FONT = "Orbitron, 'Rajdhani', 'DejaVu Sans', 'Ubuntu', sans-serif"

# Kutu tanimi:
#  key, isim, ALGILADIGI (EN), birim, mod, p1, p2, g1, g2
#   mod "ratio" -> gaz: baseline orani. endeks=(oran-1)*100. p1/p2 kullanilmaz
#   mod "range" -> endeks=(deger-p1)/(p2-p1)*100
#   g1 = yesil/turuncu siniri, g2 = turuncu/kirmizi siniri  (0-100 endekste)
TILES = [
    ("mq2",   "MQ-2",       "LPG · Smoke · Methane",   "mV",  "ratio", 0,   0,    40, 70),
    ("mq8",   "MQ-8",       "Hydrogen (H2)",           "mV",  "ratio", 0,   0,    40, 70),
    ("mq135", "MQ-135",     "Air Quality · CO2 · NH3", "mV",  "ratio", 0,   0,    40, 70),
    ("mq7",   "MQ-7",       "Carbon Monoxide",         "mV",  "ratio", 0,   0,    40, 70),
    ("mics",  "MiCS-5524",  "CO · Alcohol · VOC",      "mV",  "ratio", 0,   0,    40, 70),
    ("temp",  "TEMP",       "Temperature",             "C",   "range", 0,   80,   50, 75),
    ("pres",  "PRESSURE",   "Barometric Pressure",     "hPa", "range", 950, 1050, 80, 95),
    ("hum",   "HUMIDITY",   "Relative Humidity",       "%",   "range", 0,   100,  75, 90),
    ("dist",  "MESAFE",     "Ultrasonik Su Sev./Engel","cm",  "range", 0,   200,  60, 85),
]

# Sensor bilgi metinleri (? butonu -> popup). Aciklama + esik araligi.
# Gaz sensorleri "ratio": temiz havaya gore % artis. Esik = g1/g2 (%).
# range sensorleri: gercek birimde esikler (p1/p2 araliginda g1/g2 % noktasi).
SENSOR_INFO = {
    "mq2":   ("MQ-2 — Yanıcı Gazlar",
              "Ölçtüğü: LPG, duman, metan, propan, bütan.\n"
              "Değer: temiz havaya göre % artış.\n"
              "Yeşil: <%40   Turuncu: %40–70   Kırmızı: >%70"),
    "mq8":   ("MQ-8 — Hidrojen",
              "Ölçtüğü: Hidrojen gazı (H₂).\n"
              "Değer: temiz havaya göre % artış.\n"
              "Yeşil: <%40   Turuncu: %40–70   Kırmızı: >%70"),
    "mq135": ("MQ-135 — Hava Kalitesi",
              "Ölçtüğü: CO₂, amonyak (NH₃), benzen, duman.\n"
              "Değer: temiz havaya göre % artış.\n"
              "Yeşil: <%40   Turuncu: %40–70   Kırmızı: >%70"),
    "mq7":   ("MQ-7 — Karbonmonoksit",
              "Ölçtüğü: Karbonmonoksit (CO).\n"
              "Değer: temiz havaya göre % artış.\n"
              "Yeşil: <%40   Turuncu: %40–70   Kırmızı: >%70"),
    "mics":  ("MiCS-5524 — VOC / CO",
              "Ölçtüğü: CO, alkol, uçucu organik bileşikler (VOC).\n"
              "Değer: temiz havaya göre % artış.\n"
              "Yeşil: <%40   Turuncu: %40–70   Kırmızı: >%70"),
    "temp":  ("TEMP — Sıcaklık (BME280)",
              "Ölçtüğü: Ortam sıcaklığı.\n"
              "Aralık: 0–80 °C.\n"
              "Yeşil: <50°C   Turuncu: 50–75°C   Kırmızı: >75°C"),
    "pres":  ("PRESSURE — Basınç (BME280)",
              "Ölçtüğü: Barometrik basınç.\n"
              "Aralık: 950–1050 hPa.\n"
              "Yeşil: <1030   Turuncu: 1030–1045   Kırmızı: >1045 hPa"),
    "hum":   ("HUMIDITY — Nem (BME280)",
              "Ölçtüğü: Bağıl nem.\n"
              "Aralık: %0–100.\n"
              "Yeşil: <%75   Turuncu: %75–90   Kırmızı: >%90"),
    "dist":  ("MESAFE — JSN-SR04T Ultrasonik",
              "Ölçtüğü: Mesafe / tank su seviyesi (su geçirmez).\n"
              "Aralık: 2–500 cm.\n"
              "Yakında engel/yüksek seviye = uyarı."),
}


# ----------------------------------------------------------------------
# SERI OKUYUCU
# ----------------------------------------------------------------------
class SerialReader(QtCore.QThread):
    data = QtCore.pyqtSignal(dict)
    status = QtCore.pyqtSignal(str, bool)

    def __init__(self, port=None):
        super().__init__()
        self.port = port
        self._run = True
        self._ser = None            # acik port referansi (yazma icin)
        self._lock = QtCore.QMutex()

    @staticmethod
    def _xor_checksum(line: str) -> str:
        """UART gurultusune karsi: 'M,...' -> 'M,...*7F'
        ESP32 tarafinda '*' oncesi tum karakterlerin XOR'u dogrulanir.
        Tutmazsa satir sessizce elenir (bozuk komut uygulanmaz)."""
        x = 0
        for ch in line:
            x ^= ord(ch)
        return f"{line}*{x:02X}"

    def send_cmd(self, line):
        # 2. ESP32'ye komut yaz (ttyTHS1 TX). Ornek: "M,40,0,20,0,90,0*3C"
        line = self._xor_checksum(line)
        self._lock.lock()
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.write((line + "\n").encode("ascii", "ignore"))
        except Exception:
            pass
        finally:
            self._lock.unlock()

    def _find_port(self):
        if self.port:
            return self.port
        for p in PORT_CANDIDATES:
            if glob.glob(p):
                return p
        found = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        return found[0] if found else None

    def run(self):
        while self._run:
            port = self._find_port()
            if not port:
                self.status.emit("NO PORT — is ESP32 plugged in?", False)
                self.msleep(1500); continue
            try:
                ser = serial.Serial(port, BAUD, timeout=2)
                self._lock.lock(); self._ser = ser; self._lock.unlock()
                self.status.emit(f"LINK · {port}", True)
            except Exception as e:
                self.status.emit(f"OPEN FAIL · {e}", False)
                self.msleep(1500); continue
            try:
                while self._run:
                    raw = ser.readline().decode("utf-8", "ignore").strip()
                    if not raw or not raw.startswith("{"):
                        continue
                    try:
                        obj = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    if "error" in obj:
                        self.status.emit("ESP32: NO DATA (3s)", False); continue
                    self.data.emit(obj)
                    self.status.emit(f"LIVE · {port}", True)
            except Exception as e:
                self.status.emit(f"DROPPED · {e}", False)
                try: ser.close()
                except Exception: pass
                self.msleep(1000)

    def stop(self):
        self._run = False
        self.wait(1500)


# ----------------------------------------------------------------------
# KAMERA OKUYUCU  —  RealSense (RGB + hizali DEPTH), ayri thread
#   RGB frame + merkez mesafe (metre) yayinlar.
#   pyrealsense2 yoksa OpenCV /dev/video4 RGB'ye duser (mesafe olmaz).
# ----------------------------------------------------------------------
class CameraReader(QtCore.QThread):
    frame = QtCore.pyqtSignal(np.ndarray)          # BGR renkli frame
    heat  = QtCore.pyqtSignal(np.ndarray)          # depth colormap (BGR)
    dist  = QtCore.pyqtSignal(float)               # merkez mesafe (m); <=0 = gecersiz
    status = QtCore.pyqtSignal(str, bool)

    def __init__(self, index=CAM_INDEX):
        super().__init__()
        self.index = index
        self._run = True
        # YOLO modeli (varsa yukle)
        self.yolo = None
        self._fc = 0                 # frame sayaci
        self._last_dets = []         # [(x1,y1,x2,y2,label,conf), ...] son detection
        if HAS_YOLO:
            try:
                self.yolo = YOLO(YOLO_MODEL)
                self.status_yolo = "YOLO ready"
            except Exception as e:
                self.yolo = None
                self.status_yolo = f"YOLO fail: {e}"
        else:
            self.status_yolo = "YOLO yok"

    # YOLO detection + her kutunun depth'ten mesafesi -> frame uzerine ciz
    def _draw_detections(self, color, depth, depth_scale):
        # her YOLO_EVERY frame'de bir gercek detection; arada son sonucu ciz
        self._fc += 1
        if self.yolo is not None and (self._fc % YOLO_EVERY == 0):
            try:
                res = self.yolo(color, conf=YOLO_CONF, verbose=False)[0]
                dets = []
                for b in res.boxes:
                    x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                    cls = int(b.cls[0]); conf = float(b.conf[0])
                    label = res.names.get(cls, str(cls))
                    dets.append((x1, y1, x2, y2, label, conf))
                self._last_dets = dets
            except Exception:
                pass

        # kutulari ciz (mesafe depth'ten)
        for (x1, y1, x2, y2, label, conf) in self._last_dets:
            # kutu merkezinin mesafesi
            mtxt = ""
            if depth is not None:
                mcx = np.clip((x1 + x2) // 2, 0, depth.shape[1]-1)
                mcy = np.clip((y1 + y2) // 2, 0, depth.shape[0]-1)
                patch = depth[max(0,mcy-3):mcy+4, max(0,mcx-3):mcx+4].astype(np.float32)
                patch = patch[patch > 0]
                if patch.size:
                    m = float(np.median(patch)) * depth_scale
                    mtxt = f"  {m:.2f}m"
            # kutu
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 200), 2)
            cap = f"{label} {conf:.2f}{mtxt}"
            (tw, th), _ = cv2.getTextSize(cap, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(color, (x1, y1 - th - 6), (x1 + tw + 6, y1), (0, 255, 200), -1)
            cv2.putText(color, cap, (x1 + 3, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return color

    # --- RealSense yolu (RGB + depth) ---
    def _run_realsense(self):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, CAM_W, CAM_H, rs.format.z16,  30)
        try:
            profile = pipe.start(cfg)
        except Exception as e:
            self.status.emit(f"RS START FAIL: {e}", False)
            return False

        align = rs.align(rs.stream.color)          # depth'i renkli goruntuye hizala
        # depth olcegi (birim -> metre)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.status.emit(f"CAM · RealSense · {self.status_yolo}", True)

        try:
            while self._run:
                frames = pipe.wait_for_frames(2000)
                frames = align.process(frames)
                cframe = frames.get_color_frame()
                dframe = frames.get_depth_frame()
                if not cframe:
                    continue
                color = np.asanyarray(cframe.get_data())      # BGR
                depth = np.asanyarray(dframe.get_data()) if dframe else None

                # --- YOLO nesne tanima + mesafe (frame uzerine ciz) ---
                color = self._draw_detections(color, depth, depth_scale)
                self.frame.emit(color)

                # merkez mesafe (ortadaki 5x5 alanin medyani -> gurultuye dayanikli)
                if dframe:
                    h, w = color.shape[:2]
                    cx, cy = w // 2, h // 2
                    patch = depth[max(0,cy-2):cy+3, max(0,cx-2):cx+3].astype(np.float32)
                    patch = patch[patch > 0]                  # 0 = olcum yok
                    if patch.size:
                        meters = float(np.median(patch)) * depth_scale
                        self.dist.emit(meters)
                    else:
                        self.dist.emit(-1.0)

                    # --- DEPTH ISI HARITASI (yakin kirmizi / uzak mavi) ---
                    # 0.2m - 4.0m arasini 0-255'e olcekle, JET colormap
                    dm = depth.astype(np.float32) * depth_scale       # metre
                    lo, hi = 0.2, 4.0
                    norm = np.clip((dm - lo) / (hi - lo), 0, 1)
                    norm = (1.0 - norm) * 255.0                       # yakin=255(kirmizi tarafi)
                    norm[dm <= 0] = 0                                 # olcum yok -> koyu
                    heat = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_JET)
                    heat[dm <= 0] = (30, 20, 15)                      # gecersiz pikseller koyu
                    self.heat.emit(heat)
        except Exception as e:
            self.status.emit(f"RS DROPPED: {e}", False)
        finally:
            try: pipe.stop()
            except Exception: pass
        return True

    # --- OpenCV yedek yolu (sadece RGB, mesafe yok) ---
    def _run_opencv(self):
        # Once verilen index'i dene, acilmaz/goruntu vermezse +1 dene (A4Tech video6/7 gibi)
        tried = []
        cap = None
        for idx in (self.index, self.index + 1):
            tried.append(idx)
            c = cv2.VideoCapture(idx, cv2.CAP_V4L2)   # Jetson'da V4L2 backend
            # MJPEG dene (cogu USB kamera bu formatta akici calisir)
            c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            c.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
            if c.isOpened():
                ok, _ = c.read()
                if ok:
                    cap = c; self.index = idx; break     # goruntu veren index'i kullan
            c.release()
        if cap is None:
            self.status.emit(f"CAM video{tried}: ACILAMADI", False)
            return
        self.status.emit(f"CAM · video{self.index} (RGB only)", True)
        self.dist.emit(-1.0)
        fail = 0
        while self._run:
            ok, f = cap.read()
            if not ok or f is None:
                fail += 1
                if fail > 30: break
                self.msleep(20); continue
            fail = 0
            self.frame.emit(f)
            self.msleep(15)
        cap.release()

    def run(self):
        while self._run:
            if HAS_RS and not getattr(self, "force_opencv", False):
                self._run_realsense()
            else:
                self._run_opencv()
            if self._run:
                self.msleep(800)               # koptuysa yeniden dene

    def stop(self):
        self._run = False
        self.wait(2000)


# ----------------------------------------------------------------------
# KAMERA GORUNTU WIDGET'I  (BGR frame -> QLabel, nisangah + mesafe overlay)
# ----------------------------------------------------------------------
class CameraView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(300, 220)
        self._last = None
        self._dist = -1.0                       # metre; <0 = gecersiz
        self._heading = None                    # 0-360 derece (BNO pusula); None = yok
        self._imu_cal = 0                       # 0-3 kalibrasyon
        self._lat = None                        # GPS enlem (None = fix yok)
        self._lon = None                        # GPS boylam
        self._sats = 0                          # uydu sayisi
        self.setText("KAMERA BEKLENIYOR…")
        self.setStyleSheet(f"background:{C_BG}; color:{C_MUTED}; font-family:{FONT}; font-size:11px;")

    def show_frame(self, bgr):
        self._last = bgr
        self._render()

    def set_dist(self, meters):
        self._dist = meters
        # frame zaten geliyorsa _render mesafeyi de basar

    def set_heading(self, deg, cal=0):
        # BNO055'ten gelen pusula yonu (0-360). None = veri yok.
        self._heading = deg
        self._imu_cal = cal

    def set_gps(self, lat, lon, sats=0):
        # GPS konumu (None = fix yok)
        self._lat = lat
        self._lon = lon
        self._sats = sats if sats else 0

    def _dist_color(self, m):
        # yakin = tehlike (kirmizi), orta = turuncu, uzak = yesil
        if m is None or m <= 0: return QtGui.QColor(C_MUTED)
        if m < 0.5:  return QtGui.QColor(C_RED)
        if m < 1.2:  return QtGui.QColor(C_ORANGE)
        return QtGui.QColor(C_GREEN)

    def _render(self):
        if self._last is None:
            return
        bgr = self._last
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        pix = pix.scaled(self.width(), self.height(),
                         QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)

        # --- overlay: nisangah + mesafe (pixmap uzerine ciz) ---
        p = QtGui.QPainter(pix)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        pw, ph = pix.width(), pix.height()
        cx, cy = pw / 2, ph / 2
        col = self._dist_color(self._dist)

        # nisangah (artı + kucuk halka) — 800x480 icin kompakt
        p.setPen(QtGui.QPen(col, 2))
        gap, ln = 4, 12
        p.drawLine(int(cx-gap-ln), int(cy), int(cx-gap), int(cy))
        p.drawLine(int(cx+gap), int(cy), int(cx+gap+ln), int(cy))
        p.drawLine(int(cx), int(cy-gap-ln), int(cx), int(cy-gap))
        p.drawLine(int(cx), int(cy+gap), int(cx), int(cy+gap+ln))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(QtCore.QPointF(cx, cy), 3, 3)

        # mesafe yazisi (nisangahin altinda)
        if self._dist is not None and self._dist > 0:
            txt = f"{self._dist:.2f} m"
        else:
            txt = "-- m"
        f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(max(13, int(ph*0.045)))
        p.setFont(f)
        # arka golge (okunurluk)
        p.setPen(QtGui.QColor(0, 0, 0))
        tr = QtCore.QRectF(cx - 90 + 1, cy + gap + ln + 4 + 1, 180, 28)
        p.drawText(tr, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop, txt)
        p.setPen(col)
        tr = QtCore.QRectF(cx - 90, cy + gap + ln + 4, 180, 28)
        p.drawText(tr, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop, txt)

        # --- PUSULA (sag ust kose) ---
        self._draw_compass(p, pw, ph)

        p.end()

        self.setPixmap(pix)

    # Kameranin sag ust kosesine dairesel pusula ciz. Heading'e gore doner.
    # N = kirmizi, E/S/W = cyan, ibre = sari. imuCal dusukse uyari.
    def _draw_compass(self, p, pw, ph):
        R = max(34, int(min(pw, ph) * 0.11))     # pusula yaricapi (ekrana gore)
        margin = 12
        ccx = pw - margin - R                     # merkez x (sag ust)
        ccy = margin + R                          # merkez y
        hd = self._heading

        p.save()
        p.setRenderHint(QtGui.QPainter.Antialiasing)

        # koyu yari-saydam disk (okunurluk)
        p.setPen(QtCore.Qt.NoPen)
        bg = QtGui.QColor(0, 0, 0); bg.setAlpha(140)
        p.setBrush(bg)
        p.drawEllipse(QtCore.QPointF(ccx, ccy), R + 4, R + 4)

        # dis halka
        ring = QtGui.QColor(C_ACCENT); ring.setAlpha(180)
        p.setPen(QtGui.QPen(ring, 2)); p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(QtCore.QPointF(ccx, ccy), R, R)

        if hd is None:
            # BNO yok: gri "IMU?" yaz
            p.setPen(QtGui.QColor(C_MUTED))
            f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(int(R*0.42))
            p.setFont(f)
            p.drawText(QtCore.QRectF(ccx-R, ccy-R, 2*R, 2*R),
                       QtCore.Qt.AlignCenter, "IMU?")
            p.restore()
            self._draw_gps(p, ccx, ccy, R)       # IMU yoksa da GPS goster
            return

        # kadran robotla birlikte doner: heading kadar ters cevir.
        # (robot saga donunce N ibresi sola kayar gibi gorunur)
        import math as _m
        def dir_point(ang_deg, rad):
            a = _m.radians(ang_deg - 90 - hd)     # ekran: -90 ustte, hd kadar don
            return (ccx + _m.cos(a) * rad, ccy + _m.sin(a) * rad)

        # ana yon harfleri (N kirmizi, digerleri cyan)
        marks = [("N", 0, C_RED), ("E", 90, C_ACCENT),
                 ("S", 180, C_ACCENT), ("W", 270, C_ACCENT)]
        f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(int(R*0.32))
        p.setFont(f)
        for label, ang, colr in marks:
            tx, ty = dir_point(ang, R * 0.72)
            # kisa cizgi (tik)
            ox, oy = dir_point(ang, R * 0.95)
            ix, iy = dir_point(ang, R * 0.78)
            p.setPen(QtGui.QPen(QtGui.QColor(colr), 2))
            p.drawLine(QtCore.QPointF(ox, oy), QtCore.QPointF(ix, iy))
            # harf
            p.setPen(QtGui.QColor(colr))
            p.drawText(QtCore.QRectF(tx-R*0.3, ty-R*0.3, R*0.6, R*0.6),
                       QtCore.Qt.AlignCenter, label)

        # KUZEY IBRESI: hep N yonunu gosterir (kirmizi ok)
        nx, ny = dir_point(0, R * 0.55)
        p.setPen(QtGui.QPen(QtGui.QColor(C_RED), max(2, int(R*0.06)),
                            QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(QtCore.QPointF(ccx, ccy), QtCore.QPointF(nx, ny))

        # merkez gobek
        p.setPen(QtCore.Qt.NoPen); p.setBrush(QtGui.QColor(C_ACCENT))
        p.drawEllipse(QtCore.QPointF(ccx, ccy), R*0.08, R*0.08)

        # ust ortada SABIT robot yonu ucgeni (robot hep yukari bakar = ilerisi)
        p.setBrush(QtGui.QColor(C_TEXT)); p.setPen(QtCore.Qt.NoPen)
        tri = QtGui.QPolygonF([
            QtCore.QPointF(ccx, ccy - R - 2),
            QtCore.QPointF(ccx - 5, ccy - R - 10),
            QtCore.QPointF(ccx + 5, ccy - R - 10),
        ])
        p.drawPolygon(tri)

        # heading derece yazisi (pusula altinda)
        p.setPen(QtGui.QColor(C_TEXT))
        f2 = QtGui.QFont("Orbitron"); f2.setBold(True); f2.setPixelSize(int(R*0.30))
        p.setFont(f2)
        deg_txt = f"{int(round(hd))}°"
        p.drawText(QtCore.QRectF(ccx-R, ccy+R+2, 2*R, R*0.6),
                   QtCore.Qt.AlignCenter, deg_txt)

        # kalibrasyon dusukse uyari (BNO gezdirilmeli)
        if self._imu_cal < 2:
            p.setPen(QtGui.QColor(C_ORANGE))
            f3 = QtGui.QFont("Orbitron"); f3.setBold(True); f3.setPixelSize(int(R*0.24))
            p.setFont(f3)
            p.drawText(QtCore.QRectF(ccx-R, ccy+R+R*0.6, 2*R, R*0.5),
                       QtCore.Qt.AlignCenter, "KALIBRE")

        p.restore()

        # --- GPS koordinati (pusulanin yaninda/altinda) ---
        self._draw_gps(p, ccx, ccy, R)

    # Pusulanin altina GPS enlem/boylam + uydu yaz
    def _draw_gps(self, p, ccx, ccy, R):
        p.save()
        # kalibre uyarisi varsa biraz daha asagi baslat
        y0 = ccy + R + (R*1.1 if self._imu_cal < 2 else R*0.6) + 6
        box_w = R * 3.0
        x0 = ccx + R - box_w                     # sag kenara hizali

        if self._lat is not None and self._lon is not None:
            lat_txt = f"{self._lat:.6f}"
            lon_txt = f"{self._lon:.6f}"
            sat_txt = f"SAT {self._sats}"
            col = C_GREEN if self._sats >= 4 else C_ORANGE
        else:
            lat_txt = "GPS yok"
            lon_txt = "fix bekleniyor"
            sat_txt = f"SAT {self._sats}"
            col = C_MUTED

        f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(max(11, int(R*0.24)))
        p.setFont(f)

        # koyu arka plan (okunurluk)
        lines = [f"LAT {lat_txt}", f"LON {lon_txt}", sat_txt]
        line_h = int(R * 0.42)
        bg = QtGui.QColor(0, 0, 0); bg.setAlpha(150)
        p.setPen(QtCore.Qt.NoPen); p.setBrush(bg)
        p.drawRoundedRect(QtCore.QRectF(x0-4, y0-2, box_w+8, line_h*3+6), 5, 5)

        for i, txt in enumerate(lines):
            c = col if i < 2 else (C_GREEN if self._sats >= 4 else C_ORANGE)
            # golge
            p.setPen(QtGui.QColor(0, 0, 0))
            p.drawText(QtCore.QRectF(x0+1, y0+i*line_h+1, box_w, line_h),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, txt)
            p.setPen(QtGui.QColor(c))
            p.drawText(QtCore.QRectF(x0, y0+i*line_h, box_w, line_h),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, txt)
        p.restore()

    def resizeEvent(self, e):
        self._render()
        super().resizeEvent(e)


# ----------------------------------------------------------------------
# DEPTH ISI HARITASI (kucuk panel) — yakin kirmizi / uzak mavi
# ----------------------------------------------------------------------
class HeatView(QtWidgets.QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(90, 60)               # 800x480 kompakt
        self._last = None
        self.setText("DEPTH…")
        self.setStyleSheet(f"background:{C_TILE}; color:{C_MUTED}; "
                           f"font-family:{FONT}; font-size:10px; "
                           f"border:1px solid {C_EDGE}; border-radius:8px;")

    def show_heat(self, bgr):
        self._last = bgr
        self._render()

    def _render(self):
        if self._last is None:
            return
        bgr = self._last
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        pix = pix.scaled(self.width(), self.height(),
                         QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.setPixmap(pix)

    def resizeEvent(self, e):
        self._render()
        super().resizeEvent(e)


class Gauge(QtWidgets.QWidget):
    START = 225.0        # baslangic acisi (sol-alt)
    SWEEP = 270.0        # toplam yay

    def __init__(self, g1, g2):
        super().__init__()
        self.g1 = g1          # yesil/turuncu siniri
        self.g2 = g2          # turuncu/kirmizi siniri
        self.value = None     # 0-100 endeks (None = veri yok)
        self.setMinimumSize(70, 54)               # 800x480: 3 sutun dar dock'a sigar

    def set_value(self, v):
        self.value = None if v is None else max(0.0, min(100.0, v))
        self.update()

    def _zone_color(self, v):
        if v is None:  return QtGui.QColor(C_MUTED)
        if v >= self.g2: return QtGui.QColor(C_RED)
        if v >= self.g1: return QtGui.QColor(C_ORANGE)
        return QtGui.QColor(C_GREEN)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        W, H = self.width(), self.height()
        side = min(W, H * 1.35)
        pw = side * 0.10
        r = QtCore.QRectF(0, 0, side - pw*2, side - pw*2)
        r.moveCenter(QtCore.QPointF(W/2, H*0.60))

        def arc(frm, to, color, width):
            a0 = self.START - frm/100.0*self.SWEEP
            span = -(to-frm)/100.0*self.SWEEP
            pen = QtGui.QPen(color, width, QtCore.Qt.SolidLine, QtCore.Qt.FlatCap)
            p.setPen(pen); p.drawArc(r, int(a0*16), int(span*16))

        # arka iz
        arc(0, 100, QtGui.QColor(C_TRACK), pw)
        # renkli bolgeler
        arc(0, self.g1,  QtGui.QColor(C_GREEN),  pw)
        arc(self.g1, self.g2, QtGui.QColor(C_ORANGE), pw)
        arc(self.g2, 100, QtGui.QColor(C_RED),   pw)

        cx, cy = r.center().x(), r.center().y()
        rad = r.width()/2

        # ibre
        if self.value is not None:
            ang = math.radians(self.START - self.value/100.0*self.SWEEP)
            nx = cx + math.cos(ang) * rad * 0.80
            ny = cy - math.sin(ang) * rad * 0.80
            pen = QtGui.QPen(QtGui.QColor(C_NEEDLE), max(2.0, side*0.020),
                             QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
            p.setPen(pen)
            p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(nx, ny))

        # gobek
        hub = self._zone_color(self.value)
        p.setPen(QtCore.Qt.NoPen); p.setBrush(hub)
        hr = side*0.045
        p.drawEllipse(QtCore.QPointF(cx, cy), hr, hr)

        # endeks sayisi (kadran ortasinda, ibrenin altinda)
        p.setPen(hub)
        f = QtGui.QFont("Orbitron"); f.setBold(True)
        f.setPixelSize(int(side*0.17)); p.setFont(f)
        txt = "--" if self.value is None else f"{int(round(self.value))}"
        tr = QtCore.QRectF(cx - rad, cy + rad*0.18, rad*2, side*0.24)
        p.drawText(tr, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter, txt)


# ----------------------------------------------------------------------
# TEK KUTU: isim + algilanan (EN) + kadran + ham deger
# ----------------------------------------------------------------------
class GaugeTile(QtWidgets.QFrame):
    """Kompakt SATIR: ● isim .......... değer  (?)   (kadran yok, 7 inc icin)"""
    def __init__(self, name, detect_en, unit, mode, p1, p2, g1, g2, key=None):
        super().__init__()
        self.unit = unit
        self.mode = mode
        self.p1, self.p2 = p1, p2
        self.g1, self.g2 = g1, g2
        self.key = key
        self.baseline = None
        self.samples = []

        self.setObjectName("tile")
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(8, 1, 5, 1); lay.setSpacing(6)

        self.dot = QtWidgets.QLabel("●"); self.dot.setObjectName("sdot")
        self.lbl_name = QtWidgets.QLabel(name); self.lbl_name.setObjectName("sname")
        # isim kirpilmasin + KESIN gorunur: stil + FONT dogrudan (CSS font-family'ye guvenme)
        self.lbl_name.setMinimumWidth(90)
        namef = QtGui.QFont(); namef.setPixelSize(14); namef.setBold(True)
        self.lbl_name.setFont(namef)
        self.lbl_name.setStyleSheet(f"color:{C_ACCENT}; background:transparent;")
        self.lbl_val  = QtWidgets.QLabel("--"); self.lbl_val.setObjectName("sval")
        self.lbl_val.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        valf = QtGui.QFont(); valf.setPixelSize(15); valf.setBold(True)
        self.lbl_val.setFont(valf)
        self.lbl_val.setStyleSheet(f"color:{C_TEXT}; background:transparent;")
        self.btn_info = QtWidgets.QToolButton(); self.btn_info.setText("?")
        self.btn_info.setObjectName("sinfo")
        self.btn_info.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_info.clicked.connect(self._show_info)

        lay.addWidget(self.dot)
        lay.addWidget(self.lbl_name)
        lay.addStretch(1)
        lay.addWidget(self.lbl_val)
        lay.addWidget(self.btn_info)

        self._set_dot(None)                          # baslangic: gri

    def _zone(self, idx):
        # idx = 0-100 endeks. g1/g2 esiklerine gore renk.
        if idx is None: return C_MUTED
        if idx >= self.g2: return C_RED
        if idx >= self.g1: return C_ORANGE
        return C_GREEN

    def _set_dot(self, idx):
        col = self._zone(idx)
        self.dot.setStyleSheet(f"#sdot{{color:{col}; font-size:14px;}}")
        # sadece renk (font explicit set edildi, dokunma)
        self.lbl_val.setStyleSheet(f"color:{col}; background:transparent;")

    def _show_info(self):
        title, body = SENSOR_INFO.get(self.key, (self.lbl_name.text(), "Bilgi yok."))
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle("Sensör Bilgisi")
        box.setText(title)
        box.setInformativeText(body)
        box.setIcon(QtWidgets.QMessageBox.Information)
        box.setStyleSheet(
            f"QMessageBox{{background:{C_BG};}} "
            f"QLabel{{color:{C_TEXT}; font-family:{FONT}; font-size:13px;}} "
            f"QPushButton{{background:{C_TILE}; color:{C_ACCENT}; "
            f"border:1px solid {C_EDGE}; border-radius:5px; padding:5px 14px;}}"
        )
        box.exec_()

    def _index(self, v):
        if self.mode == "range":
            if self.p2 == self.p1: return 0.0
            return (v - self.p1) / (self.p2 - self.p1) * 100.0
        # ratio (gaz)
        if self.baseline is None:
            self.samples.append(v)
            if len(self.samples) >= BASELINE_SAMPLES:
                self.baseline = sum(self.samples) / len(self.samples)
            return None                      # baseline hazir degil
        if self.baseline <= 0: return 0.0
        return (v / self.baseline - 1.0) * 100.0

    def _fmt(self, v, idx):
        # gaz (ratio): % artis goster. range: gercek birim.
        if self.mode == "ratio":
            if idx is None: return "kalibre…"
            sign = "+" if idx >= 0 else ""
            return f"{sign}{idx:.0f}%"
        if self.unit == "C":   return f"{v:.1f}°C"
        if self.unit == "hPa": return f"{v:.0f} hPa"
        if self.unit == "%":   return f"{v:.0f}%"
        if self.unit == "cm":  return f"{v:.0f} cm"
        return f"{v}"

    def update_value(self, v):
        if v is None:
            self.lbl_val.setText("--"); self._set_dot(None); return
        idx = self._index(v)
        self.lbl_val.setText(self._fmt(v, idx))
        self._set_dot(idx)


# ----------------------------------------------------------------------
# JOYSTICK RADAR + BUTON LED PANELI (QPainter)
#   joyX/joyY: 0-1023, merkez ~512.  btns: "CDEFK" -> "00000"/"10100"...
# ----------------------------------------------------------------------
class JoyPanel(QtWidgets.QFrame):
    BTN_NAMES = ["C", "D", "E", "F", "K"]
    ADC_MAX = 1023.0          # Leonardo 10-bit ADC
    CENTER  = 512.0           # bosta joystick merkezi

    def __init__(self):
        super().__init__()
        self.setObjectName("tile")
        self.jx = None        # 0-1023 (None = veri yok)
        self.jy = None
        self.btns = "00000"
        self.setMinimumHeight(64)             # 800x480: kompakt joystick

    def set_joy(self, jx, jy):
        self.jx = jx
        self.jy = jy
        self.update()

    def set_btns(self, s):
        # gelen string "00000" formatinda; eksikse tamamla
        if s is None:
            s = "00000"
        s = str(s).strip()
        s = (s + "00000")[:5]
        self.btns = s
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        W, H = self.width(), self.height()

        # --- Sol taraf: joystick radar scope ---
        scope = min(W * 0.52, H) - 16
        cx = 8 + scope / 2 + 6
        cy = H / 2
        rad = scope / 2

        # dis halka
        p.setPen(QtGui.QPen(QtGui.QColor(C_EDGE), 1.4))
        p.setBrush(QtGui.QColor(C_TRACK))
        p.drawEllipse(QtCore.QPointF(cx, cy), rad, rad)
        # ic halkalar
        p.setBrush(QtCore.Qt.NoBrush)
        p.setPen(QtGui.QPen(QtGui.QColor(C_EDGE), 1.0, QtCore.Qt.DashLine))
        p.drawEllipse(QtCore.QPointF(cx, cy), rad * 0.66, rad * 0.66)
        p.drawEllipse(QtCore.QPointF(cx, cy), rad * 0.33, rad * 0.33)
        # crosshair
        p.setPen(QtGui.QPen(QtGui.QColor(C_EDGE), 1.0))
        p.drawLine(QtCore.QPointF(cx - rad, cy), QtCore.QPointF(cx + rad, cy))
        p.drawLine(QtCore.QPointF(cx, cy - rad), QtCore.QPointF(cx, cy + rad))

        # nokta konumu
        if self.jx is not None and self.jy is not None:
            # 0-1023 -> -1..+1  (merkez 512)
            nx = (self.jx - self.CENTER) / self.CENTER
            ny = (self.jy - self.CENTER) / self.CENTER
            nx = max(-1.0, min(1.0, nx))
            ny = max(-1.0, min(1.0, ny))
            # ekranda Y yukari pozitif olsun diye ny ters
            px = cx + nx * rad * 0.9
            py = cy + ny * rad * 0.9
            # merkezden noktaya iz
            live = abs(nx) > 0.06 or abs(ny) > 0.06     # bosta mi?
            col = QtGui.QColor(C_ACCENT) if live else QtGui.QColor(C_GREEN)
            p.setPen(QtGui.QPen(col, 1.4))
            p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(px, py))
            # nokta (glow etkisi icin iki daire)
            p.setPen(QtCore.Qt.NoPen)
            g = QtGui.QColor(col); g.setAlpha(70)
            p.setBrush(g); p.drawEllipse(QtCore.QPointF(px, py), rad*0.18, rad*0.18)
            p.setBrush(col); p.drawEllipse(QtCore.QPointF(px, py), rad*0.09, rad*0.09)
        else:
            # veri yok
            p.setPen(QtGui.QColor(C_MUTED))
            f = QtGui.QFont("Orbitron"); f.setPixelSize(int(rad*0.22)); p.setFont(f)
            p.drawText(QtCore.QRectF(cx-rad, cy-rad, rad*2, rad*2),
                       QtCore.Qt.AlignCenter, "--")

        # scope etiketi
        p.setPen(QtGui.QColor(C_MUTED))
        f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(7); p.setFont(f)
        p.drawText(QtCore.QRectF(cx - rad, cy + rad + 1, rad*2, 11),
                   QtCore.Qt.AlignHCenter, "JOYSTICK")

        # --- Sag taraf: 5 buton LED (dikey) ---
        bx = cx + rad + 16
        avail = W - bx - 6
        n = len(self.BTN_NAMES)
        slot = H / n
        led_r = min(slot * 0.28, avail * 0.30, 8)
        for i, nm in enumerate(self.BTN_NAMES):
            yy = slot * (i + 0.5)
            on = (i < len(self.btns) and self.btns[i] == "1")
            # LED
            p.setPen(QtGui.QPen(QtGui.QColor(C_EDGE), 1.2))
            if on:
                glow = QtGui.QColor(C_GREEN); glow.setAlpha(90)
                p.setBrush(QtCore.Qt.NoPen if False else glow)
                p.setPen(QtCore.Qt.NoPen)
                p.drawEllipse(QtCore.QPointF(bx + led_r, yy), led_r*1.7, led_r*1.7)
                p.setPen(QtGui.QPen(QtGui.QColor(C_GREEN), 1.2))
                p.setBrush(QtGui.QColor(C_GREEN))
            else:
                p.setBrush(QtGui.QColor(C_TRACK))
            p.drawEllipse(QtCore.QPointF(bx + led_r, yy), led_r, led_r)
            # etiket
            p.setPen(QtGui.QColor(C_TEXT if on else C_MUTED))
            f = QtGui.QFont("Orbitron"); f.setBold(True); f.setPixelSize(9); p.setFont(f)
            p.drawText(QtCore.QRectF(bx + led_r*2 + 5, yy - 7, avail, 14),
                       QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, nm)


# ----------------------------------------------------------------------
# ANA DOCK (sag, dikey, 2 sutun)
# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
# MJPEG YAYIN  —  panel goruntusunu HTTP'den yayinla (PC tarayicidan izler)
# ----------------------------------------------------------------------
class FrameBus:
    """Panelin en son JPEG karesini thread-safe tutar."""
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None
    def set(self, jpeg_bytes):
        with self._lock:
            self._jpeg = jpeg_bytes
    def get(self):
        with self._lock:
            return self._jpeg

FRAME_BUS = FrameBus()

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):        # sessiz (konsolu kirletme)
        pass
    def do_GET(self):
        if self.path in ("/", "/stream", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            import time
            try:
                while True:
                    jpeg = FRAME_BUS.get()
                    if jpeg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(1.0 / max(1, STREAM_FPS))
            except (BrokenPipeError, ConnectionResetError):
                pass                  # PC tarayici kapandi -> sessizce bitir
        else:
            self.send_response(404); self.end_headers()

# =============================================================================
#  UZAKTAN KONTROL DINLEYICISI  (remote_control.py'den gelen UDP paketleri)
# =============================================================================
#  Paket bicimi: "joyX,joyY\n"   ornek: "512,512" (dur), "800,300" (ileri-sol)
#  Leonardo'nun urettigi bicimle BIREBIR AYNI (0..1023, merkez 512).
#
#  GUVENLIK KURALLARI (uc katman):
#    1) FIZIKSEL JOYSTICK ONCELIKLIDIR. Arac ustundeki operator kolu
#       merkezden ayirdigi anda uzak komut yok sayilir.
#    2) REMOTE_TIMEOUT icinde paket gelmezse uzak giris DUSER (dur).
#       Ag koparsa robot ilerlemeye devam etmez.
#    3) 2.ESP32'nin kendi CMD_TIMEOUT emniyeti (1500 ms) ucuncu katmandir.
#
#  Bu port, Bolum 4.1.2.9'da tanimlanan YALITILMIS yerel agda kalmalidir.
#  Internete acilmaz; port yonlendirmesi tanimlanmaz.
REMOTE_PORT    = 9000
REMOTE_TIMEOUT = 0.8      # saniye; bu sureden eski paket gecersiz sayilir


class RemoteJoyListener(threading.Thread):
    """UDP 9000'i dinler, son gecerli joystick degerini tutar."""

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._jx = None
        self._jy = None
        self._ts = 0.0
        self._pkts = 0
        self._sock = None
        self.error = None

    def run(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", REMOTE_PORT))
            self._sock.settimeout(0.5)
        except Exception as e:
            self.error = str(e)
            return
        while True:
            try:
                data, _addr = self._sock.recvfrom(64)
            except socket.timeout:
                continue
            except Exception:
                continue
            try:
                txt = data.decode("ascii", "ignore").strip()
                if not txt:
                    continue
                xs, ys = txt.split(",")[:2]
                jx = max(0, min(1023, int(xs)))
                jy = max(0, min(1023, int(ys)))
            except Exception:
                continue                      # bozuk paket -> sessizce at
            with self._lock:
                self._jx, self._jy = jx, jy
                self._ts = time.time()
                self._pkts += 1

    def get(self):
        """Taze paket varsa (jx, jy), yoksa (None, None)."""
        with self._lock:
            if self._jx is None:
                return (None, None)
            if time.time() - self._ts > REMOTE_TIMEOUT:
                return (None, None)           # bayat -> uzak giris duser
            return (self._jx, self._jy)

    @property
    def packets(self):
        with self._lock:
            return self._pkts


def start_mjpeg_server():
    try:
        from http.server import ThreadingHTTPServer
        srv = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), MJPEGHandler)
    except Exception:
        srv = HTTPServer(("0.0.0.0", STREAM_PORT), MJPEGHandler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


class GasDock(QtWidgets.QWidget):
    def __init__(self, port=None):
        super().__init__()
        # TAM EKRAN, cerceve yok, SIYAH OPAK (translucent KALDIRILDI)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.setStyleSheet(f"background:{C_BG};")     # tam siyah zemin

        # --- Motor/kol kontrol durumu ---
        self.arm_mode = False           # False=Direksiyon, True=Kol
        self.electric_mode = False      # False=Benzinli, True=Elektrikli
        self.rear_cam_active = False    # False=On kamera, True=Arka kamera (kol)
        # elektrikli mod icin son joystick degeri + duzenli gonderim
        self._last_jx = 512; self._last_jy = 512; self._last_btns = "00000"
        self._last_dir = 0              # son direksiyon step (kol modunda sabit)
        self._last_elec = None          # son elektrikli komut (tekrar yollamamak icin)

        # Elektrikli robot icin UDP broadcast soketi (Deneyap Kart'a)
        self.UDP_PORT = 4210
        try:
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            self.udp = None
        self._last_arm_base = 0         # son kol taban step (direksiyon modunda sabit)
        self._elbow = 90               # dirsek servo (orta baslar)
        self._grip = 0                 # parmak servo (kapali baslar)
        # Elektrikli robot 4 servo (D4,D5,D6,D7): [s1,s2,s3,s4]
        self._elec_servo = [90, 90, 0]   # 3 servo: en alt, bir ust, kiskac(kapali)

        self._build_layout()
        # Elektrikli mod duzenli komut gonderimi (10 Hz - gecikme azaltma)
        self._elec_timer = QtCore.QTimer(self)
        self._elec_timer.timeout.connect(self._elec_tick)
        self._elec_timer.start(100)     # her 100 ms
        self._toggle_mode()             # baslangic: direksiyon modu stili
        self._toggle_power()            # baslangic: benzinli modu stili
        self._toggle_camera()           # baslangic: on kamera stili
        self._go_fullscreen()

        # --- Seri (gaz/joystick) ---
        self.reader = SerialReader(port)
        self.reader.data.connect(self.on_data)
        self.reader.status.connect(self.on_status)
        self.reader.start()

        # ---- UZAKTAN KONTROL: UDP dinleyici (remote_control.py) ----
        self.remote = RemoteJoyListener()
        self.remote.start()
        self._remote_active = False      # durum cubugu icin

        # --- On Kamera (RealSense ya da index 4) ---
        self.cam = CameraReader(CAM_INDEX)
        self.cam.frame.connect(self._on_front_frame)
        self.cam.heat.connect(self.heatview.show_heat)
        self.cam.dist.connect(self.camview.set_dist)
        self.cam.status.connect(self.on_cam_status)
        self.cam.start()

        # --- Arka Kamera (kolu gormek icin, USB - RealSense DEGIL) ---
        self.cam_rear = CameraReader(CAM_REAR_INDEX)
        self.cam_rear.force_opencv = True         # arka kamera her zaman OpenCV (depth yok)
        self.cam_rear.frame.connect(self._on_rear_frame)
        self.cam_rear.start()

        # --- MJPEG yayin (panel tamami -> PC) ---
        if STREAM_ENABLE:
            try:
                self._mjpeg = start_mjpeg_server()
                ip = self._my_ip()
                print(f"[STREAM] PC'de izle: http://{ip}:{STREAM_PORT}")
            except Exception as e:
                print(f"[STREAM] baslatilamadi: {e}")
            # paneli periyodik yakala -> JPEG -> FRAME_BUS
            self._stream_timer = QtCore.QTimer(self)
            self._stream_timer.timeout.connect(self._grab_panel)
            self._stream_timer.start(int(1000 / max(1, STREAM_FPS)))

    def _my_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close()
            return ip
        except Exception:
            return "<jetson-ip>"

    def _grab_panel(self):
        # panelin tamamini yakala -> kucult -> JPEG -> yayin bus'a koy
        try:
            pix = self.grab()                       # tum widget (panel) goruntusu
            img = pix.toImage().convertToFormat(QtGui.QImage.Format_RGB888)
            w, h = img.width(), img.height()
            ptr = img.constBits(); ptr.setsize(img.byteCount())
            arr = np.frombuffer(ptr, np.uint8).reshape((h, img.bytesPerLine()))
            arr = arr[:, :w*3].reshape((h, w, 3))   # satir hizalamasini kirp
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            # kucult (yayin genisligine)
            if w > STREAM_W:
                nh = int(h * STREAM_W / w)
                bgr = cv2.resize(bgr, (STREAM_W, nh), interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode(".jpg", bgr,
                                    [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY])
            if ok:
                FRAME_BUS.set(jpeg.tobytes())
        except Exception:
            pass

    # ---- Ana yerlesim: SOL kamera (buyuk) + SAG dock (dar) ----
    def _build_layout(self):
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # SOL: kamera
        self.camview = CameraView()
        outer.addWidget(self.camview, 1)          # esner, buyuk pay

        # SAG: sensor dock (7 inc 800x480 icin dar sabit genislik)
        dock = QtWidgets.QWidget()
        dock.setObjectName("dockpanel")
        dock.setFixedWidth(300)                    # 800px ekran: kameraya ~500px kalir
        self._build_dock(dock)
        outer.addWidget(dock)

        self.setStyleSheet(self._qss())

    # ---- Sag dock icerigi: ust=isi haritasi, alt=gaz + joystick ----
    def _build_dock(self, dock):
        root = QtWidgets.QVBoxLayout(dock)
        root.setContentsMargins(5, 3, 5, 4); root.setSpacing(3)   # 480px: sikisik

        # --- ust: depth isi haritasi (kadranlar kalkti -> BUYUK) ---
        heat_lbl = QtWidgets.QLabel("◈  DEPTH  ·  near red/far blue")
        heat_lbl.setObjectName("title2")
        self.heatview = HeatView()
        self.heatview.setFixedHeight(72)           # mod butonu + mesafe icin yer

        # --- baslik + durum ---
        top = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("◈  GAS ARRAY"); title.setObjectName("title")
        self.dot = QtWidgets.QLabel("●"); self.dot.setObjectName("dot")
        self.status = QtWidgets.QLabel("BOOTING…"); self.status.setObjectName("status")
        self.status.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        # --- PROGRAMI KAPAT butonu (SAG EN UST) ---
        self.btn_quit = QtWidgets.QPushButton("✕")
        self.btn_quit.setObjectName("quitbtn")
        self.btn_quit.setFixedSize(38, 32)
        self.btn_quit.setStyleSheet(
            f"#quitbtn{{background:{C_RED}; color:#fff; font-weight:700;"
            f" border:none; border-radius:6px; font-size:16px;}}"
            f"#quitbtn:pressed{{background:#c01048;}}")
        self.btn_quit.clicked.connect(self._quit_app)
        top.addWidget(title); top.addStretch(1)
        top.addWidget(self.dot); top.addWidget(self.status)
        top.addWidget(self.btn_quit)

        # --- gaz + BME: SATIR listesi (● isim  deger  ?) ---
        rows = QtWidgets.QVBoxLayout()
        rows.setSpacing(1)
        self.tiles = {}
        for (key, name, det, unit, mode, p1, p2, g1, g2) in TILES:
            t = GaugeTile(name, det, unit, mode, p1, p2, g1, g2, key=key)
            self.tiles[key] = t
            rows.addWidget(t)

        # --- joystick + buton ---
        self.joy = JoyPanel()
        joy_lbl = QtWidgets.QLabel("◈  MANUAL INPUT"); joy_lbl.setObjectName("title")

        # --- MOD TOGGLE butonu (dokunmatik): Direksiyon <-> Kol ---
        self.btn_mode = QtWidgets.QPushButton("🚗  DİREKSİYON")
        self.btn_mode.setObjectName("modebtn")
        self.btn_mode.setCheckable(True)
        self.btn_mode.setMinimumHeight(38)
        self.btn_mode.clicked.connect(self._toggle_mode)

        # --- GUC TOGGLE butonu (dokunmatik): Benzinli <-> Elektrikli ---
        self.btn_power = QtWidgets.QPushButton("⛽ BENZİNLİ")
        self.btn_power.setObjectName("powerbtn")
        self.btn_power.setCheckable(True)
        self.btn_power.setMinimumHeight(38)
        self.btn_power.clicked.connect(self._toggle_power)

        # --- KAMERA TOGGLE butonu (dokunmatik): On <-> Arka (kol) ---
        self.btn_cam = QtWidgets.QPushButton("📷 ÖN")
        self.btn_cam.setObjectName("cambtn")
        self.btn_cam.setCheckable(True)
        self.btn_cam.setMinimumHeight(38)
        self.btn_cam.clicked.connect(self._toggle_camera)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(4)
        btn_row.addWidget(self.btn_mode)
        btn_row.addWidget(self.btn_power)
        btn_row.addWidget(self.btn_cam)

        self.cam_status = QtWidgets.QLabel("CAM: …"); self.cam_status.setObjectName("status")

        root.addWidget(heat_lbl)
        root.addWidget(self.heatview)
        root.addLayout(top)
        root.addLayout(rows, 1)
        root.addWidget(joy_lbl)
        root.addLayout(btn_row)                 # iki toggle yan yana
        root.addWidget(self.joy)
        root.addWidget(self.cam_status)

    def _toggle_power(self):
        self.electric_mode = self.btn_power.isChecked()
        if self.electric_mode:
            self.btn_power.setText("🔋 ELEKTRİKLİ")
            self.btn_power.setStyleSheet(
                f"#powerbtn{{background:{C_GREEN}; color:#000; font-weight:700;"
                f" border:2px solid {C_GREEN}; border-radius:7px; font-size:13px;}}")
        else:
            self.btn_power.setText("⛽ BENZİNLİ")
            self.btn_power.setStyleSheet(
                f"#powerbtn{{background:{C_TILE}; color:{C_TEXT}; font-weight:700;"
                f" border:2px solid {C_EDGE}; border-radius:7px; font-size:13px;}}")

    def _toggle_mode(self):
        self.arm_mode = self.btn_mode.isChecked()
        if self.arm_mode:
            self.btn_mode.setText("🦾  ROBOT KOL")
            self.btn_mode.setStyleSheet(
                f"#modebtn{{background:{C_ORANGE}; color:#000; font-weight:700;"
                f" border:2px solid {C_ORANGE}; border-radius:7px; font-size:14px;}}")
        else:
            self.btn_mode.setText("🚗  DİREKSİYON")
            self.btn_mode.setStyleSheet(
                f"#modebtn{{background:{C_TILE}; color:{C_ACCENT}; font-weight:700;"
                f" border:2px solid {C_ACCENT}; border-radius:7px; font-size:14px;}}")

    def _qss(self):
        return f"""
        QWidget {{ font-family: {FONT}; color: {C_TEXT}; }}
        #dockpanel {{
            background: {C_BG};
            border-left: 1px solid {C_ACCENT};
        }}
        #title {{ color:{C_ACCENT}; font-size:12px; font-weight:700; letter-spacing:1px; }}
        #title2 {{ color:{C_ACCENT}; font-size:9px; font-weight:600; letter-spacing:1px; }}
        #status {{ color:{C_MUTED}; font-size:9px; letter-spacing:1px; }}
        #dot {{ color:{C_RED}; font-size:11px; }}
        QFrame#tile {{
            background:{C_TILE}; border:1px solid {C_EDGE}; border-radius:6px;
        }}
        /* SATIR sensor: nokta + isim + deger + ? */
        #sdot  {{ color:{C_MUTED}; font-size:14px; }}
        #sname {{ color:{C_ACCENT}; font-size:14px; font-weight:700; }}
        #sval  {{ color:{C_TEXT};  font-size:15px; font-weight:700; }}
        #sinfo {{
            color:{C_ACCENT}; background:{C_BG}; border:1px solid {C_EDGE};
            border-radius:9px; font-size:12px; font-weight:700;
            min-width:18px; max-width:18px; min-height:18px; max-height:18px;
        }}
        #sinfo:hover {{ background:{C_TILE}; border:1px solid {C_ACCENT}; }}
        """

    def _go_fullscreen(self):
        s = QtWidgets.QApplication.primaryScreen().geometry()
        self.setGeometry(s)
        self.showFullScreen()

    def _on_front_frame(self, img):
        # On kamera frame'i - sadece ON aktifse goster
        if not self.rear_cam_active:
            self.camview.show_frame(img)

    def _on_rear_frame(self, img):
        # Arka kamera frame'i - sadece ARKA aktifse goster
        if self.rear_cam_active:
            self.camview.show_frame(img)

    def _toggle_camera(self):
        self.rear_cam_active = self.btn_cam.isChecked()
        if self.rear_cam_active:
            self.btn_cam.setText("📷 ARKA (KOL)")
            self.btn_cam.setStyleSheet(
                f"#cambtn{{background:{C_ORANGE}; color:#000; font-weight:700;"
                f" border:2px solid {C_ORANGE}; border-radius:7px; font-size:13px;}}")
        else:
            self.btn_cam.setText("📷 ÖN")
            self.btn_cam.setStyleSheet(
                f"#cambtn{{background:{C_TILE}; color:{C_ACCENT}; font-weight:700;"
                f" border:2px solid {C_ACCENT}; border-radius:7px; font-size:13px;}}")

    def on_data(self, obj):
        for key, tile in self.tiles.items():
            tile.update_value(obj.get(key))
        # joystick + butonlar
        jx = obj.get("joyX"); jy = obj.get("joyY"); btns = obj.get("btns")

        # ---- UZAKTAN KONTROL DEVREYE GIRISI ----
        # Kural: FIZIKSEL joystick merkezdeyse VE taze uzak paket varsa,
        #        surus komutu uzak joystick'ten alinir. Fiziksel kol
        #        merkezden ayrildigi anda uzak komut yok sayilir.
        REMOTE_DEAD = 80
        rjx, rjy = self.remote.get()
        fiziksel_merkezde = (
            jx is not None and jy is not None
            and abs(jx - 512) < REMOTE_DEAD
            and abs(jy - 512) < REMOTE_DEAD
        )
        if rjx is not None and fiziksel_merkezde:
            jx, jy = rjx, rjy
            if not self._remote_active:
                self._remote_active = True
                print("[UZAK] uzaktan kontrol devrede")
        else:
            if self._remote_active:
                self._remote_active = False
                print("[UZAK] uzaktan kontrol birakildi (fiziksel joystick veya zaman asimi)")

        self.joy.set_joy(jx, jy)
        self.joy.set_btns(btns)
        # son joystick degerini sakla (elektrikli timer icin)
        if jx is not None: self._last_jx = jx
        if jy is not None: self._last_jy = jy
        if btns is not None: self._last_btns = btns
        # BNO055 pusula -> kamera overlay
        hd = obj.get("heading")
        cal = obj.get("imuCal", 0) or 0
        self.camview.set_heading(hd, cal)
        # GPS -> pusulanin yaninda
        self.camview.set_gps(obj.get("lat"), obj.get("lon"), obj.get("sats", 0))
        # ---- MOTOR KONTROL: joystick -> komut -> 2.ESP32 (benzinli anlik) ----
        self._drive_from_joystick(jx, jy, btns)

    # Elektrikli mod icin duzenli komut gonderimi (10 Hz - gecikmeyi azaltir)
    def _elec_tick(self):
        if not self.electric_mode:
            return
        CENTER = 512; DEAD = 110       # merkez titreme icin genis deadzone
        jx = self._last_jx; jy = self._last_jy
        dx = jx - CENTER; dy = jy - CENTER
        self._drive_electric(dx, dy, DEAD, self._last_btns or "00000")

    # Joystick + mod -> M,gaz,fren,dir,kolBaz,dirsek,parmak komutu uret ve yolla
    def _drive_from_joystick(self, jx, jy, btns):
        if jx is None or jy is None:
            return
        CENTER = 512; DEAD = 80
        dx = jx - CENTER; dy = jy - CENTER
        btns = btns or "00000"

        # ---- ELEKTRIKLI ROBOT: WiFi/UDP hareket + 4 servo broadcast ----
        if self.electric_mode:
            self._drive_electric(dx, dy, DEAD, btns)
            return

        # ---- BENZINLI ROBOT: UART M,... komutu (asagida) ----
        if self.arm_mode:
            # ---- KOL MODU (MUTLAK KONUM/AYNA): joystick nerede, kol orada ----
            # gaz/fren KAPALI, direksiyon SABIT (guvenlik)
            gaz = 0; fren = 0
            dir_step = self._last_dir       # direksiyon son konumda kalir
            # taban step: X ekseni DOGRUDAN pozisyona esler (merkez=0)
            #   joystick yarida -> kol yarida | joystick birakilinca -> merkeze doner
            if abs(dx) > DEAD:
                kolBaz = int(self._map(dx, -512, 511, -600, 600))
            else:
                kolBaz = 0                  # joystick merkez -> kol merkeze doner
            self._last_arm_base = kolBaz
            # dirsek servo: Y ekseni DOGRUDAN aciya esler (merkez=90)
            if abs(dy) > DEAD:
                self._elbow = int(self._map(dy, -512, 511, 0, 180))
            else:
                self._elbow = 90            # joystick merkez -> dirsek orta
            # parmak: C(1.hane)=tam ac, D(2.hane)=tam kapa (basili tutulunca sabit kalir)
            if len(btns) >= 1 and btns[0] == "1":
                self._grip = 180          # C -> tam acik
            elif len(btns) >= 2 and btns[1] == "1":
                self._grip = 0            # D -> tam kapali
            cmd = f"M,{gaz},{fren},{dir_step},{kolBaz},{self._elbow},{self._grip}"
        else:
            # ---- DIREKSIYON MODU: Y=gaz/fren, X=direksiyon ----
            # kol SABIT (son konumda), joystick surus yapar
            if dy > DEAD:
                gaz = int(self._map(dy, DEAD, 511, 0, 90)); fren = 0
            elif dy < -DEAD:
                gaz = 0; fren = int(self._map(-dy, DEAD, 512, 0, 90))
            else:
                gaz = 0; fren = 0
            if dx > DEAD:
                # saga: joystick tam sagda -> +15 derece (STEER_STEPS_RIGHT adim)
                dir_step = int(self._map(dx, DEAD, 511, 0, STEER_STEPS_RIGHT))
            elif dx < -DEAD:
                # sola: joystick tam solda -> -30 derece (STEER_STEPS_LEFT adim)
                dir_step = -int(self._map(-dx, DEAD, 512, 0, STEER_STEPS_LEFT))
            else:
                dir_step = 0
            self._last_dir = dir_step
            cmd = (f"M,{gaz},{fren},{dir_step},"
                   f"{self._last_arm_base},{self._elbow},{self._grip}")

        if self.reader:
            self.reader.send_cmd(cmd)

    @staticmethod
    def _map(v, in_min, in_max, out_min, out_max):
        v = max(in_min, min(in_max, v))
        return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # Elektrikli robot: joystick -> hareket + 4 servo -> UDP broadcast
    def _drive_electric(self, dx, dy, dead, btns):
        if self.arm_mode:
            # ---- KOL MODU: hareket DUR, joystick -> servolar ----
            cmd = "S"                       # robot durur (kol kullanilir)
            # 3 servo: en alt <- X, bir ust <- Y, kiskac <- butonlar
            if abs(dx) > dead:
                self._elec_servo[0] = int(self._map(dx, -512, 511, 0, 180))  # en alt
            if abs(dy) > dead:
                self._elec_servo[1] = int(self._map(dy, -512, 511, 0, 180))  # bir ust
            # kiskac (servo3, index 2): C(1.hane)=tam ac(180), D(2.hane)=tam kapa(0)
            if len(btns) >= 1 and btns[0] == "1":
                self._elec_servo[2] = 180        # kiskac ac
            elif len(btns) >= 2 and btns[1] == "1":
                self._elec_servo[2] = 0          # kiskac kapa
        else:
            # ---- DIREKSIYON MODU: joystick -> F/B/L/R, servolar sabit ----
            if dy > dead:     cmd = "F"
            elif dy < -dead:  cmd = "B"
            elif dx > dead:   cmd = "R"
            elif dx < -dead:  cmd = "L"
            else:             cmd = "S"

        self._last_elec = cmd
        # paket: "F,s1,s2,s3,s4"  (her cagride yolla, Deneyap timeout icin)
        s = self._elec_servo
        pkt = f"{cmd},{s[0]},{s[1]},{s[2]}"   # 3 servo
        if self.udp:
            try:
                self.udp.sendto(pkt.encode("ascii"),
                                ("255.255.255.255", self.UDP_PORT))
            except Exception:
                pass

    def on_status(self, msg, ok):
        self.status.setText(msg)
        self.dot.setStyleSheet(f"#dot{{color:{C_GREEN if ok else C_RED}; font-size:12px;}}")

    def on_cam_status(self, msg, ok):
        col = C_GREEN if ok else C_RED
        self.cam_status.setText(msg)
        self.cam_status.setStyleSheet(f"#status{{color:{col}; font-size:11px;}}")

    def keyPressEvent(self, e):
        if e.key() in (QtCore.Qt.Key_Escape, QtCore.Qt.Key_Q):
            self.close()

    def _quit_app(self):
        # Temiz kapat: kameralar + seri port dur, sonra cik
        try: self.reader.stop()
        except Exception: pass
        try: self.cam.stop()
        except Exception: pass
        try:
            if hasattr(self, "cam_rear"): self.cam_rear.stop()
        except Exception: pass
        QtWidgets.QApplication.quit()

    def closeEvent(self, e):
        self.reader.stop()
        self.cam.stop()
        if hasattr(self, "cam_rear"): self.cam_rear.stop()
        e.accept()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    app = QtWidgets.QApplication(sys.argv)
    dock = GasDock(port); dock.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
