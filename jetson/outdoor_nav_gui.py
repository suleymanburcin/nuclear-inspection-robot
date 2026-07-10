#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
outdoor_nav_gui.py  —  DIS MEKAN SEYRUSEFER ARAYUZU
====================================================================
outdoor_nav.py'yi DEGISTIRMEZ, ondan import eder. Tek kaynak, kopya yok.
Iki kip bir arada:

  ROTA PLANLAYICI : serpantin waypoint'leri ciz, listeyi disa aktar
  SAHA IZLEME     : seri porttan JSON oku, arac konumunu/burnunu ciz,
                    direksiyon + gaz + durum rozetini canli goster

CALISTIRMA:
    python3 outdoor_nav_gui.py                 # sadece rota planlayici
    python3 outdoor_nav_gui.py /dev/ttyTHS1    # izleme kipi (seri port)

GEREKSINIM: PyQt5   (izleme kipi icin ayrica pyserial)

GUVENLIK:
  * DUR butonu, pure pursuit'i durdurur ve steer=0 / gaz=0 yapar.
  * Arayuz eyleyicilere KOMUT GONDERMEZ. Yalnizca hesaplanan degerleri
    gosterir (kuru test). Eyleyiciye baglama isi ayrica yapilmalidir.

!!! SAHA TESTINDEN ONCE OLCULECEK IKI DEGER (outdoor_nav.py icinde) !!!
    WHEELBASE   : on-arka dingil mesafesi (m). Yanlissa direksiyon acisi
                  SISTEMATIK olarak hatali cikar.
    DECLINATION : manyetik sapma (derece). Yanlissa arac sabit bir aci
                  kadar yanlis yone gider.
  Arayuz bu hatalari GOSTERMEZ; sadece cizer. Once bu ikisini olcun.
"""

import sys
import json
import math

from PyQt5 import QtCore, QtGui, QtWidgets

# --- tek kaynak: outdoor_nav.py -------------------------------------------
from outdoor_nav import (
    ANCHOR_LAT, ANCHOR_LON, FIELD_W, FIELD_H, LANE_M,
    WHEELBASE, MAX_STEER, LOOKAHEAD, WP_RADIUS,
    CRUISE, SLOW, SPD_FUSE, DECLINATION, GEOFENCE,
    WAYPOINTS, local_offset, fused_heading, in_geofence, PurePursuit,
)

# ---------------------------------------------------------------- renkler
C_BG    = QtGui.QColor(12, 14, 16)
C_GRID  = QtGui.QColor(34, 40, 44)
C_FIELD = QtGui.QColor(60, 72, 78)
C_FENCE = QtGui.QColor(196, 60, 60)
C_PATH  = QtGui.QColor(70, 92, 104)
C_WP    = QtGui.QColor(120, 140, 150)
C_WP_OK = QtGui.QColor(0, 200, 120)
C_WP_TG = QtGui.QColor(255, 200, 0)
C_CAR   = QtGui.QColor(0, 230, 200)
C_TXT   = QtGui.QColor(226, 234, 236)
C_MUTE  = QtGui.QColor(120, 132, 138)

DURUM_RENK = {
    "RUN":            QtGui.QColor(0, 200, 120),
    "DONE":           QtGui.QColor(90, 160, 255),
    "NO_FIX":         QtGui.QColor(255, 170, 40),
    "NO_HEADING":     QtGui.QColor(255, 170, 40),
    "GEOFENCE_STOP":  QtGui.QColor(240, 70, 70),
    "STOPPED":        QtGui.QColor(240, 70, 70),
    "BEKLIYOR":       QtGui.QColor(120, 132, 138),
}


# =========================================================== seri okuyucu
class SerialReader(QtCore.QThread):
    """Seri porttan JSON satirlari okur. GUI'yi bloklamaz."""
    nav = QtCore.pyqtSignal(dict)
    durum = QtCore.pyqtSignal(str, bool)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._calisiyor = True

    def run(self):
        try:
            import serial
        except ImportError:
            self.durum.emit("pyserial kurulu degil (pip install pyserial)", False)
            return
        try:
            ser = serial.Serial(self.port, 115200, timeout=2)
        except Exception as e:
            self.durum.emit(f"PORT ACILAMADI · {e}", False)
            return
        self.durum.emit(f"LIVE · {self.port}", True)
        bos = 0
        while self._calisiyor:
            try:
                raw = ser.readline().decode("utf-8", "ignore").strip()
            except Exception as e:
                self.durum.emit(f"OKUMA HATASI · {e}", False)
                break
            if not raw:
                bos += 1
                if bos > 3:
                    self.durum.emit("VERI YOK (ESP32 bagli mi?)", False)
                continue
            bos = 0
            # 1. ESP32'nin JSON'u; kalibrasyon/debug satirlarini atla
            if not raw.startswith("{") or '"lat"' not in raw:
                continue
            try:
                self.nav.emit(json.loads(raw))
            except json.JSONDecodeError:
                continue
        try:
            ser.close()
        except Exception:
            pass

    def stop(self):
        self._calisiyor = False


# =========================================================== harita cizimi
class HaritaView(QtWidgets.QWidget):
    """Ustten gorunum: alan, cograf cit, waypoint'ler, arac."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(560, 460)
        self.wp_en = [local_offset(la, lo) for la, lo in WAYPOINTS]  # (E, N)
        self.aktif = 0            # o an hedeflenen waypoint
        self.arac = None          # (E, N)
        self.heading = None       # derece, gercek kuzey
        self.iz = []              # gecmis konumlar

    def guncelle(self, arac_en, heading, aktif):
        self.arac = arac_en
        self.heading = heading
        self.aktif = aktif
        if arac_en is not None:
            self.iz.append(arac_en)
            if len(self.iz) > 4000:
                self.iz = self.iz[-4000:]
        self.update()

    # --- dunya (m) -> ekran (px) ---
    def _olcek(self):
        pay = GEOFENCE + 3.0
        gw, gh = FIELD_W + 2 * pay, FIELD_H + 2 * pay
        k = min(self.width() / gw, self.height() / gh)
        ox = (self.width() - gw * k) / 2 + pay * k
        oy = (self.height() - gh * k) / 2 + pay * k
        return k, ox, oy

    def _p(self, e, n):
        k, ox, oy = self._olcek()
        # +N yukari olsun diye y ters
        return QtCore.QPointF(ox + e * k, self.height() - (oy + n * k))

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), C_BG)
        k, _, _ = self._olcek()

        # 5 m izgara
        p.setPen(QtGui.QPen(C_GRID, 1))
        m = -GEOFENCE
        while m <= FIELD_W + GEOFENCE:
            p.drawLine(self._p(m, -GEOFENCE), self._p(m, FIELD_H + GEOFENCE)); m += 5
        m = -GEOFENCE
        while m <= FIELD_H + GEOFENCE:
            p.drawLine(self._p(-GEOFENCE, m), self._p(FIELD_W + GEOFENCE, m)); m += 5

        # cografi cit
        p.setPen(QtGui.QPen(C_FENCE, 1.4, QtCore.Qt.DashLine))
        p.drawRect(QtCore.QRectF(self._p(-GEOFENCE, FIELD_H + GEOFENCE),
                                 self._p(FIELD_W + GEOFENCE, -GEOFENCE)))
        # tarama alani
        p.setPen(QtGui.QPen(C_FIELD, 1.6))
        p.drawRect(QtCore.QRectF(self._p(0, FIELD_H), self._p(FIELD_W, 0)))

        # rota
        p.setPen(QtGui.QPen(C_PATH, 1.4))
        for i in range(len(self.wp_en) - 1):
            p.drawLine(self._p(*self.wp_en[i]), self._p(*self.wp_en[i + 1]))

        # waypoint'ler
        for i, (e, n) in enumerate(self.wp_en):
            if i < self.aktif:
                c, r = C_WP_OK, 3.5
            elif i == self.aktif:
                c, r = C_WP_TG, 6.0
            else:
                c, r = C_WP, 3.0
            p.setBrush(c); p.setPen(QtCore.Qt.NoPen)
            p.drawEllipse(self._p(e, n), r, r)
            if i == self.aktif:      # kabul yaricapi
                p.setBrush(QtCore.Qt.NoBrush)
                p.setPen(QtGui.QPen(C_WP_TG, 1, QtCore.Qt.DotLine))
                p.drawEllipse(self._p(e, n), WP_RADIUS * k, WP_RADIUS * k)

        # iz
        if len(self.iz) > 1:
            p.setPen(QtGui.QPen(QtGui.QColor(0, 120, 110), 1.2))
            for i in range(len(self.iz) - 1):
                p.drawLine(self._p(*self.iz[i]), self._p(*self.iz[i + 1]))

        # arac
        if self.arac is not None:
            pt = self._p(*self.arac)
            p.setBrush(C_CAR); p.setPen(QtCore.Qt.NoPen)
            p.drawEllipse(pt, 6, 6)
            if self.heading is not None:
                # heading: 0 = kuzey, saat yonu +
                a = math.radians(self.heading)
                ln = 26
                uc = QtCore.QPointF(pt.x() + ln * math.sin(a), pt.y() - ln * math.cos(a))
                p.setPen(QtGui.QPen(C_CAR, 2.2))
                p.drawLine(pt, uc)

        # olcek
        p.setPen(QtGui.QPen(C_MUTE, 1))
        p.setFont(QtGui.QFont("DejaVu Sans", 8))
        p.drawText(10, self.height() - 10, f"alan {FIELD_W:.0f}×{FIELD_H:.0f} m · "
                                           f"şerit {LANE_M:.0f} m · {len(WAYPOINTS)} WP · çit {GEOFENCE:.0f} m")


# =========================================================== ana pencere
class Pencere(QtWidgets.QWidget):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Dış Mekân Seyrüsefer — planlayıcı ve saha izleme")
        self.resize(1000, 620)
        self.setStyleSheet(f"background:{C_BG.name()}; color:{C_TXT.name()};")

        self.pp = PurePursuit(list(WAYPOINTS))
        self.durdu = False
        self.harita = HaritaView()

        self.lbl = {}
        sag = QtWidgets.QVBoxLayout()

        self.rozet = QtWidgets.QLabel("BEKLIYOR")
        self.rozet.setAlignment(QtCore.Qt.AlignCenter)
        self.rozet.setFixedHeight(46)
        self._rozet_renk("BEKLIYOR")
        sag.addWidget(self.rozet)

        for ad in ["Durum", "Uydu", "IMU kalib.", "Yön (°)", "Hedef WP",
                   "Mesafe (m)", "Hedef yön (°)", "Direksiyon (°)", "Gaz (0–1)",
                   "Enlem", "Boylam"]:
            sat = QtWidgets.QHBoxLayout()
            a = QtWidgets.QLabel(ad); a.setStyleSheet(f"color:{C_MUTE.name()};")
            b = QtWidgets.QLabel("—"); b.setAlignment(QtCore.Qt.AlignRight)
            b.setStyleSheet("font-family:monospace; font-size:14px;")
            sat.addWidget(a); sat.addWidget(b)
            sag.addLayout(sat)
            self.lbl[ad] = b

        sag.addSpacing(10)
        self.btn_dur = QtWidgets.QPushButton("DUR")
        self.btn_dur.setFixedHeight(44)
        self.btn_dur.setStyleSheet("background:#b03030; color:white; font-weight:bold; font-size:16px;")
        self.btn_dur.clicked.connect(self._dur)
        sag.addWidget(self.btn_dur)

        self.btn_wp = QtWidgets.QPushButton("Waypoint listesini dışa aktar")
        self.btn_wp.clicked.connect(self._disa_aktar)
        sag.addWidget(self.btn_wp)

        self.uyari = QtWidgets.QLabel(
            f"Doğrulanmamış:  WHEELBASE={WHEELBASE:.2f} m · DECLINATION={DECLINATION:.1f}°\n"
            "Saha testinden önce ölçün. Arayüz bu hataları göstermez.")
        self.uyari.setWordWrap(True)
        self.uyari.setStyleSheet("color:#e0a35f; font-size:11px; border:1px solid #e0a35f; padding:6px;")
        sag.addWidget(self.uyari)

        self.durum = QtWidgets.QLabel("kuru test · eyleyiciye komut gönderilmiyor")
        self.durum.setStyleSheet(f"color:{C_MUTE.name()}; font-size:11px;")
        sag.addWidget(self.durum)
        sag.addStretch()

        kok = QtWidgets.QHBoxLayout(self)
        kok.addWidget(self.harita, 3)
        sarici = QtWidgets.QWidget(); sarici.setLayout(sag); sarici.setFixedWidth(300)
        kok.addWidget(sarici, 1)

        self.reader = None
        if port:
            self.reader = SerialReader(port)
            self.reader.nav.connect(self.on_nav)
            self.reader.durum.connect(self.on_durum)
            self.reader.start()
        else:
            self.durum.setText("rota planlayıcı kipi · seri port verilmedi")

    def _rozet_renk(self, st):
        c = DURUM_RENK.get(st, C_MUTE)
        self.rozet.setText(st)
        self.rozet.setStyleSheet(
            f"background:{c.name()}; color:#0c0e10; font-weight:bold; font-size:17px;")

    def _dur(self):
        self.durdu = True
        self._rozet_renk("STOPPED")
        self.lbl["Direksiyon (°)"].setText("0.0")
        self.lbl["Gaz (0–1)"].setText("0.00")
        self.durum.setText("DUR basıldı · seyrüsefer durduruldu")

    def _disa_aktar(self):
        yol, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Waypoint listesi", "waypoints.csv",
                                                       "CSV (*.csv)")
        if not yol:
            return
        with open(yol, "w", encoding="utf-8") as f:
            f.write("no,lat,lon,E_m,N_m\n")
            for k, (la, lo) in enumerate(WAYPOINTS, 1):
                e, n = local_offset(la, lo)
                f.write(f"{k},{la:.7f},{lo:.7f},{e:.2f},{n:.2f}\n")
        self.durum.setText(f"{len(WAYPOINTS)} waypoint yazıldı → {yol}")

    def on_durum(self, msg, ok):
        self.durum.setText(msg)
        self.durum.setStyleSheet(f"color:{'#00c878' if ok else '#f04646'}; font-size:11px;")

    def on_nav(self, nav):
        lat, lon = nav.get("lat"), nav.get("lon")
        self.lbl["Uydu"].setText(str(nav.get("sats", "—")))
        self.lbl["IMU kalib."].setText(f'{nav.get("imuCal", 0)}/3')
        self.lbl["Enlem"].setText(f"{lat:.7f}" if lat else "—")
        self.lbl["Boylam"].setText(f"{lon:.7f}" if lon else "—")

        hd = fused_heading(nav)
        self.lbl["Yön (°)"].setText(f"{hd:.1f}" if hd is not None else "—")

        if self.durdu:
            return

        steer, throttle, info = self.pp.step(nav)
        st = info.get("state", "?")
        self._rozet_renk(st)
        self.lbl["Durum"].setText(st)
        self.lbl["Direksiyon (°)"].setText(f"{steer:+.1f}")
        self.lbl["Gaz (0–1)"].setText(f"{throttle:.2f}")
        self.lbl["Hedef WP"].setText(f'{info.get("wp", "—")} / {len(WAYPOINTS)}')
        self.lbl["Mesafe (m)"].setText(str(info.get("dist", "—")))
        self.lbl["Hedef yön (°)"].setText(str(info.get("brg", "—")))

        if lat is not None and lon is not None:
            self.harita.guncelle(local_offset(lat, lon), hd, self.pp.i)
            if not in_geofence(lat, lon):
                self.durum.setText("COĞRAFİ ÇİT İHLALİ — araç alan dışında")

    def closeEvent(self, e):
        if self.reader:
            self.reader.stop()
            self.reader.wait(1500)
        e.accept()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    app = QtWidgets.QApplication(sys.argv)
    w = Pencere(port)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
