#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
remote_control.py  —  PC UZAKTAN KONTROL (REMOTE mode)
=======================================================
PC'deki joystick VEYA klavye ok tuslarindan okur, Leonardo joystick'i ile
AYNI formatta (joyX,joyY = 0..1023, merkez 512) komut uretir ve UDP ile
Jetson'a yollar. Jetson bunu ESP32'ye iletir; ESP32 REMOTE moddaysa dinler.

FORMAT (Leonardo ile birebir ayni):
    "joyX,joyY\n"   ornek: "512,512" (bosta/dur), "800,300" (ileri-sol)

KULLANIM:
    1) Asagidaki JETSON_IP'yi kendi Jetson'unun IP'si yap (Jetson'da: hostname -I)
    2) python remote_control.py
    3) Gamepad tak (Xbox tarzi) VEYA ok tuslari / WASD kullan
    4) ESC = cikis (cikarken guvenli "512,512 = dur" yollar)

GUVENLIK:
    - Tus/stick birakilinca -> 512,512 (merkez = dur)
    - Pencere odagi giderse  -> 512,512
    - Cikista               -> 512,512 birkac kez
"""

import sys
import socket
import pygame

# ======================= AYARLAR (BUNU DUZENLE) =======================
JETSON_IP   = "192.168.1.42"     # <-- Jetson'da 'hostname -I' ile ogren, buraya yaz
JETSON_PORT = 9000               # UDP port (Jetson dinleyici ile ayni olmali)
SEND_HZ     = 20                 # saniyede kac komut (20 = her 50ms)

ADC_MIN, ADC_MAX, ADC_MID = 0, 1023, 512   # Leonardo joystick araligi
DEADZONE    = 0.12               # analog stick olu bolge (merkez titremesini keser)
KEY_STEP    = 300                # klavye ile merkezden sapma (0-511 arasi)
# =====================================================================

W, H = 520, 620
C_BG    = (10, 12, 14)
C_ACC   = (0, 230, 200)
C_GREEN = (0, 230, 120)
C_RED   = (240, 70, 70)
C_MUTE  = (110, 120, 125)
C_TEXT  = (225, 235, 235)
C_TRACK = (26, 30, 33)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RemoteControl:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("SEARCHER — REMOTE CONTROL (joyX,joyY)")
        self.clock = pygame.time.Clock()
        self.font  = pygame.font.SysFont("Consolas", 18)
        self.font_s = pygame.font.SysFont("Consolas", 14)
        self.font_b = pygame.font.SysFont("Consolas", 26, bold=True)

        # UDP soket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (JETSON_IP, JETSON_PORT)

        # joystick (varsa)
        self.js = None
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            self.js_name = self.js.get_name()
            self.js_axes = self.js.get_numaxes()
            print(f"Joystick: {self.js_name}  ({self.js_axes} eksen, "
                  f"{self.js.get_numbuttons()} buton)")
        else:
            self.js_name = "YOK (klavye modu)"
            self.js_axes = 0

        self.joyX = ADC_MID
        self.joyY = ADC_MID
        self.src  = "IDLE"        # komut kaynagi: JOYSTICK / KEYBOARD / IDLE
        self.sent = 0
        self.focused = True

    # analog eksen (-1..+1) -> ADC (0..1023), deadzone uygulanmis
    def axis_to_adc(self, a):
        if abs(a) < DEADZONE:
            return ADC_MID
        # deadzone sonrasi yeniden olcekle (yumusak gecis)
        sign = 1 if a > 0 else -1
        mag = (abs(a) - DEADZONE) / (1 - DEADZONE)
        val = ADC_MID + sign * mag * (ADC_MAX - ADC_MID)
        return int(clamp(val, ADC_MIN, ADC_MAX))

    def read_inputs(self):
        keys = pygame.key.get_pressed()
        jx, jy = ADC_MID, ADC_MID
        src = "IDLE"

        # --- 1) JOYSTICK (oncelik) ---
        # Logitech Extreme 3D Pro eksen haritasi:
        #   axis 0 = X (stick sag/sol)     -> joyX (steering)
        #   axis 1 = Y (stick on/arka)     -> joyY (throttle)
        #   axis 2 = Z (twist/yaw)         -> kullanilmiyor (istege bagli)
        #   axis 3 = throttle kolu (slider)-> kullanilmiyor (istege bagli hiz limiti)
        if self.js is not None:
            ax = self.js.get_axis(0)          # stick X (sol-sag)
            ay = self.js.get_axis(1)          # stick Y (on-arka; ileri itince negatif)
            jx = self.axis_to_adc(ax)
            jy = self.axis_to_adc(-ay)        # ileri (stick one) -> joyY artsin
            if jx != ADC_MID or jy != ADC_MID:
                src = "JOYSTICK"

        # --- 2) KLAVYE (joystick bostaysa) ---
        if src == "IDLE":
            dx = dy = 0
            if keys[pygame.K_LEFT]  or keys[pygame.K_a]: dx -= KEY_STEP
            if keys[pygame.K_RIGHT] or keys[pygame.K_d]: dx += KEY_STEP
            if keys[pygame.K_UP]    or keys[pygame.K_w]: dy += KEY_STEP
            if keys[pygame.K_DOWN]  or keys[pygame.K_s]: dy -= KEY_STEP
            if dx or dy:
                jx = int(clamp(ADC_MID + dx, ADC_MIN, ADC_MAX))
                jy = int(clamp(ADC_MID + dy, ADC_MIN, ADC_MAX))
                src = "KEYBOARD"

        # --- 3) odak yoksa guvenli dur ---
        if not self.focused:
            jx, jy, src = ADC_MID, ADC_MID, "NO-FOCUS"

        self.joyX, self.joyY, self.src = jx, jy, src

    def send(self):
        msg = f"{self.joyX},{self.joyY}\n".encode()
        try:
            self.sock.sendto(msg, self.addr)
            self.sent += 1
        except Exception:
            pass

    def send_stop(self, times=3):
        for _ in range(times):
            try:
                self.sock.sendto(b"512,512\n", self.addr)
            except Exception:
                pass

    # ---------- cizim ----------
    def draw(self):
        s = self.screen
        s.fill(C_BG)

        # baslik
        s.blit(self.font_b.render("REMOTE CONTROL", True, C_ACC), (20, 16))
        s.blit(self.font_s.render(f"-> {JETSON_IP}:{JETSON_PORT}  (UDP)", True, C_MUTE), (22, 50))

        # kaynak durumu
        col = {"JOYSTICK": C_GREEN, "KEYBOARD": C_ACC,
               "IDLE": C_MUTE, "NO-FOCUS": C_RED}.get(self.src, C_MUTE)
        s.blit(self.font.render(f"KAYNAK: {self.src}", True, col), (22, 78))
        s.blit(self.font_s.render(f"Joystick: {self.js_name}", True, C_MUTE), (22, 104))

        # radar scope (Leonardo panelindeki gibi)
        cx, cy, rad = W // 2, 300, 150
        pygame.draw.circle(s, C_TRACK, (cx, cy), rad)
        pygame.draw.circle(s, (40, 46, 50), (cx, cy), rad, 2)
        pygame.draw.circle(s, (30, 35, 39), (cx, cy), int(rad * 0.66), 1)
        pygame.draw.circle(s, (30, 35, 39), (cx, cy), int(rad * 0.33), 1)
        pygame.draw.line(s, (35, 40, 44), (cx - rad, cy), (cx + rad, cy), 1)
        pygame.draw.line(s, (35, 40, 44), (cx, cy - rad), (cx, cy + rad), 1)

        # nokta konumu (joyX/joyY -> ekran)
        nx = (self.joyX - ADC_MID) / ADC_MID
        ny = (self.joyY - ADC_MID) / ADC_MID
        px = cx + int(nx * rad * 0.9)
        py = cy - int(ny * rad * 0.9)          # joyY artinca yukari (ileri)
        live = self.src in ("JOYSTICK", "KEYBOARD")
        dot_col = C_ACC if live else C_GREEN
        if live:
            pygame.draw.line(s, dot_col, (cx, cy), (px, py), 2)
        pygame.draw.circle(s, dot_col, (px, py), 14)
        pygame.draw.circle(s, C_BG, (px, py), 6)

        # sayisal degerler
        s.blit(self.font_b.render(f"X:{self.joyX:4d}  Y:{self.joyY:4d}", True, C_TEXT),
               (cx - 110, cy + rad + 20))

        # alt bilgi
        s.blit(self.font_s.render(f"paket: {self.sent}   |   ESC = cikis (guvenli dur)",
                                  True, C_MUTE), (22, H - 30))
        pygame.display.flip()

    def run(self):
        interval = 1000 // SEND_HZ
        acc = 0
        running = True
        while running:
            dt = self.clock.tick(60)
            acc += dt
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                    running = False
                elif e.type == pygame.ACTIVEEVENT:
                    # pencere odagi degisimi (guvenlik icin izle)
                    if hasattr(e, "gain"):
                        self.focused = bool(e.gain)
                elif e.type == pygame.JOYDEVICEADDED and self.js is None:
                    self.js = pygame.joystick.Joystick(0); self.js.init()
                    self.js_name = self.js.get_name()

            self.read_inputs()
            if acc >= interval:            # sabit hizda gonder
                self.send()
                acc = 0
            self.draw()

        # cikista guvenli dur
        self.send_stop()
        pygame.quit()


if __name__ == "__main__":
    print("REMOTE CONTROL baslatiliyor...")
    print(f"Hedef: {JETSON_IP}:{JETSON_PORT} (UDP)")
    print("Jetson IP'sini kod icindeki JETSON_IP satirindan ayarlamayi unutma!")
    RemoteControl().run()
