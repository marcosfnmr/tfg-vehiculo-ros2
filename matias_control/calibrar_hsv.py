#!/usr/bin/env python3
"""
calibrar_hsv.py — Calibración del rango HSV de la línea (headless).
Como la Raspberry Pi opera sin pantalla, no pueden usarse trackbars de
OpenCV. Este script sigue el mismo flujo de revisión remota que ya se
empleó en el Incremento 3 (descarga vía python3 -m http.server).
Modo 1 — sugerencia automática:
    python3 calibrar_hsv.py
  Captura un frame, analiza la franja inferior central (donde debe
  estar la cinta delante del coche) y propone un rango HSV a partir de
  los percentiles 5-95 de los píxeles dominantes.
Modo 2 — verificación de un rango:
    python3 calibrar_hsv.py 20 80 80 35 255 255
  Aplica el rango indicado (h_low s_low v_low h_high s_high v_high) y
  guarda frame y máscara en /tmp para revisarlos desde el PC:
    cd /tmp && python3 -m http.server 8080
"""
import sys
import time
import numpy as np
import cv2
from matias_control.vision_common import CamaraRpicam


def capturar():
    cam = CamaraRpicam(width=320, height=240, fps=30)
    # vision_common ahora usa MJPEG con hilo lector en background.
    # leer_frame() es no bloqueante, por lo que hay que esperar a que
    # el hilo capture el primer frame antes de empezar a iterar.
    time.sleep(1.5)
    # Descartar los primeros frames (auto-exposición estabilizándose)
    frame = None
    for _ in range(15):
        time.sleep(0.05)  # Un frame entre iteraciones (~30fps)
        frame = cam.leer_frame()
    cam.cerrar()
    if frame is None:
        print("ERROR: no se pudo capturar frame")
        sys.exit(1)
    return frame


def sugerir(frame):
    h, w = frame.shape[:2]
    # Parche: franja inferior (75-95% de la altura), tercio central
    parche = frame[int(0.75 * h):int(0.95 * h), int(w / 3):int(2 * w / 3)]
    hsv = cv2.cvtColor(parche, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    p5 = np.percentile(hsv, 5, axis=0).astype(int)
    p95 = np.percentile(hsv, 95, axis=0).astype(int)
    if p95[0] - p5[0] > 90:
        print("AVISO: el rango de tono (H) sugerido es muy amplio.")
        print("Si la cinta es ROJA, su tono envuelve en H=0/180 y los")
        print("percentiles no son validos: usa cinta amarilla o azul,")
        print("o dos rangos (0-10 y 170-180) como en nodo_vision.\n")
    print("Coloca la cinta delante del coche antes de ejecutar.")
    print(f"Rango HSV sugerido (percentiles 5-95 del parche central):")
    print(f"  h_low={p5[0]}  s_low={p5[1]}  v_low={p5[2]}")
    print(f"  h_high={p95[0]} s_high={p95[1]} v_high={p95[2]}")
    print("\nVerifica con:")
    print(f"  python3 calibrar_hsv.py {p5[0]} {p5[1]} {p5[2]} "
          f"{p95[0]} {p95[1]} {p95[2]}")
    cv2.imwrite("/tmp/calib_frame.jpg", frame)
    cv2.imwrite("/tmp/calib_parche.jpg", parche)
    print("Imagenes guardadas en /tmp (calib_frame.jpg, calib_parche.jpg)")


def verificar(frame, valores):
    low = np.array(valores[:3])
    high = np.array(valores[3:])
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    cobertura = 100.0 * np.count_nonzero(mask) / mask.size
    cv2.imwrite("/tmp/calib_frame.jpg", frame)
    cv2.imwrite("/tmp/calib_mask.jpg", mask)
    print(f"Rango {valores[:3]} - {valores[3:]}")
    print(f"Cobertura de la mascara: {cobertura:.1f}% del frame")
    print("Revisa /tmp/calib_mask.jpg: la cinta debe verse blanca y "
          "limpia, el resto negro.")


if __name__ == "__main__":
    frame = capturar()
    if len(sys.argv) == 7:
        verificar(frame, [int(v) for v in sys.argv[1:]])
    else:
        sugerir(frame)
