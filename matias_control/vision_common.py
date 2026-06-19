#!/usr/bin/env python3
"""
Módulo de captura de imagen compartido entre los nodos de visión.
Encapsula la lectura de frames desde rpicam-vid mediante un pipe,
solución adoptada tras descubrir que picamera2 requiere entorno gráfico
(dependencia de pykms) y no funciona en modo headless.

Codec: MJPEG (en lugar de YUV420).
MJPEG incluye marcadores de inicio (0xFF 0xD8) y fin (0xFF 0xD9) en cada
frame, lo que permite localizar los límites del frame en el stream y
recuperar la sincronización en caso de desalineación. YUV420 no tiene
marcadores, por lo que una lectura desalineada corrompe todos los frames
siguientes — problema que aparece al usar resoluciones no estándar como
320x240 que el sensor OV5647 sirve con stride diferente al ancho.

Hilo lector en background: leer_frame() devuelve siempre el frame más
reciente de forma instantánea (no bloqueante), desacoplando la captura
del procesamiento y evitando que el timer de ROS 2 quede bloqueado
esperando al pipe.
"""
import subprocess
import threading

import cv2
import numpy as np


class CamaraRpicam:
    """Lector de frames desde rpicam-vid en formato MJPEG."""

    def __init__(self, width=320, height=240, fps=30):
        # Resolución reducida a 320x240 (era 640x480 con YUV420): 4x menos
        # píxeles por frame. Con MJPEG no hay problemas de stride a esta
        # resolución porque los límites del frame se detectan por marcadores.
        self.proc = subprocess.Popen(
            ["rpicam-vid", "-t", "0", "--nopreview",
             "--width",     str(width),
             "--height",    str(height),
             "--framerate", str(fps),
             "--codec",     "mjpeg", "--inline", "-o", "-"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        self._buf    = b""
        self._ultimo: np.ndarray | None = None
        self._lock   = threading.Lock()

        # Hilo de fondo: consume el pipe MJPEG continuamente y guarda
        # solo el frame más reciente para que leer_frame() sea no bloqueante.
        self._hilo = threading.Thread(
            target=self._lector, daemon=True, name="camara-lector"
        )
        self._hilo.start()

    def _lector(self):
        """Hilo de fondo: extrae frames MJPEG del pipe usando sus marcadores."""
        while True:
            chunk = self.proc.stdout.read(4096)
            if not chunk:
                break
            self._buf += chunk

            # Buscar marcador de fin de frame (EOI: 0xFF 0xD9)
            end = self._buf.rfind(b"\xff\xd9")
            if end == -1:
                continue

            # Buscar el marcador de inicio más cercano antes del fin (SOI: 0xFF 0xD8)
            start = self._buf.rfind(b"\xff\xd8", 0, end)
            if start == -1:
                continue

            jpg          = self._buf[start:end + 2]
            self._buf    = self._buf[end + 2:]

            frame = cv2.imdecode(
                np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is not None:
                # El módulo de cámara está invertido en el chasis,
                # se corrige por software para que la imagen no quede invertida
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                with self._lock:
                    self._ultimo = frame

    def leer_frame(self) -> np.ndarray | None:
        """Devuelve el último frame disponible en formato BGR (no bloqueante).
        Retorna None si el hilo lector aún no ha capturado el primer frame."""
        with self._lock:
            return self._ultimo

    def cerrar(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
