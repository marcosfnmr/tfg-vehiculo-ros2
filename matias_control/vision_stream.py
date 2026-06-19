#!/usr/bin/env python3
"""
Servidor MJPEG ligero para depuración visual del pipeline de visión.

Sirve un stream de frames compuestos (imagen original + máscara)
accesible desde cualquier navegador en:
    http://<IP_RASPBERRY>:<puerto>/

Para ELIMINAR este módulo del proyecto basta con:
  1. Borrar este archivo.
  2. Quitar la línea 'from matias_control.vision_stream import StreamServer'
     del nodo que lo importe.
  3. Los parámetros 'stream' y 'stream_port' quedarán sin efecto.

Sin dependencias externas — usa únicamente la biblioteca estándar de
Python, OpenCV y NumPy, que ya son requisitos del proyecto.
"""

import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np


# ── HTML de la página de depuración ──────────────────────────────────────────

_HTML = ("""<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Vision Debug — nodo_linea</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #111;
      color: #ccc;
      font-family: monospace;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 12px;
      gap: 8px;
    }
    h1 { font-size: 13px; color: #888; letter-spacing: 1px; }
    img { width: 100%; max-width: 1280px; border: 1px solid #333; display: block; }
    .labels {
      display: flex;
      width: 100%;
      max-width: 1280px;
      justify-content: space-around;
      font-size: 11px;
      color: #555;
    }
  </style>
</head>
<body>
  <h1>VISION DEBUG &mdash; frame izquierda / m&aacute;scara derecha</h1>
  <img src="/stream" alt="stream mjpeg">
  <div class="labels">
    <span>ROI (imagen procesada)</span>
    <span>M&aacute;scara HSV</span>
  </div>
</body>
</html>
""").encode("utf-8")


# ── Buffer thread-safe ────────────────────────────────────────────────────────

class _FrameBuffer:
    """Almacena el último JPEG compuesto, notifica a los clientes."""

    def __init__(self):
        self._data: bytes | None = None
        self._lock = threading.Lock()
        self._event = threading.Event()

    def put(self, jpg: bytes) -> None:
        with self._lock:
            self._data = jpg
        self._event.set()

    def get(self, timeout: float = 2.0) -> bytes | None:
        self._event.wait(timeout=timeout)
        self._event.clear()
        with self._lock:
            return self._data


# ── Handler HTTP ──────────────────────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass  # silenciar logs del servidor en consola

    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/stream':
            self._serve_mjpeg()
        else:
            self.send_error(404)

    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(_HTML)))
        self.end_headers()
        self.wfile.write(_HTML)

    def _serve_mjpeg(self):
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header(
            'Content-Type',
            'multipart/x-mixed-replace; boundary=frame'
        )
        self.end_headers()
        try:
            while True:
                jpg = self.server.buffer.get(timeout=2.0)
                if jpg is None:
                    continue
                header = (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n'
                )
                self.wfile.write(header + jpg + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass  # cliente desconectado


# ── API pública ───────────────────────────────────────────────────────────────

class StreamServer:
    """
    Servidor MJPEG para depuración visual del nodo de línea.

    Uso mínimo en un nodo ROS 2
    ----------------------------
    from matias_control.vision_stream import StreamServer

    # En __init__:
    self.stream = StreamServer(port=8080)
    self.stream.start()

    # En ciclo(), tras calcular roi y mask:
    self.stream.update(roi, mask, cx=cx, error=error)

    # En destroy_node():
    self.stream.stop()

    Parámetros
    ----------
    port     : puerto HTTP (defecto 8080)
    quality  : calidad JPEG 1-100 (defecto 70); bajar para reducir CPU
    """

    def __init__(self, port: int = 8080, quality: int = 70):
        self._port = port
        self._quality = [cv2.IMWRITE_JPEG_QUALITY, quality]
        self._buf = _FrameBuffer()
        self._server: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        """Arranca el servidor HTTP en un hilo daemon."""
        self._server = HTTPServer(('0.0.0.0', self._port), _Handler)
        self._server.buffer = self._buf
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name='vision-stream'
        )
        self._thread.start()

    def stop(self) -> None:
        """Para el servidor limpiamente."""
        if self._server:
            self._server.shutdown()

    # ── Actualización de frame ────────────────────────────────────────────────

    def update(
        self,
        frame_bgr: np.ndarray,
        mask_gray: np.ndarray,
        cx: float | None = None,
        error: float | None = None,
        v: float | None = None,
        w: float | None = None,
    ) -> None:
        """
        Compone frame + máscara y actualiza el buffer del stream.

        Parámetros opcionales de telemetría (se dibujan sobre el frame):
          cx     : posición horizontal del centroide en píxeles
          error  : error normalizado [-1, 1]
          v      : velocidad lineal publicada
          w      : velocidad angular publicada
        """
        if frame_bgr is None or mask_gray is None:
            return

        # Copia para no modificar los arrays originales
        viz = frame_bgr.copy()
        h, w_img = viz.shape[:2]

        # Línea central de referencia (verde)
        cv2.line(viz, (w_img // 2, 0), (w_img // 2, h), (0, 200, 0), 1)

        # Centroide detectado (círculo rojo + línea de error)
        if cx is not None:
            cx_i = int(cx)
            cv2.circle(viz, (cx_i, h // 2), 6, (0, 0, 255), -1)
            cv2.line(viz, (w_img // 2, h // 2), (cx_i, h // 2), (0, 0, 255), 2)

        # Telemetría en esquina superior izquierda
        lines = []
        if error is not None:
            lines.append(f'err={error:+.2f}')
        if v is not None:
            lines.append(f'v={v:.2f}')
        if w is not None:
            lines.append(f'w={w:+.2f}')
        for i, txt in enumerate(lines):
            cv2.putText(
                viz, txt, (4, 14 + i * 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1,
                cv2.LINE_AA
            )

        # Máscara → BGR para poder apilarla
        mask_bgr = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)

        # Frame compuesto: imagen a la izquierda, máscara a la derecha
        composed = np.hstack([viz, mask_bgr])

        ok, jpg = cv2.imencode('.jpg', composed, self._quality)
        if ok:
            self._buf.put(jpg.tobytes())
