#!/usr/bin/env python3
"""
Nodo de seguimiento de línea por detección de bordes (Canny).
A diferencia de nodo_linea, que detecta la cinta por su color en el
espacio HSV, este nodo detecta los bordes de contraste entre la cinta y
el suelo mediante el algoritmo de Canny. Esto lo hace independiente del
color real o percibido de la cinta, resolviendo la limitación de
variabilidad cromática del sensor OV5647 documentada en la Prueba 6.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np

from matias_control.vision_common import CamaraRpicam

# ── Streaming opcional ────────────────────────────────────────────────────────
# Para eliminar el streaming del proyecto basta con borrar vision_stream.py
# y las líneas marcadas con [STREAM]. El nodo funciona igual sin ellas.
try:
    from matias_control.vision_stream import StreamServer  # [STREAM]
    _STREAM_AVAILABLE = True                               # [STREAM]
except ImportError:                                        # [STREAM]
    _STREAM_AVAILABLE = False                              # [STREAM]
# ─────────────────────────────────────────────────────────────────────────────


class NodoCanny(Node):
    def __init__(self):
        super().__init__("nodo_canny")

        # Umbrales de histéresis de Canny.
        # canny_high marca bordes seguros; canny_low descarta directamente.
        # Los puntos intermedios solo se conservan si están conectados a
        # un borde seguro (histéresis). Regla práctica: high ~= 2-3 x low.
        self.declare_parameter("canny_low",  50)
        self.declare_parameter("canny_high", 150)

        # Tamaño del kernel de blur gaussiano previo a Canny (debe ser impar).
        # Difumina el ruido de textura del suelo conservando el contraste
        # fuerte y sostenido del borde real cinta-suelo.
        self.declare_parameter("blur_kernel", 5)

        # Tamaño del kernel de dilatación tras Canny.
        # Los bordes de Canny tienen 1 px de grosor; dilatar los engorda
        # ligeramente para que cv2.moments calcule un centroide más estable
        # y para conectar fragmentos de borde interrumpidos por el blur.
        self.declare_parameter("dilate_kernel", 3)

        # Parámetros de control (mismos nombres que nodo_linea para
        # comparabilidad directa entre ambos algoritmos)
        self.declare_parameter("kp",     0.9)
        self.declare_parameter("v_max",  0.3)

        # roi_frac: fracción inferior de la imagen que se analiza.
        self.declare_parameter("roi_frac",      0.50)
        # area_min: a diferencia de nodo_linea (área rellena), aquí mide
        # píxeles de borde (líneas finas), por lo que su valor típico es
        # considerablemente menor.
        self.declare_parameter("area_min",      80)
        self.declare_parameter("frames_gracia", 10)
        self.declare_parameter("debug", False)

        # ── Streaming [STREAM] ────────────────────────────────────────────────
        self.declare_parameter("stream",      False) # [STREAM]
        self.declare_parameter("stream_port", 8080)  # [STREAM]

        self.stream = None                                          # [STREAM]
        if self.get_parameter("stream").value and _STREAM_AVAILABLE: # [STREAM]
            port = int(self.get_parameter("stream_port").value)     # [STREAM]
            self.stream = StreamServer(port=port)                   # [STREAM]
            self.stream.start()                                     # [STREAM]
            self.get_logger().info(f"Stream en http://<IP>:{port}/") # [STREAM]
        # ─────────────────────────────────────────────────────────────────────

        self.cam = CamaraRpicam(width=320, height=240, fps=30)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.frames_sin_linea = 0
        self.ultimo_giro = 0.0
        self.contador_debug = 0

        self.timer = self.create_timer(1.0 / 30.0, self.ciclo)
        self.get_logger().info("Nodo de seguimiento por bordes (Canny) iniciado")

    def ciclo(self):
        frame = self.cam.leer_frame()
        if frame is None:
            self.get_logger().warn("Frame no disponible; deteniendo")
            self.publicar(0.0, 0.0)
            return

        h, w = frame.shape[:2]
        # Recortamos la parte inferior de la imagen (ROI), igual que en
        # nodo_linea: ignorar el fondo reduce el ruido y el coste computacional
        roi_frac = self.get_parameter("roi_frac").value
        y0  = int(h * (1.0 - roi_frac))
        roi = frame[y0:, :]

        # Paso 1: escala de grises. Canny trabaja sobre intensidad, no color.
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Paso 2: blur gaussiano. Elimina el ruido de textura del suelo
        # (alta frecuencia) conservando el borde real cinta-suelo (baja
        # frecuencia, transición sostenida). El kernel debe ser impar.
        k = int(self.get_parameter("blur_kernel").value)
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(gray, (k, k), 0)

        # Paso 3: Canny. Devuelve una máscara binaria de 1 px de grosor
        # donde los píxeles blancos son los bordes detectados.
        canny_low  = int(self.get_parameter("canny_low").value)
        canny_high = int(self.get_parameter("canny_high").value)
        edges = cv2.Canny(blurred, canny_low, canny_high)

        # Paso 4: dilatación. Engorda los bordes finos para estabilizar el
        # cálculo del centroide y conectar fragmentos interrumpidos.
        dk = int(self.get_parameter("dilate_kernel").value)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dk, dk))
        mask = cv2.dilate(edges, kernel)

        area     = cv2.countNonZero(mask)
        area_min = self.get_parameter("area_min").value

        # Variables de telemetría inicializadas para el stream [STREAM]
        cx    = None  # [STREAM]
        error = 0.0   # [STREAM]
        giro  = 0.0   # [STREAM]
        v     = 0.0   # [STREAM]

        if area > area_min:
            # binaryImage=True: sin él, moments pondera por el valor del
            # píxel (0 ó 255) y el área calculada queda distorsionada
            M  = cv2.moments(mask, binaryImage=True)
            cx = M["m10"] / M["m00"]

            # Error normalizado en [-1, 1]: negativo = bordes a la izquierda
            error = (cx - w / 2.0) / (w / 2.0)
            kp    = self.get_parameter("kp").value
            v_max = self.get_parameter("v_max").value

            # Control proporcional, idéntico al de nodo_linea
            giro = max(-1.0, min(1.0, -kp * error))
            v    = v_max * (1.0 - 0.6 * abs(error))

            self.frames_sin_linea = 0
            self.ultimo_giro      = giro
            self.publicar(v, giro)

            self.get_logger().info(
                f"Bordes cx={cx:.0f} err={error:+.2f} -> v={v:.2f} w={giro:+.2f}"
            )
        else:
            self.frames_sin_linea += 1
            gracia = self.get_parameter("frames_gracia").value
            if self.frames_sin_linea <= gracia:
                self.publicar(0.15, self.ultimo_giro)
            else:
                self.publicar(0.0, 0.0)
                self.get_logger().warn("Bordes perdidos: vehiculo detenido")

        if self.get_parameter("debug").value:
            self.contador_debug += 1
            if self.contador_debug % 20 == 0:
                cv2.imwrite("/tmp/canny_frame.jpg", roi)
                cv2.imwrite("/tmp/canny_edges.jpg", edges)
                cv2.imwrite("/tmp/canny_mask.jpg",  mask)

        # ── Stream en tiempo real [STREAM] ────────────────────────────────────
        if self.stream:                                             # [STREAM]
            self.stream.update(                                     # [STREAM]
                roi, mask,                                          # [STREAM]
                cx=cx, error=error if cx is not None else None,     # [STREAM]
                v=v, w=giro,                                        # [STREAM]
            )                                                       # [STREAM]
        # ─────────────────────────────────────────────────────────────────────

    def publicar(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def destroy_node(self):
        self.publicar(0.0, 0.0)
        if self.stream:         # [STREAM]
            self.stream.stop()  # [STREAM]
        self.cam.cerrar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoCanny()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
