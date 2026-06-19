#!/usr/bin/env python3
"""
Nodo de seguimiento de línea por color.
Detecta una cinta coloreada en el suelo mediante umbralizado HSV
y publica consignas de dirección y velocidad en /cmd_vel.
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


class NodoLinea(Node):
    def __init__(self):
        super().__init__("nodo_linea")

        # Rango HSV del color a seguir.
        # Para la cinta negra: H y S se dejan abiertos al máximo porque en
        # píxeles muy oscuros esos canales son ruidosos e irrelevantes.
        # Solo se filtra por V (brillo bajo).
        self.declare_parameter("h_low",  0)
        self.declare_parameter("s_low",  0)
        self.declare_parameter("v_low",  0)
        self.declare_parameter("h_high", 180)
        self.declare_parameter("s_high", 255)
        self.declare_parameter("v_high", 80)

        # Rango HSV secundario (opcional, solo activo si v2_high > 0).
        # Permite detectar cintas que la cámara OV5647 renderiza en dos
        # colores distintos según el ángulo o la distancia (p.ej. azul
        # en un tramo y magenta en otro).
        self.declare_parameter("h2_low",  0)
        self.declare_parameter("s2_low",  0)
        self.declare_parameter("v2_low",  0)
        self.declare_parameter("h2_high", 0)
        self.declare_parameter("s2_high", 0)
        self.declare_parameter("v2_high", 0)

        # Parámetros de control
        self.declare_parameter("kp",     0.9) # ganancia proporcional
        self.declare_parameter("v_max",  0.3) # velocidad de crucero

        # roi_frac: fracción inferior de la imagen que se analiza.
        # Solo nos interesa el suelo delante del vehículo, no el fondo.
        self.declare_parameter("roi_frac",     0.50)
        self.declare_parameter("area_min",     300) # píxeles mínimos para considerar detección válida
        self.declare_parameter("frames_gracia", 10) # ciclos antes de declarar línea perdida
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

        # Resolución reducida a 320x240 y fps subidos a 30 para mayor
        # rendimiento. vision_common.py gestiona la captura en background.
        self.cam = CamaraRpicam(width=320, height=240, fps=30)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.frames_sin_linea = 0
        self.ultimo_giro = 0.0
        self.contador_debug = 0

        # Timer a 20 Hz
        self.timer = self.create_timer(1.0 / 20.0, self.ciclo)
        self.get_logger().info("Nodo de seguimiento de linea iniciado")

    def ciclo(self):
        frame = self.cam.leer_frame()
        if frame is None:
            self.get_logger().warn("Frame no disponible; deteniendo")
            self.publicar(0.0, 0.0)
            return

        h, w = frame.shape[:2]
        # Recortamos la parte inferior de la imagen (ROI)
        # ignorar el fondo reduce el ruido y el coste computacional
        roi_frac = self.get_parameter("roi_frac").value
        y0  = int(h * (1.0 - roi_frac))
        roi = frame[y0:, :]

        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        low  = np.array([
            self.get_parameter("h_low").value,
            self.get_parameter("s_low").value,
            self.get_parameter("v_low").value,
        ])
        high = np.array([
            self.get_parameter("h_high").value,
            self.get_parameter("s_high").value,
            self.get_parameter("v_high").value,
        ])
        mask = cv2.inRange(hsv, low, high)

        # Máscara secundaria: se combina con OR si v2_high > 0
        v2_high = self.get_parameter("v2_high").value
        if v2_high > 0:
            low2  = np.array([
                self.get_parameter("h2_low").value,
                self.get_parameter("s2_low").value,
                self.get_parameter("v2_low").value,
            ])
            high2 = np.array([
                self.get_parameter("h2_high").value,
                self.get_parameter("s2_high").value,
                v2_high,
            ])
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low2, high2))

        # Apertura morfológica: elimina manchas pequeñas de ruido
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        area     = cv2.countNonZero(mask)
        area_min = self.get_parameter("area_min").value

        # Variables de telemetría inicializadas para el stream [STREAM]
        cx    = None  # [STREAM]
        error = 0.0   # [STREAM]
        giro  = 0.0   # [STREAM]
        v     = 0.0   # [STREAM]

        if area > area_min:
            # binaryImage=True es importante: sin él, moments pondera por
            # el valor del píxel (0 ó 255) y m00 resulta 255 veces el área real
            M  = cv2.moments(mask, binaryImage=True)
            cx = M["m10"] / M["m00"]

            # Error normalizado en [-1, 1]: negativo = línea a la izquierda
            error = (cx - w / 2.0) / (w / 2.0)
            kp    = self.get_parameter("kp").value
            v_max = self.get_parameter("v_max").value

            # Control proporcional, si la línea está a la derecha (error > 0)
            # giramos a la derecha (angular.z negativo)
            giro = max(-1.0, min(1.0, -kp * error))

            # Reducimos velocidad en curvas para no salirnos de la línea
            v    = v_max * (1.0 - 0.6 * abs(error))

            self.frames_sin_linea = 0
            self.ultimo_giro      = giro
            self.publicar(v, giro)

            self.get_logger().info(
                f"Linea cx={cx:.0f} err={error:+.2f} -> v={v:.2f} w={giro:+.2f}"
            )
        else:
            self.frames_sin_linea += 1
            gracia = self.get_parameter("frames_gracia").value
            if self.frames_sin_linea <= gracia:
                # Durante el periodo de gracia mantenemos el último giro
                # a velocidad reducida para superar pequeñas interrupciones
                self.publicar(0.15, self.ultimo_giro)
            else:
                self.publicar(0.0, 0.0)
                self.get_logger().warn("Linea perdida: vehiculo detenido")

        if self.get_parameter("debug").value:
            self.contador_debug += 1
            if self.contador_debug % 20 == 0:
                cv2.imwrite("/tmp/linea_frame.jpg", roi)
                cv2.imwrite("/tmp/linea_mask.jpg",  mask)

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
    nodo = NodoLinea()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
