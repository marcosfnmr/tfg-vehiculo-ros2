#!/usr/bin/env python3
"""
Nodo de seguimiento de línea con controlador PID completo.
Mismo pipeline de visión que nodo_linea (segmentación HSV de la cinta),
pero sustituye el control proporcional puro por un PID: el término
integral corrige errores sistemáticos persistentes (una rueda
desalineada) y el término derivativo amortigua las oscilaciones que el
control solo-P produce cuando kp es alta.
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
        self.declare_parameter("h_low",  0)
        self.declare_parameter("s_low",  0)
        self.declare_parameter("v_low",  0)
        self.declare_parameter("h_high", 180)
        self.declare_parameter("s_high", 255)
        self.declare_parameter("v_high", 80)

        # Rango HSV secundario opcional 
        self.declare_parameter("h2_low",  0)
        self.declare_parameter("s2_low",  0)
        self.declare_parameter("v2_low",  0)
        self.declare_parameter("h2_high", 0)
        self.declare_parameter("s2_high", 0)
        self.declare_parameter("v2_high", 0)

        # Ganancias del PID
        self.declare_parameter("kp", 0.9)   # término proporcional: reacciona al error actual
        self.declare_parameter("ki", 0.0)   # término integral: corrige sesgos sistemáticos
        self.declare_parameter("kd", 0.0)   # término derivativo: amortigua oscilaciones

        # Límite del acumulador integral (anti-windup). Sin él, si la
        # línea se pierde durante mucho tiempo el término integral
        # crecería sin límite y al recuperarla provocaría un giro brusco.
        self.declare_parameter("integral_max", 1.0)

        self.declare_parameter("v_max",  0.3)

        self.declare_parameter("roi_frac",     0.50)
        self.declare_parameter("area_min",     300)
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

        # Estado del PID, persistente entre ciclos
        self.integral       = 0.0
        self.error_anterior = 0.0

        # Timer a 20 Hz, igual que nodo_linea. dt se deriva de la misma
        # frecuencia para que ki y kd tengan unidades consistentes.
        self.frecuencia_hz = 20.0
        self.dt = 1.0 / self.frecuencia_hz
        self.timer = self.create_timer(self.dt, self.ciclo)
        self.get_logger().info("Nodo de seguimiento con PID iniciado")

    def ciclo(self):
        frame = self.cam.leer_frame()
        if frame is None:
            self.get_logger().warn("Frame no disponible; deteniendo")
            self.publicar(0.0, 0.0)
            return

        h, w = frame.shape[:2]
        roi_frac = self.get_parameter("roi_frac").value
        y0  = int(h * (1.0 - roi_frac))
        roi = frame[y0:, :]

        # Segmentación HSV, idéntica a nodo_linea
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
            M  = cv2.moments(mask, binaryImage=True)
            cx = M["m10"] / M["m00"]

            # Error normalizado en [-1, 1]
            error = (cx - w / 2.0) / (w / 2.0)

            kp = self.get_parameter("kp").value
            ki = self.get_parameter("ki").value
            kd = self.get_parameter("kd").value
            integral_max = self.get_parameter("integral_max").value
            v_max = self.get_parameter("v_max").value

            # Término proporcional: reacciona al error de este frame
            termino_p = kp * error

            # Término integral: acumula el error en el tiempo. Se limita
            # con un clamp simétrico (anti-windup) para que no se dispare
            # si la línea está perdida o el error es persistente.
            self.integral += error * self.dt
            self.integral = max(-integral_max, min(integral_max, self.integral))
            termino_i = ki * self.integral

            # Término derivativo: reacciona a la velocidad de cambio del
            # error. Si el error está creciendo rápido, frena la
            # corrección antes de que se dispare, reduciendo las
            # oscilaciones típicas del control solo-P.
            derivada = (error - self.error_anterior) / self.dt
            termino_d = kd * derivada
            self.error_anterior = error

            salida = termino_p + termino_i + termino_d
            giro   = max(-1.0, min(1.0, -salida))
            v      = v_max * (1.0 - 0.6 * abs(error))

            self.frames_sin_linea = 0
            self.ultimo_giro      = giro
            self.publicar(v, giro)

            self.get_logger().info(
                f"PID cx={cx:.0f} err={error:+.2f} "
                f"P={termino_p:+.2f} I={termino_i:+.2f} D={termino_d:+.2f} "
                f"-> v={v:.2f} w={giro:+.2f}"
            )
        else:
            self.frames_sin_linea += 1
            gracia = self.get_parameter("frames_gracia").value
            if self.frames_sin_linea <= gracia:
                self.publicar(0.15, self.ultimo_giro)
            else:
                self.publicar(0.0, 0.0)
                self.get_logger().warn("Linea perdida: vehiculo detenido")
                # Reiniciamos el estado del PID al perder la línea: si no,
                # el integral seguiría acumulando error nulo (sin efecto)
                # pero la derivada al recuperar la línea compararía contra
                # un error_anterior obsoleto, provocando un giro brusco.
                self.integral       = 0.0
                self.error_anterior = 0.0

        if self.get_parameter("debug").value:
            self.contador_debug += 1
            if self.contador_debug % 20 == 0:
                cv2.imwrite("/tmp/pid_frame.jpg", roi)
                cv2.imwrite("/tmp/pid_mask.jpg",  mask)

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