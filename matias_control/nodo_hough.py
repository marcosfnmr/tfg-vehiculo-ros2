#!/usr/bin/env python3
"""
Nodo de seguimiento de línea por rectas dominantes (Canny + Hough).
Extiende a nodo_canny: en lugar de calcular el centroide de TODOS los
píxeles de borde, aplica la Transformada de Hough sobre la salida de
Canny para quedarse solo con los segmentos rectos que reciben suficientes
"votos" de píxeles alineados.
"""
import math

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


class NodoHough(Node):
    def __init__(self):
        super().__init__("nodo_hough")

        # Umbrales de histéresis de Canny (mismo criterio que nodo_canny:
        # canny_high ~= 2-3 x canny_low).
        self.declare_parameter("canny_low",  50)
        self.declare_parameter("canny_high", 150)

        # Kernel de blur gaussiano previo a Canny (debe ser impar).
        self.declare_parameter("blur_kernel", 5)

        # Parámetros de la Transformada de Hough probabilística
        # (cv2.HoughLinesP). hough_threshold es el número mínimo de votos
        # (píxeles de borde alineados) para aceptar una recta candidata.
        self.declare_parameter("hough_threshold",   20)
        self.declare_parameter("hough_min_len",      15)  # px, longitud mínima del segmento
        self.declare_parameter("hough_max_gap",      10)  # px, hueco máximo entre puntos de la misma recta

        # Filtro angular: descarta segmentos casi horizontales, que en
        # esta ROI corresponden casi siempre a ruido de textura del suelo
        # o al horizonte, nunca a la cinta vista de frente.
        self.declare_parameter("angulo_min_deg", 25.0)

        # Ganancias del PID (mismos nombres que nodo_linea/nodo_canny
        # para comparabilidad directa entre los tres algoritmos)
        self.declare_parameter("kp", 0.9)   # término proporcional: reacciona al error actual
        self.declare_parameter("ki", 0.0)   # término integral: corrige sesgos sistemáticos
        self.declare_parameter("kd", 0.0)   # término derivativo: amortigua oscilaciones

        # Límite del acumulador integral (anti-windup). Sin él, si la
        # línea se pierde durante mucho tiempo el término integral
        # crecería sin límite y al recuperarla provocaría un giro brusco.
        self.declare_parameter("integral_max", 1.0)

        self.declare_parameter("v_max",  0.3)

        self.declare_parameter("roi_frac",      0.50)
        # min_votos: suma mínima de longitudes de los segmentos válidos
        # para considerar que hay una recta detectada de verdad.
        self.declare_parameter("min_votos",     40)
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

        # dt se deriva de la misma frecuencia del timer para que ki y kd
        # tengan unidades consistentes.
        self.dt = 1.0 / 30.0
        self.timer = self.create_timer(self.dt, self.ciclo)
        self.get_logger().info("Nodo de seguimiento por rectas (Hough) iniciado")

    def ciclo(self):
        frame = self.cam.leer_frame()
        if frame is None:
            self.get_logger().warn("Frame no disponible; deteniendo")
            self.publicar(0.0, 0.0)
            return

        h, w = frame.shape[:2]
        # ROI inferior, igual que en nodo_linea y nodo_canny
        roi_frac = self.get_parameter("roi_frac").value
        y0  = int(h * (1.0 - roi_frac))
        roi = frame[y0:, :]

        # Pasos 1-3: idénticos a nodo_canny (gris -> blur -> Canny)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        k = int(self.get_parameter("blur_kernel").value)
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(gray, (k, k), 0)

        canny_low  = int(self.get_parameter("canny_low").value)
        canny_high = int(self.get_parameter("canny_high").value)
        edges = cv2.Canny(blurred, canny_low, canny_high)

        # Paso 4: Hough probabilística sobre los bordes. Cada línea
        # devuelta es un segmento [x1, y1, x2, y2] que ya superó el
        # umbral de votos: suficientes píxeles de borde alineados en la
        # misma recta dentro de la tolerancia angular de 1 grado
        # (np.pi/180) y de posición de 1 px.
        threshold = int(self.get_parameter("hough_threshold").value)
        min_len   = int(self.get_parameter("hough_min_len").value)
        max_gap   = int(self.get_parameter("hough_max_gap").value)
        lineas = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold,
            minLineLength=min_len, maxLineGap=max_gap,
        )

        angulo_min = self.get_parameter("angulo_min_deg").value

        # Variables de telemetría inicializadas para el stream [STREAM]
        cx    = None  # [STREAM]
        error = 0.0   # [STREAM]
        giro  = 0.0   # [STREAM]
        v     = 0.0   # [STREAM]

        votos_validos = 0.0
        suma_cx_ponderada = 0.0

        if lineas is not None:
            for (x1, y1, x2, y2) in lineas[:, 0]:
                # Ángulo del segmento respecto a la horizontal. Una cinta
                # vista de frente es casi vertical en la imagen (cerca de
                # 90 grados); el ruido de textura tiende a ser casi
                # horizontal o aleatorio.
                angulo = math.degrees(math.atan2(y2 - y1, x2 - x1))
                angulo = abs(angulo) % 180
                if angulo > 90:
                    angulo = 180 - angulo

                if angulo < angulo_min:
                    continue  # descartado: demasiado horizontal, es ruido

                longitud = math.hypot(x2 - x1, y2 - y1)
                cx_segmento = (x1 + x2) / 2.0

                # Cada segmento "vota" con un peso igual a su longitud:
                # los segmentos largos son evidencia más fiable de la
                # recta real que los fragmentos cortos.
                votos_validos      += longitud
                suma_cx_ponderada  += cx_segmento * longitud

        min_votos = self.get_parameter("min_votos").value

        if votos_validos > min_votos:
            cx = suma_cx_ponderada / votos_validos

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
                f"Hough cx={cx:.0f} votos={votos_validos:.0f} "
                f"err={error:+.2f} P={termino_p:+.2f} I={termino_i:+.2f} "
                f"D={termino_d:+.2f} -> v={v:.2f} w={giro:+.2f}"
            )
        else:
            self.frames_sin_linea += 1
            gracia = self.get_parameter("frames_gracia").value
            if self.frames_sin_linea <= gracia:
                self.publicar(0.15, self.ultimo_giro)
            else:
                self.publicar(0.0, 0.0)
                self.get_logger().warn("Rectas perdidas: vehiculo detenido")
                # Reiniciamos el estado del PID al perder la línea: si no,
                # el integral seguiría acumulando error nulo (sin efecto)
                # pero la derivada al recuperar la línea compararía contra
                # un error_anterior obsoleto, provocando un giro brusco.
                self.integral       = 0.0
                self.error_anterior = 0.0

        if self.get_parameter("debug").value:
            self.contador_debug += 1
            if self.contador_debug % 20 == 0:
                debug_img = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                if lineas is not None:
                    for (x1, y1, x2, y2) in lineas[:, 0]:
                        cv2.line(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.imwrite("/tmp/hough_frame.jpg", roi)
                cv2.imwrite("/tmp/hough_edges.jpg", edges)
                cv2.imwrite("/tmp/hough_lineas.jpg", debug_img)

        # ── Stream en tiempo real [STREAM] ────────────────────────────────────
        if self.stream:                                             # [STREAM]
            self.stream.update(                                     # [STREAM]
                roi, edges,                                         # [STREAM]
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
    nodo = NodoHough()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()