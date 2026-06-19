#!/usr/bin/env python3
"""
Nodo ROS 2 que convierte mensajes Twist de /cmd_vel en señales PWM
para el servo de dirección y el ESC, a través del controlador PCA9685.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import board
import busio
from adafruit_pca9685 import PCA9685


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CmdVelToPCA9685(Node):
    def __init__(self):
        super().__init__('cmdvel_to_pca9685')

        # Canales del PCA9685
        self.declare_parameter('pca_frequency_hz',  50)
        self.declare_parameter('servo_channel',      0)
        self.declare_parameter('esc_channel',        1)

        # Valores PWM calibrados experimentalmente para este ESC (µs).
        # esc_us_min_fwd/rev es el umbral mínimo que supera el punto muerto del ESC
        self.declare_parameter('esc_us_neutral',  1600)
        self.declare_parameter('esc_us_fwd',      1800)
        self.declare_parameter('esc_us_rev',      1400)
        self.declare_parameter('esc_us_min_fwd',  1680)
        self.declare_parameter('esc_us_min_rev',  1520)

        # Valores PWM calibrados para el servo de dirección (µs)
        self.declare_parameter('servo_us_center', 1500)
        self.declare_parameter('servo_us_left',   1800)
        self.declare_parameter('servo_us_right',  1200)

        self.declare_parameter('max_speed_mps',    1.0)
        self.declare_parameter('max_yaw_rate_rps', 1.5)
        self.declare_parameter('cmd_timeout_s',    0.3)
        self.declare_parameter('update_rate_hz',  50.0)

        # Modo pulso alterna señal de avance y neutro para
        # reducir la velocidad media en ESCs sin respuesta lineal.
        self.declare_parameter('pulse_mode',         True)

        # pulse_duty = fracción del periodo en avance (0.0-1.0)
        self.declare_parameter('pulse_duty',         0.35)

        # pulse_period_ticks = duración total del ciclo en ticks
        self.declare_parameter('pulse_period_ticks', 20)

        self.pca_freq        = int(self.get_parameter('pca_frequency_hz').value)
        self.servo_ch        = int(self.get_parameter('servo_channel').value)
        self.esc_ch          = int(self.get_parameter('esc_channel').value)
        self.esc_us_neutral  = int(self.get_parameter('esc_us_neutral').value)
        self.esc_us_fwd      = int(self.get_parameter('esc_us_fwd').value)
        self.esc_us_rev      = int(self.get_parameter('esc_us_rev').value)
        self.esc_us_min_fwd  = int(self.get_parameter('esc_us_min_fwd').value)
        self.esc_us_min_rev  = int(self.get_parameter('esc_us_min_rev').value)
        self.servo_us_center = int(self.get_parameter('servo_us_center').value)
        self.servo_us_left   = int(self.get_parameter('servo_us_left').value)
        self.servo_us_right  = int(self.get_parameter('servo_us_right').value)
        self.max_speed       = float(self.get_parameter('max_speed_mps').value)
        self.max_yaw         = float(self.get_parameter('max_yaw_rate_rps').value)
        self.cmd_timeout     = float(self.get_parameter('cmd_timeout_s').value)
        self.update_rate     = float(self.get_parameter('update_rate_hz').value)
        self.pulse_mode      = bool(self.get_parameter('pulse_mode').value)
        self.pulse_duty      = float(self.get_parameter('pulse_duty').value)
        self.pulse_period    = int(self.get_parameter('pulse_period_ticks').value)

        self.pulse_tick = 0

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = self.pca_freq
        self.servo = self.pca.channels[self.servo_ch]
        self.esc   = self.pca.channels[self.esc_ch]

        self.last_cmd_time = self.get_clock().now()
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        # Arranque seguro: neutro + centro antes de aceptar comandos
        self._write_esc_us(self.esc_us_neutral)
        self._write_servo_us(self.servo_us_center)
        time.sleep(0.2)

        self.sub   = self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        period     = 1.0 / max(1.0, self.update_rate)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Nodo listo — pulse_mode={self.pulse_mode} '
            f'duty={self.pulse_duty} period={self.pulse_period} ticks'
        )

    def _us_to_duty16(self, us):
        """Convierte microsegundos a valor de duty cycle de 16 bits (0-65535)."""
        period_us = 1_000_000.0 / float(self.pca_freq)
        return int(clamp(us / period_us, 0.0, 1.0) * 65535.0)

    def _write_esc_us(self, us):
        self.esc.duty_cycle = self._us_to_duty16(us)

    def _write_servo_us(self, us):
        self.servo.duty_cycle = self._us_to_duty16(us)

    def on_cmd(self, msg: Twist):
        v = clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        w = clamp(float(msg.angular.z), -self.max_yaw,   self.max_yaw)
        self.get_logger().info(f'RECIBIDO cmd_vel -> v={v:.3f}, w={w:.3f}')
        self.v_cmd = v
        self.w_cmd = w
        self.last_cmd_time = self.get_clock().now()

    def _map_speed_to_esc_us(self, v):
        """
        Mapea velocidad lineal a µs del ESC.
        El rango útil empieza en esc_us_min_fwd para
        garantizar que la señal siempre supere el punto muerto del ESC.
        """
        if self.max_speed <= 1e-6:
            return self.esc_us_neutral
        if v > 0.0:
            t  = v / self.max_speed
            us = self.esc_us_min_fwd + t * (self.esc_us_fwd - self.esc_us_min_fwd)
        elif v < 0.0:
            t  = (-v) / self.max_speed
            us = self.esc_us_min_rev - t * (self.esc_us_min_rev - self.esc_us_rev)
        else:
            return self.esc_us_neutral
        lo = min(self.esc_us_rev, self.esc_us_neutral, self.esc_us_fwd)
        hi = max(self.esc_us_rev, self.esc_us_neutral, self.esc_us_fwd)
        return int(clamp(us, lo, hi))

    def _map_yaw_to_servo_us(self, w):
        """Mapea velocidad angular a µs del servo. angular.z > 0 = izquierda."""
        if abs(self.max_yaw) <= 1e-6:
            return self.servo_us_center
        t = clamp(w / self.max_yaw, -1.0, 1.0)
        if t >= 0.0:
            us = self.servo_us_center + t * (self.servo_us_left  - self.servo_us_center)
        else:
            us = self.servo_us_center + t * (self.servo_us_center - self.servo_us_right)
        lo = min(self.servo_us_left, self.servo_us_center, self.servo_us_right)
        hi = max(self.servo_us_left, self.servo_us_center, self.servo_us_right)
        return int(clamp(us, lo, hi))

    def on_timer(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        # Si no llega ningún comando en cmd_timeout segundos, paramos el vehículo
        if age > self.cmd_timeout:
            self.get_logger().info('TIMEOUT -> STOP')
            self._write_esc_us(self.esc_us_neutral)
            self._write_servo_us(self.servo_us_center)
            self.pulse_tick = 0
            return
        
        # El servo se actualiza siempre, sin pulso
        servo_us = self._map_yaw_to_servo_us(self.w_cmd)
        self._write_servo_us(servo_us)

        if self.pulse_mode and self.v_cmd != 0.0:
            # Avanzamos el contador del ciclo y enviamos señal solo en la
            # fracción 'on' del periodo, neutro el resto
            self.pulse_tick = (self.pulse_tick + 1) % self.pulse_period
            on_ticks = int(self.pulse_duty * self.pulse_period)
            if self.pulse_tick < on_ticks:
                esc_us = self._map_speed_to_esc_us(self.v_cmd)
            else:
                esc_us = self.esc_us_neutral
        else:
            esc_us = self._map_speed_to_esc_us(self.v_cmd)

        self.get_logger().info(f'PWM -> ESC={esc_us} us, SERVO={servo_us} us')
        self._write_esc_us(esc_us)

    def destroy_node(self):
        # Antes de cerrar, dejar el vehículo en posición segura
        try:
            self._write_esc_us(self.esc_us_neutral)
            self._write_servo_us(self.servo_us_center)
            time.sleep(0.2)
            self.pca.deinit()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToPCA9685()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
