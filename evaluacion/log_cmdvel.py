#!/usr/bin/env python3
import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelLogger(Node):
    def __init__(self, output_path, duration):
        super().__init__('cmdvel_logger')
        self.output_path = output_path
        self.duration = duration
        self.start_time = time.time()
        self.rows = []
        self.periods = []
        self.last_t = None
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.get_logger().info(f'Registrando /cmd_vel durante {duration:.1f} s -> {output_path}')

    def callback(self, msg):
        now = time.time()
        elapsed = now - self.start_time
        self.rows.append((round(elapsed, 3), msg.linear.x, msg.angular.z))
        if self.last_t is not None:
            self.periods.append(now - self.last_t)
        self.last_t = now
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_s', 'linear_x', 'angular_z'])
            writer.writerows(self.rows)

        n = len(self.rows)
        print(f'\n--- Resumen ({n} mensajes recibidos en {self.duration:.1f} s) ---')
        if n > 1 and self.periods:
            mean_period = sum(self.periods) / len(self.periods)
            freq = 1.0 / mean_period if mean_period > 0 else float('inf')
            print(f'Frecuencia media:  {freq:.2f} Hz')
            print(f'Periodo mínimo:    {min(self.periods) * 1000:.1f} ms')
            print(f'Periodo máximo:    {max(self.periods) * 1000:.1f} ms')
        else:
            print('No hay suficientes mensajes para calcular frecuencia.')
        if self.rows:
            lin = [r[1] for r in self.rows]
            ang = [r[2] for r in self.rows]
            print(f'linear.x   -> min {min(lin):+.3f}  max {max(lin):+.3f}  media {sum(lin) / n:+.3f}')
            print(f'angular.z  -> min {min(ang):+.3f}  max {max(ang):+.3f}  media {sum(ang) / n:+.3f}')
        else:
            print('No se recibió ningún mensaje en /cmd_vel. Comprueba que nodo_vision está publicando.')
        print(f'Detalle guardado en: {self.output_path}\n')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--caso', required=True)
    parser.add_argument('--etiqueta', default='')
    parser.add_argument('--duracion', type=float, default=15.0)
    args = parser.parse_args()

    nombre = f'caso{args.caso}'
    if args.etiqueta:
        nombre += f'_{args.etiqueta}'
    output_path = f'{nombre}.csv'

    rclpy.init()
    node = CmdVelLogger(output_path, args.duracion)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()


if __name__ == '__main__':
    main()
