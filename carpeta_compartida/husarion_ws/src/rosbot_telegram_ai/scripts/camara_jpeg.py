#!/usr/bin/env python3
"""Copia ligera de la cámara para mandarla por Wi-Fi (se ejecuta EN EL ROBOT).

La imagen en bruto (640x480 RGB a 30 FPS, ~27 MB/s) no cabe por una Wi-Fi normal.
Este nodo la recoge en el propio robot y publica /camera/color/jpeg: JPEG a pocas
imágenes por segundo (~60 KB cada una). Sólo se suscribe a la cámara mientras
alguien escucha la copia, así que no gasta CPU si nadie la usa.

Uso: python3 camara_jpeg.py [fps] [calidad]
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CamaraJpeg(Node):

    def __init__(self, fps, calidad):
        super().__init__('camara_jpeg')
        self.periodo = 1.0 / fps
        self.calidad = calidad
        self.ultimo = 0.0
        self.sub = None
        self.pub = self.create_publisher(
            CompressedImage, '/camera/color/jpeg',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0, self._revisar)
        self.get_logger().info('Publicando /camera/color/jpeg a %.1f FPS' % fps)

    def _revisar(self):
        alguien = self.pub.get_subscription_count() > 0
        if alguien and self.sub is None:
            self.sub = self.create_subscription(Image, '/camera/color/image_raw',
                                                self._imagen, qos_profile_sensor_data)
            self.get_logger().info('Alguien escucha: empiezo a comprimir')
        elif not alguien and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info('Nadie escucha: dejo de comprimir')

    def _imagen(self, msg):
        ahora = time.monotonic()
        if ahora - self.ultimo < self.periodo:
            return
        self.ultimo = ahora
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn('Codificación no soportada: %s' % msg.encoding, once=True)
            return
        img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.calidad])
        if not ok:
            return
        salida = CompressedImage()
        salida.header = msg.header
        salida.format = 'jpeg'
        salida.data = buf.tobytes()
        self.pub.publish(salida)


def main():
    fps = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    calidad = int(sys.argv[2]) if len(sys.argv) > 2 else 70
    rclpy.init()
    nodo = CamaraJpeg(fps, calidad)
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
