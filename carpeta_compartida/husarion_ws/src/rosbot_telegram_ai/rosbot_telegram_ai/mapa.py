"""Dibuja el mapa de SLAM (nav_msgs/OccupancyGrid) como imagen para Claude y Telegram.

Blanco = libre, negro = obstáculo, gris = desconocido. Cuadrícula cada metro con las
coordenadas del marco `map`, el robot en rojo (con una raya hacia donde mira) y el
punto de salida en verde.
"""

import math

import cv2
import numpy as np


def dibujar_mapa(grid, robot=None, inicio=None, objetivo=None, lado_max=900):
    """Devuelve los bytes JPEG del mapa. robot/inicio/objetivo: (x, y[, yaw]) en metros."""
    ancho, alto = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    if ancho == 0 or alto == 0:
        return None

    datos = np.array(grid.data, dtype=np.int16).reshape(alto, ancho)
    gris = np.full((alto, ancho), 160, np.uint8)
    gris[datos == 0] = 255
    gris[(datos > 0) & (datos < 50)] = 210
    gris[datos >= 50] = 0
    gris = cv2.flip(gris, 0)  # en la imagen la y crece hacia abajo

    escala = min(lado_max / float(max(ancho, alto)), 8.0)
    img = cv2.resize(gris, (int(ancho * escala), int(alto * escala)),
                     interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def px(x, y):
        return (int((x - ox) / res * escala), int((alto - (y - oy) / res) * escala))

    # Cuadrícula de 1 m (2 m si el mapa es grande) con etiquetas.
    paso = 1 if ancho * res <= 25 and alto * res <= 25 else 2
    fuente = cv2.FONT_HERSHEY_SIMPLEX
    for gx in range(int(math.ceil(ox / paso)) * paso, int(ox + ancho * res) + 1, paso):
        c, _ = px(gx, oy)
        cv2.line(img, (c, 0), (c, img.shape[0]), (230, 180, 120), 1)
        cv2.putText(img, '%d' % gx, (c + 2, img.shape[0] - 4), fuente, 0.4, (200, 90, 0), 1)
    for gy in range(int(math.ceil(oy / paso)) * paso, int(oy + alto * res) + 1, paso):
        _, f = px(ox, gy)
        cv2.line(img, (0, f), (img.shape[1], f), (230, 180, 120), 1)
        cv2.putText(img, '%d' % gy, (2, f - 3), fuente, 0.4, (200, 90, 0), 1)
    cv2.putText(img, 'x (m) ->', (img.shape[1] - 70, img.shape[0] - 16), fuente, 0.4, (200, 90, 0), 1)
    cv2.putText(img, 'y (m) ^', (4, 14), fuente, 0.4, (200, 90, 0), 1)

    radio = max(4, int(0.12 / res * escala))
    if inicio is not None:
        cv2.circle(img, px(inicio[0], inicio[1]), radio, (0, 170, 0), -1)
    if objetivo is not None:
        cv2.drawMarker(img, px(objetivo[0], objetivo[1]), (255, 0, 200), cv2.MARKER_CROSS,
                       radio * 3, 2)
    if robot is not None:
        centro = px(robot[0], robot[1])
        cv2.circle(img, centro, radio, (0, 0, 230), -1)
        if len(robot) > 2:
            largo = 0.4
            punta = px(robot[0] + largo * math.cos(robot[2]), robot[1] + largo * math.sin(robot[2]))
            cv2.line(img, centro, punta, (0, 0, 230), 2)

    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    return buf.tobytes() if ok else None
