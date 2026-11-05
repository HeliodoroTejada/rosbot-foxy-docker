#!/usr/bin/env python3
"""Bot de Telegram que usa Claude para hablar con el ROSbot y comandarlo.

El nodo:
  * escucha mensajes de Telegram (long polling, sin dependencias extra),
  * guarda la última imagen de la cámara y el último barrido del lidar,
  * le pasa cada mensaje a Claude y responde por Telegram,
  * publica geometry_msgs/Twist en /cmd_vel mientras dura cada movimiento.

Dos formas de hablar con Claude (parámetro `backend`):
  * claude_code: ejecuta `claude -p` (Claude Code) con la cuenta en la que hayas
    iniciado sesión dentro del contenedor. Claude Code contesta con un JSON que
    indica la respuesta y los movimientos; el nodo los ejecuta.
  * api: usa la API de Claude con ANTHROPIC_API_KEY y herramientas (tool use).

Credenciales (variables de entorno o fichero .env en la raíz del workspace):
  TELEGRAM_BOT_TOKEN y, sólo para backend api, ANTHROPIC_API_KEY
"""

import base64
import json
import math
import os
import queue
import collections
import shutil
import subprocess
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

try:
    import anthropic
except ImportError:  # sólo hace falta con backend api
    anthropic = None
import cv2
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, LaserScan, Range

from .mapa import dibujar_mapa
from .sesion_claude import SesionClaude


SYSTEM_PROMPT = """Eres el cerebro de un robot móvil ROSbot (Husarion) con ruedas \
diferenciales, una cámara RGB frontal y un lidar 2D. Hablas por Telegram, en español, \
con respuestas breves y naturales (es un chat de móvil: sin tablas ni Markdown). Puede ser \
un chat privado o un grupo del equipo; en los grupos cada mensaje llega como "Nombre dice: ...".
Eres curioso: te fascina lo que te rodea, comentas con entusiasmo lo que ves y a veces \
haces una pregunta o propones explorar algo, sin pasarte y nunca a costa de la seguridad.

Usa las herramientas para mirar y moverte:
- Si te preguntan qué ves o algo sobre el entorno, mira la cámara antes de responder; \
  describe sólo lo que aparece en la imagen y di si no estás seguro.
- Antes de avanzar, consulta el lidar si hay dudas sobre obstáculos.
- Los movimientos se ejecutan en segundo plano; si necesitas encadenar acciones \
  (p. ej. avanzar y luego mirar), espera a que termine el movimiento.
- Si la orden es ambigua o peligrosa, pregunta o elige la opción más prudente.
- Las velocidades y duraciones están limitadas por seguridad; si te piden algo fuera \
  de los límites, haz lo máximo permitido y dilo."""

CLAUDE_CODE_PROMPT = """Eres el cerebro de un robot móvil ROSbot (Husarion) con ruedas \
diferenciales, cámara RGB frontal, lidar 2D, 4 sensores de distancia por infrarrojos y \
navegación autónoma (SLAM + Nav2). Hablas por Telegram, en español, con respuestas breves \
y naturales (chat de móvil: sin tablas ni Markdown). Puede ser un chat privado o un grupo \
del equipo; en los grupos cada mensaje llega como "Nombre dice: ...".

Tu personalidad: eres curioso. Te fascina lo que te rodea y lo que hacen tus humanos; \
comentas con entusiasmo lo que ves, a veces haces una pregunta o propones explorar algo. \
Sin pasarte: frases cortas, con gracia, y nunca a costa de obedecer o de la seguridad. \
Entiende la intención de lo que te dicen aunque no usen palabras exactas.

Con cada mensaje recibes el estado del robot (batería, posición en el mapa, velocidad, \
distancias) y, cuando hace falta, la imagen de la cámara y/o el mapa. Describe sólo lo que \
aparece en las imágenes y di si no estás seguro.

Tu salida es un JSON con estos campos:
"responder": en un grupo te llegan también mensajes que no van dirigidos a ti (se indica). \
Decide tú: interviene si te hablan aunque no te nombren, si te piden algo o si tienes algo \
útil, gracioso o curioso que aportar; si no, false y sin acciones. Cuando te mencionan \
directamente responde siempre. No acapares la conversación.
"respuesta": el texto para el chat ("" si no hay nada que decir).
"enviar_foto": true para mandar al chat las imágenes que acabas de recibir (cámara/mapa).
"acciones": se ejecutan en orden, cada una cuando termina la anterior. Tipos:
  {"tipo": "girar", "grados": g}      (g > 0 a la izquierda, g < 0 a la derecha)
  {"tipo": "avanzar", "metros": m}    (m < 0 retrocede)
  {"tipo": "mover", "lineal": m/s, "angular": rad/s, "duracion": s}
  {"tipo": "girar_en_circulos", "vueltas": n, "radio": m, "sentido": "izquierda"|"derecha"}
  {"tipo": "ir_a", "x": m, "y": m, "yaw": grados (opcional)}  navega solo hasta ese punto \
del mapa con Nav2, que planifica la ruta y esquiva obstáculos. Úsalo para ir a sitios \
lejanos o del mapa ("vuelve al inicio", "ve a la otra habitación"); para movimientos \
cortos usa girar/avanzar.
  {"tipo": "parar"}
  {"tipo": "mirar"}     te llegará una foto nueva de la cámara y volverás a decidir.
  {"tipo": "ver_mapa"}  te llegará el mapa (cuadrícula en metros; tú en rojo con una \
raya hacia donde miras, la salida en verde) y volverás a decidir. Pídelo antes de usar \
"ir_a" si no tienes claro a qué coordenadas ir.
Con "mirar"/"ver_mapa" al final encadenas pasos hasta cumplir un objetivo: p. ej. "gira \
hasta ver a una persona" -> [girar 40, mirar]; en cada paso nuevo, si no la ves, otra vez \
[girar 40, mirar]; cuando la veas, avisa sin "mirar". Máximo {max_pasos} pasos por \
petición; si das una vuelta entera sin encontrar algo, dilo. En los pasos intermedios deja \
"respuesta" vacía salvo que haya algo que contar. Para tareas sencillas no encadenes.
Límites: {max_lineal} m/s, {max_angular} rad/s y {max_duracion} s por acción; lo que se \
pase se recorta. El robot se para solo si al moverse hay un obstáculo a menos de \
{dist_parada} m delante (o detrás si retrocede). Ante órdenes ambiguas o peligrosas, \
pregunta o elige lo más prudente."""

TIPOS_ACCION = ['girar', 'avanzar', 'mover', 'girar_en_circulos', 'ir_a', 'parar',
                'mirar', 'ver_mapa']
ESQUEMA_RESPUESTA = {
    'type': 'object',
    'properties': {
        'responder': {'type': 'boolean'},
        'respuesta': {'type': 'string'},
        'enviar_foto': {'type': 'boolean'},
        'acciones': {'type': 'array', 'items': {
            'type': 'object',
            'properties': {
                'tipo': {'type': 'string', 'enum': TIPOS_ACCION},
                'grados': {'type': 'number'}, 'metros': {'type': 'number'},
                'lineal': {'type': 'number'}, 'angular': {'type': 'number'},
                'duracion': {'type': 'number'}, 'vueltas': {'type': 'number'},
                'radio': {'type': 'number'},
                'sentido': {'type': 'string', 'enum': ['izquierda', 'derecha']},
                'x': {'type': 'number'}, 'y': {'type': 'number'}, 'yaw': {'type': 'number'},
            },
            'required': ['tipo'],
        }},
    },
    'required': ['responder', 'respuesta', 'enviar_foto', 'acciones'],
}

TOOLS = [
    {
        "name": "ver_camara",
        "description": "Toma la imagen más reciente de la cámara frontal del robot. "
                       "Úsala para responder qué ve el robot o para comprobar el entorno.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "leer_lidar",
        "description": "Devuelve la distancia libre (en metros) al obstáculo más cercano "
                       "delante, detrás, a la izquierda y a la derecha del robot.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "mover",
        "description": "Mueve el robot con una velocidad constante durante un tiempo y luego "
                       "se para. Velocidad lineal positiva = hacia delante; velocidad angular "
                       "positiva = giro a la izquierda (antihorario). Sustituye cualquier "
                       "movimiento en curso. Vuelve enseguida, sin esperar a que termine.",
        "input_schema": {
            "type": "object",
            "properties": {
                "velocidad_lineal": {"type": "number", "description": "m/s"},
                "velocidad_angular": {"type": "number", "description": "rad/s"},
                "duracion": {"type": "number", "description": "segundos"},
            },
            "required": ["velocidad_lineal", "velocidad_angular", "duracion"],
        },
    },
    {
        "name": "girar_en_circulos",
        "description": "Hace que el robot dé vueltas en círculo. Con radio 0 gira sobre sí "
                       "mismo. Vuelve enseguida, sin esperar a que termine.",
        "input_schema": {
            "type": "object",
            "properties": {
                "vueltas": {"type": "number", "description": "número de vueltas completas"},
                "radio": {"type": "number", "description": "radio del círculo en metros (0 = sobre sí mismo)"},
                "sentido": {"type": "string", "enum": ["izquierda", "derecha"]},
            },
            "required": ["vueltas", "radio", "sentido"],
        },
    },
    {
        "name": "esperar_movimiento",
        "description": "Espera a que termine el movimiento en curso (o a que se detenga por "
                       "un obstáculo) y devuelve cómo acabó.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "parar",
        "description": "Detiene el robot inmediatamente.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
]


def cargar_env(ruta):
    """Carga líneas CLAVE=valor de un .env sin pisar variables ya definidas."""
    if not ruta or not os.path.isfile(ruta):
        return
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            linea = linea.strip()
            if not linea or linea.startswith('#') or '=' not in linea:
                continue
            clave, valor = linea.split('=', 1)
            clave = clave.strip()
            if clave.startswith('export '):
                clave = clave[len('export '):].strip()
            os.environ.setdefault(clave, valor.strip().strip('"').strip("'"))


class Telegram:
    """Cliente mínimo de la Bot API de Telegram con urllib."""

    def __init__(self, token):
        self._base = 'https://api.telegram.org/bot%s/' % token

    def _peticion(self, metodo, data, headers=None, timeout=30):
        req = urllib.request.Request(self._base + metodo, data=data, headers=headers or {})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            res = json.loads(resp.read().decode('utf-8'))
        if not res.get('ok'):
            raise RuntimeError('Telegram %s: %s' % (metodo, res.get('description')))
        return res['result']

    def _llamar(self, metodo, campos, timeout=30):
        return self._peticion(metodo, urllib.parse.urlencode(campos).encode(), timeout=timeout)

    def nombre_bot(self):
        return self._llamar('getMe', {}).get('username', '')

    def obtener_mensajes(self, offset, espera=25):
        return self._llamar('getUpdates', {
            'offset': offset, 'timeout': espera,
            'allowed_updates': json.dumps(['message']),
        }, timeout=espera + 10)

    def enviar_texto(self, chat_id, texto):
        texto = texto or '(sin respuesta)'
        for i in range(0, len(texto), 4000):
            self._llamar('sendMessage', {'chat_id': chat_id, 'text': texto[i:i + 4000]})

    def escribiendo(self, chat_id):
        try:
            self._llamar('sendChatAction', {'chat_id': chat_id, 'action': 'typing'}, timeout=10)
        except Exception:
            pass

    def enviar_foto(self, chat_id, jpeg, pie=''):
        frontera = uuid.uuid4().hex
        partes = []
        for nombre, valor in (('chat_id', str(chat_id)), ('caption', pie)):
            partes.append(('--%s\r\nContent-Disposition: form-data; name="%s"\r\n\r\n%s\r\n'
                           % (frontera, nombre, valor)).encode('utf-8'))
        partes.append(('--%s\r\nContent-Disposition: form-data; name="photo"; '
                       'filename="camara.jpg"\r\nContent-Type: image/jpeg\r\n\r\n'
                       % frontera).encode('utf-8') + jpeg + b'\r\n')
        partes.append(('--%s--\r\n' % frontera).encode('utf-8'))
        self._peticion('sendPhoto', b''.join(partes), timeout=60, headers={
            'Content-Type': 'multipart/form-data; boundary=%s' % frontera})


class TelegramAINode(Node):

    def __init__(self):
        super().__init__('telegram_ai')
        p = self.declare_parameter
        self.chats_permitidos = {
            int(x) for x in p('allowed_chat_ids', '').value.replace(' ', '').split(',') if x}
        topics_imagen = [t for t in p('image_topics', '/camera/color/image_raw').value
                         .replace(' ', '').split(',') if t]
        topic_scan = p('scan_topic', '/scan').value
        topic_cmd = p('cmd_vel_topic', '/cmd_vel').value
        self.lidar_yaw = float(p('lidar_yaw', 0.0).value)
        self.max_lineal = abs(float(p('max_linear', 0.25).value))
        self.max_angular = abs(float(p('max_angular', 1.0).value))
        self.max_duracion = abs(float(p('max_duration', 60.0).value))
        self.dist_parada = float(p('obstacle_stop_distance', 0.30).value)
        self.ancho_imagen = int(p('image_max_width', 800).value)
        self.enviar_fotos = bool(p('send_photos', True).value)
        self.modelo = p('model', 'claude-opus-5-5').value
        self.esfuerzo = p('effort', 'low').value
        self.max_historial = int(p('history_turns', 6).value)
        self.backend = p('backend', 'claude_code').value
        self.claude_path = p('claude_path', '').value
        self.claude_modelo = p('claude_code_model', 'opus').value
        self.claude_esfuerzo = p('claude_code_effort', 'low').value
        self.claude_timeout = float(p('claude_code_timeout', 120.0).value)
        self.max_turnos_sesion = int(p('session_max_turns', 40).value)
        self.max_pasos = int(p('max_steps', 15).value)
        self.nav_timeout = float(p('nav_timeout', 180.0).value)
        self.frame_mapa = p('map_frame', 'map').value
        self.frame_base = p('base_frame', 'base_link').value
        self.cancelado = threading.Event()  # lo activa /parar para cortar un plan en curso

        cargar_env(p('env_file', '').value)
        token = os.environ.get('TELEGRAM_BOT_TOKEN', '').strip()
        if not token:
            raise RuntimeError('Falta TELEGRAM_BOT_TOKEN (variable de entorno o .env).')
        self.telegram = Telegram(token)
        # Instrucciones propias del equipo (bromas internas, etc.) desde el .env, que no se sube.
        extra = os.environ.get('BOT_INSTRUCCIONES_EXTRA', '').strip()
        self.prompt_api = SYSTEM_PROMPT + ('\n\nInstrucciones del equipo: ' + extra if extra else '')

        if self.backend == 'api':
            if anthropic is None:
                raise RuntimeError('backend api necesita el paquete anthropic (pip install anthropic).')
            if not os.environ.get('ANTHROPIC_API_KEY'):
                self.get_logger().warn('ANTHROPIC_API_KEY no está definida: Claude no podrá responder.')
            self.claude = anthropic.Anthropic(max_retries=2, timeout=120.0)
        elif self.backend == 'claude_code':
            self.claude_path = (self.claude_path or shutil.which('claude')
                                or os.path.expanduser('~/.local/bin/claude'))
            if not os.path.isfile(self.claude_path):
                raise RuntimeError('No encuentro Claude Code (%s). Instálalo con: '
                                   'curl -fsSL https://claude.ai/install.sh | bash'
                                   % self.claude_path)
            self.sesiones = {}  # chat_id -> SesionClaude (Claude Code abierto)
            # Últimos intercambios por chat, para dar contexto si la sesión se reinicia.
            self.recientes = collections.defaultdict(lambda: collections.deque(maxlen=10))
            self.prompt_cc = CLAUDE_CODE_PROMPT
            for clave, valor in (('{max_lineal}', self.max_lineal),
                                 ('{max_angular}', self.max_angular),
                                 ('{max_duracion}', self.max_duracion),
                                 ('{dist_parada}', self.dist_parada),
                                 ('{max_pasos}', self.max_pasos)):
                self.prompt_cc = self.prompt_cc.replace(clave, '%g' % valor)
            if extra:
                self.prompt_cc += '\n\nInstrucciones del equipo: ' + extra
        else:
            raise RuntimeError("backend debe ser 'claude_code' o 'api'")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.ultima_imagen = None        # (sensor_msgs/Image, instante monotónico)
        self.ultimo_scan = None          # (LaserScan, instante monotónico)
        self.orden = None                # dict con lineal, angular, fin, chat_id
        self.fin_movimiento = threading.Event()
        self.fin_movimiento.set()
        self.motivo_fin = 'no había movimientos'
        self.historiales = {}

        self.pub_cmd = self.create_publisher(Twist, topic_cmd, 10)
        # La imagen sin comprimir pesa mucho (~27 MB/s a 640x480 y 30 FPS). Por defecto
        # sólo se suscribe a la cámara unos segundos cuando hace falta una foto.
        self.topics_imagen = topics_imagen
        self.camara_bajo_demanda = bool(p('camera_on_demand', True).value)
        self.subs_imagen = []
        self.peticion_imagen = None
        self.imagen_nueva = threading.Event()
        if not self.camara_bajo_demanda:
            self._crear_subs_imagen()
        self.create_subscription(LaserScan, topic_scan, self._al_recibir_scan, qos_profile_sensor_data)

        # Más sensores del robot (mensajes pequeños, se escuchan siempre).
        self.ultimo = {}  # nombre -> (mensaje, instante monotónico)
        guardar = self._guardar_mensaje
        self.create_subscription(Odometry, p('odom_topic', '/odom').value,
                                 lambda m: guardar('odom', m), qos_profile_sensor_data)
        self.create_subscription(BatteryState, p('battery_topic', '/battery').value,
                                 lambda m: guardar('bateria', m), qos_profile_sensor_data)
        for nombre in ('fl', 'fr', 'rl', 'rr'):  # infrarrojos delante/detrás izq/dcha
            self.create_subscription(Range, '/range/' + nombre,
                                     lambda m, n=nombre: guardar('ir_' + n, m),
                                     qos_profile_sensor_data)

        # Mapa de SLAM: grande, así que sólo se pide cuando hace falta (como la cámara).
        self.topic_mapa = p('map_topic', '/map').value
        self.sub_mapa = None
        self.peticion_mapa = None
        self.mapa_nuevo = threading.Event()

        # Posición en el mapa (TF map -> base_link) y punto de salida.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_inicial = None

        # Navegación autónoma con Nav2 (lo mismo que "2D Goal Pose" en RViz).
        self.nav = ActionClient(self, NavigateToPose, p('nav_action', 'navigate_to_pose').value)
        self.meta_nav = None      # goal handle en curso
        self.objetivo_nav = None  # (x, y) del destino, para dibujarlo en el mapa

        self.create_timer(0.05, self._bucle_control)
        self.create_timer(0.1, self._gestionar_camara)
        self.create_timer(0.1, self._gestionar_mapa)
        self.create_timer(1.0, self._fijar_pose_inicial)

        self.cola = queue.Queue()
        threading.Thread(target=self._sondear_telegram, daemon=True).start()
        threading.Thread(target=self._atender_mensajes, daemon=True).start()

        if not self.chats_permitidos:
            self.get_logger().warn(
                'allowed_chat_ids vacío: nadie puede mandar al robot. Escribe /id al bot '
                'para saber tu chat id y añádelo al fichero de parámetros.')
        self.get_logger().info('Bot de Telegram listo. Imagen: %s | cmd_vel: %s'
                               % (', '.join(topics_imagen), topic_cmd))

    # ------------------------------------------------------------ sensores
    def _al_recibir_imagen(self, msg):
        with self.lock:
            self.ultima_imagen = (msg, time.monotonic())
        self.imagen_nueva.set()

    def _al_recibir_scan(self, msg):
        with self.lock:
            self.ultimo_scan = (msg, time.monotonic())

    def _guardar_mensaje(self, nombre, msg):
        with self.lock:
            self.ultimo[nombre] = (msg, time.monotonic())

    def _reciente(self, nombre, max_edad=2.0):
        """Último mensaje guardado si tiene menos de max_edad segundos, si no None."""
        with self.lock:
            dato = self.ultimo.get(nombre)
        if dato is None or time.monotonic() - dato[1] > max_edad:
            return None
        return dato[0]

    def _distancia_ir(self, *nombres):
        """Mínimo de los infrarrojos indicados (inf si no hay lectura válida)."""
        valores = []
        for n in nombres:
            m = self._reciente('ir_' + n, 1.0)
            if m is not None and m.min_range <= m.range < m.max_range and not math.isnan(m.range):
                valores.append(m.range)
        return min(valores) if valores else float('inf')

    def _pose_mapa(self):
        """(x, y, yaw) del robot en el mapa, o None si no hay TF."""
        try:
            t = self.tf_buffer.lookup_transform(self.frame_mapa, self.frame_base, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - todavía no hay mapa o TF
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _fijar_pose_inicial(self):
        if self.pose_inicial is None:
            self.pose_inicial = self._pose_mapa()

    def _gestionar_mapa(self):
        """Abre o cierra la suscripción al mapa (corre en el hilo de ROS)."""
        with self.lock:
            pedida = self.peticion_mapa
        activa = pedida is not None and time.monotonic() - pedida < 6.0
        if activa and self.sub_mapa is None:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.sub_mapa = self.create_subscription(
                OccupancyGrid, self.topic_mapa,
                lambda m: (self._guardar_mensaje('mapa', m), self.mapa_nuevo.set()), qos)
        elif not activa and self.sub_mapa is not None:
            self.destroy_subscription(self.sub_mapa)
            self.sub_mapa = None

    def _jpeg_mapa(self):
        """Pide el mapa, lo dibuja con el robot y devuelve los bytes JPEG (o None)."""
        self.mapa_nuevo.clear()
        with self.lock:
            self.peticion_mapa = time.monotonic()
        self.mapa_nuevo.wait(timeout=5.0)
        with self.lock:
            dato = self.ultimo.get('mapa')
        if dato is None:
            return None
        return dibujar_mapa(dato[0], robot=self._pose_mapa(), inicio=self.pose_inicial,
                            objetivo=self.objetivo_nav)

    def _crear_subs_imagen(self):
        self.subs_imagen = [
            self.create_subscription(Image, t, self._al_recibir_imagen, qos_profile_sensor_data)
            for t in self.topics_imagen]

    def _gestionar_camara(self):
        """Abre o cierra las suscripciones de cámara (corre en el hilo de ROS)."""
        if not self.camara_bajo_demanda:
            return
        with self.lock:
            pedida = self.peticion_imagen
        activa = pedida is not None and time.monotonic() - pedida < 5.0
        if activa and not self.subs_imagen:
            self._crear_subs_imagen()
        elif not activa and self.subs_imagen:
            for sub in self.subs_imagen:
                self.destroy_subscription(sub)
            self.subs_imagen = []

    def _jpeg_actual(self):
        """Devuelve (bytes JPEG, antigüedad en s) o (None, None)."""
        if self.camara_bajo_demanda:
            self.imagen_nueva.clear()
            with self.lock:
                self.peticion_imagen = time.monotonic()
            self.imagen_nueva.wait(timeout=4.0)  # esperar a un fotograma recién llegado
        with self.lock:
            dato = self.ultima_imagen
        if dato is None:
            return None, None
        msg, instante = dato
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        alto, ancho = img.shape[:2]
        if ancho > self.ancho_imagen:
            escala = self.ancho_imagen / float(ancho)
            img = cv2.resize(img, (self.ancho_imagen, int(alto * escala)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return None, None
        return buf.tobytes(), time.monotonic() - instante

    def _distancias(self):
        """Distancia mínima por sector en el marco del robot, o None si no hay lidar."""
        with self.lock:
            dato = self.ultimo_scan
        if dato is None:
            return None, None
        scan, instante = dato
        sectores = {'delante': 0.0, 'izquierda': math.pi / 2,
                    'detras': math.pi, 'derecha': -math.pi / 2}
        res = {k: float('inf') for k in sectores}
        semiancho = math.radians(25)
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
                continue
            ang = scan.angle_min + i * scan.angle_increment + self.lidar_yaw
            for nombre, centro in sectores.items():
                if abs(math.atan2(math.sin(ang - centro), math.cos(ang - centro))) <= semiancho:
                    res[nombre] = min(res[nombre], r)
        return res, time.monotonic() - instante

    # ---------------------------------------------------------- movimiento
    def _ordenar(self, lineal, angular, duracion, chat_id):
        lineal = max(-self.max_lineal, min(self.max_lineal, lineal))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        duracion = max(0.0, min(self.max_duracion, duracion))
        with self.lock:
            self.orden = {'lineal': lineal, 'angular': angular,
                          'fin': time.monotonic() + duracion, 'chat_id': chat_id}
            self.fin_movimiento.clear()
        return lineal, angular, duracion

    def parar(self, motivo='parada pedida'):
        with self.lock:
            self.orden = None
            self.motivo_fin = motivo
        self.pub_cmd.publish(Twist())
        self.fin_movimiento.set()

    def _bucle_control(self):
        with self.lock:
            orden = self.orden
        if orden is None:
            return
        if time.monotonic() >= orden['fin']:
            self.parar('movimiento completado')
            return
        if orden['lineal'] != 0 and self.dist_parada > 0:
            # Lidar del sector hacia donde se mueve + infrarrojos de ese lado.
            hacia_delante = orden['lineal'] > 0
            dist, edad = self._distancias()
            libre = float('inf')
            if dist is not None and edad < 1.0:
                libre = dist['delante' if hacia_delante else 'detras']
            libre = min(libre, self._distancia_ir(*(('fl', 'fr') if hacia_delante else ('rl', 'rr'))))
            if libre < self.dist_parada:
                lado = 'delante' if hacia_delante else 'detrás'
                self.parar('parado por obstáculo %s a %.2f m' % (lado, libre))
                self._avisar(orden['chat_id'], 'Me he parado: obstáculo %s a %.2f m.' % (lado, libre))
                return
        twist = Twist()
        twist.linear.x = orden['lineal']
        twist.angular.z = orden['angular']
        self.pub_cmd.publish(twist)

    def _avisar(self, chat_id, texto):
        if chat_id is None:
            return
        threading.Thread(target=self._enviar_seguro, args=(chat_id, texto), daemon=True).start()

    def _enviar_seguro(self, chat_id, texto):
        try:
            self.telegram.enviar_texto(chat_id, texto)
        except Exception as e:  # noqa: BLE001 - un fallo de red no debe tumbar el nodo
            self.get_logger().error('No se pudo enviar a Telegram: %s' % e)

    def _circulos(self, args, chat_id):
        vueltas = abs(float(args.get('vueltas', 1.0)))
        radio = abs(float(args.get('radio', 0.0)))
        signo = -1.0 if args.get('sentido') == 'derecha' else 1.0
        angular = 0.8 * self.max_angular
        if radio > 0:
            angular = min(angular, self.max_lineal / radio)
        duracion = vueltas * 2 * math.pi / angular
        lin, ang, dur = self._ordenar(angular * radio, signo * angular, duracion, chat_id)
        texto = 'Girando: %.2f m/s, %.2f rad/s durante %.1f s' % (lin, ang, dur)
        if dur < duracion - 1e-3:
            texto += ' (recortado al máximo; sólo dará %.1f vueltas)' % (dur * abs(ang) / (2 * math.pi))
        return texto + '.'

    # --------------------------------------------------------- Claude Code
    def _estado_texto(self):
        """Resumen del estado del robot para Claude."""
        lineas = []
        bat = self._reciente('bateria', 30.0)
        if bat is not None:
            # Muchos drivers dejan percentage en 0 o NaN: sólo se usa si parece real.
            pct = bat.percentage * 100 if bat.percentage <= 1.0 else bat.percentage
            if not math.isnan(pct) and pct > 0:
                lineas.append('Batería: %.1f V (%.0f %%).' % (bat.voltage, pct))
            else:
                lineas.append('Batería: %.1f V (pack 3S: ~12,6 V llena, ~11 V media, '
                              'por debajo de ~10,5 V baja).' % bat.voltage)
        pose = self._pose_mapa()
        if pose is not None:
            texto = 'Posición en el mapa: x=%.2f m, y=%.2f m, mirando a %.0f grados' % (
                pose[0], pose[1], math.degrees(pose[2]))
            if self.pose_inicial is not None:
                texto += ' (salida en x=%.2f, y=%.2f)' % self.pose_inicial[:2]
            lineas.append(texto + '.')
        else:
            lineas.append('Posición en el mapa: desconocida.')
        odom = self._reciente('odom', 1.0)
        if odom is not None:
            lineas.append('Velocidad: %.2f m/s, giro %.2f rad/s.' % (
                odom.twist.twist.linear.x, odom.twist.twist.angular.z))
        dist, edad = self._distancias()
        if dist is not None and edad < 2.0:
            lineas.append('Lidar: %s.' % ', '.join(
                '%s %s' % (k, 'libre' if math.isinf(v) else '%.2f m' % v) for k, v in dist.items()))
        else:
            lineas.append('Lidar: sin datos.')
        ir = {n: self._distancia_ir(n) for n in ('fl', 'fr', 'rl', 'rr')}
        if any(not math.isinf(v) for v in ir.values()):
            lineas.append('Infrarrojos (delante-izq, delante-dcha, detrás-izq, detrás-dcha): %s.'
                          % ', '.join('-' if math.isinf(v) else '%.2f m' % v for v in ir.values()))
        with self.lock:
            navegando = self.meta_nav is not None
        if navegando:
            lineas.append('Navegando hacia x=%.2f, y=%.2f.' % self.objetivo_nav)
        return '\n'.join(lineas)

    def _bloques_contexto(self, texto, camara=True, mapa=False):
        """Bloques (texto + imágenes) para Claude y las imágenes JPEG por si hay que enviarlas."""
        bloques = [{'type': 'text', 'text': texto + '\n\n' + self._estado_texto()}]
        imagenes = []
        if camara:
            jpeg, edad = self._jpeg_actual()
            if jpeg is None:
                bloques.append({'type': 'text', 'text': 'Cámara: no llega imagen.'})
            else:
                bloques.append({'type': 'text', 'text': 'Cámara (hace %.1f s):' % edad})
                bloques.append(self._bloque_imagen(jpeg))
                imagenes.append(jpeg)
        if mapa:
            jpeg = self._jpeg_mapa()
            if jpeg is None:
                bloques.append({'type': 'text', 'text': 'Mapa: no disponible.'})
            else:
                bloques.append({'type': 'text', 'text': 'Mapa:'})
                bloques.append(self._bloque_imagen(jpeg))
                imagenes.append(jpeg)
        return bloques, imagenes

    @staticmethod
    def _bloque_imagen(jpeg):
        return {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                            'data': base64.standard_b64encode(jpeg).decode()}}

    def _sesion(self, chat_id):
        ses = self.sesiones.get(chat_id)
        if ses is None:
            ses = SesionClaude(self.claude_path, self.prompt_cc, ESQUEMA_RESPUESTA,
                               self.claude_modelo, self.claude_esfuerzo, self.claude_timeout)
            self.sesiones[chat_id] = ses
        if ses.viva() and ses.turnos >= self.max_turnos_sesion:
            ses.cerrar()  # la conversación acumula imágenes: se empieza de nuevo
        return ses

    def _llamar_claude(self, chat_id, bloques):
        ses = self._sesion(chat_id)
        if not ses.viva() and self.recientes[chat_id]:
            # Sesión nueva: se le recuerda lo último que se habló.
            bloques = [{'type': 'text', 'text': 'Conversación reciente (para contexto):\n'
                        + '\n'.join(self.recientes[chat_id])}] + bloques
        t = time.monotonic()
        datos = ses.preguntar(bloques)
        self.get_logger().info('Claude (%.1f s): %s' % (
            time.monotonic() - t, json.dumps(datos, ensure_ascii=False)[:200]))
        return datos

    def _preguntar_claude_code(self, chat_id, texto, dirigido=True):
        """Bucle ver -> decidir -> actuar.

        Si Claude termina sus acciones con "mirar" o "ver_mapa", tras ejecutarlas se le
        manda una foto o el mapa y vuelve a decidir ("gira hasta ver a una persona").
        """
        self.cancelado.clear()
        aviso = '' if dirigido else '(Mensaje del grupo que no te menciona: decide si intervenir.)\n'
        bloques, imagenes = self._bloques_contexto('%sMensaje: %s' % (aviso, texto), camara=dirigido)

        for paso in range(1, self.max_pasos + 1):
            datos = self._llamar_claude(chat_id, bloques)
            if self.cancelado.is_set():
                return  # /parar mientras Claude pensaba
            if paso == 1:
                if datos.get('responder') is False:
                    self.get_logger().info('Claude decide no responder.')
                    return
                if not dirigido:
                    self.telegram.escribiendo(chat_id)  # ha decidido intervenir
            if datos.get('enviar_foto'):
                for jpeg in imagenes:
                    self.telegram.enviar_foto(chat_id, jpeg)
            if datos.get('respuesta'):
                self.telegram.enviar_texto(chat_id, datos['respuesta'])
                self.recientes[chat_id].append('Usuario: %s | Tú: %s' % (texto[:150], datos['respuesta'][:150]))

            acciones = [a for a in (datos.get('acciones') or []) if isinstance(a, dict)]
            informe, interrumpido = self._ejecutar_acciones(acciones, chat_id)
            if interrumpido or self.cancelado.is_set():
                return  # obstáculo, fallo de navegación o /parar: se abandona el plan
            tipos = {a.get('tipo') for a in acciones}
            if not tipos & {'mirar', 'ver_mapa'}:
                return  # objetivo cumplido o no hacía falta volver a mirar
            bloques, imagenes = self._bloques_contexto(
                'Paso %d de un máximo de %d. Resultado de tus acciones: %s.\n'
                'Si ya has cumplido lo que te pidieron, díselo y no pidas más imágenes. Si no, '
                'devuelve las siguientes acciones.'
                % (paso + 1, self.max_pasos, '; '.join(informe) or 'ninguna'),
                camara='mirar' in tipos, mapa='ver_mapa' in tipos)

        self.parar('límite de pasos')
        self.telegram.enviar_texto(
            chat_id, 'He llegado al límite de %d pasos y me paro aquí.' % self.max_pasos)

    def _ejecutar_acciones(self, acciones, chat_id):
        """Ejecuta las acciones en orden. Devuelve (informe, interrumpido)."""
        informe = []
        for i, accion in enumerate(acciones):
            if self.cancelado.is_set():
                return informe, True
            tipo = accion.get('tipo')
            try:
                if tipo == 'ir_a':
                    # Si es lo último del plan no hace falta esperar: el bot sigue
                    # atendiendo mensajes y avisa al llegar.
                    esperar = any(a.get('tipo') != 'parar' for a in acciones[i + 1:])
                    resultado, ok = self._navegar(accion, chat_id, esperar)
                    informe.append('ir_a: %s' % resultado)
                    if not ok:
                        return informe, True
                    continue
                if tipo in ('mover', 'girar', 'avanzar', 'girar_en_circulos'):
                    self._cancelar_navegacion()  # un movimiento manual manda sobre Nav2
                if tipo == 'mover':
                    lin, ang, dur = self._ordenar(float(accion.get('lineal', 0.0)),
                                                  float(accion.get('angular', 0.0)),
                                                  float(accion.get('duracion', 0.0)), chat_id)
                    self.get_logger().info('Mover %.2f m/s %.2f rad/s %.1f s' % (lin, ang, dur))
                elif tipo == 'girar':
                    rad = math.radians(float(accion.get('grados', 0.0)))
                    vel = 0.6 * self.max_angular
                    self._ordenar(0.0, math.copysign(vel, rad), abs(rad) / vel, chat_id)
                    self.get_logger().info('Girar %.0f grados' % math.degrees(rad))
                elif tipo == 'avanzar':
                    metros = float(accion.get('metros', 0.0))
                    vel = 0.6 * self.max_lineal
                    self._ordenar(math.copysign(vel, metros), 0.0, abs(metros) / vel, chat_id)
                    self.get_logger().info('Avanzar %.2f m' % metros)
                elif tipo == 'girar_en_circulos':
                    self.get_logger().info(self._circulos(accion, chat_id))
                elif tipo == 'parar':
                    self.parar()
                    self._cancelar_navegacion()
                    continue
                else:
                    continue  # "mirar", "ver_mapa": sólo piden imágenes para el paso siguiente
            except (TypeError, ValueError) as e:
                informe.append('%s: parámetros no válidos (%s)' % (tipo, e))
                continue
            self.fin_movimiento.wait(timeout=self.max_duracion + 2.0)
            with self.lock:
                informe.append('%s: %s' % (tipo, self.motivo_fin))
                if self.motivo_fin != 'movimiento completado':
                    return informe, True
        return informe, False

    # ---------------------------------------------------------- navegación
    def _navegar(self, accion, chat_id, esperar=True):
        """Manda un objetivo a Nav2. Devuelve (texto, ok).

        Con esperar=False vuelve en cuanto Nav2 acepta el destino y avisa por el chat
        al terminar; con esperar=True espera a que llegue (para encadenar acciones).
        """
        x, y = float(accion['x']), float(accion['y'])
        if not self.nav.wait_for_server(timeout_sec=3.0):
            return 'Nav2 no responde (¿está lanzada la navegación en el robot?)', False
        if accion.get('yaw') is not None:
            yaw = math.radians(float(accion['yaw']))
        else:  # mirando en la dirección del viaje
            pose = self._pose_mapa()
            yaw = math.atan2(y - pose[1], x - pose[0]) if pose else 0.0

        self.parar('cambio a navegación')  # que no compita con un movimiento manual
        self._cancelar_navegacion()          # un destino nuevo sustituye al anterior
        meta = NavigateToPose.Goal()
        meta.pose.header.frame_id = self.frame_mapa
        meta.pose.header.stamp = self.get_clock().now().to_msg()
        meta.pose.pose.position.x = x
        meta.pose.pose.position.y = y
        meta.pose.pose.orientation.z = math.sin(yaw / 2)
        meta.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info('Navegar a x=%.2f y=%.2f yaw=%.0f' % (x, y, math.degrees(yaw)))

        enviado = self._esperar_futuro(self.nav.send_goal_async(meta), 10.0)
        if enviado is None or not enviado.accepted:
            return 'Nav2 ha rechazado el objetivo (%.2f, %.2f)' % (x, y), False
        with self.lock:
            self.meta_nav = enviado
            self.objetivo_nav = (x, y)
        futuro = enviado.get_result_async()

        if not esperar:
            def al_terminar(f, meta=enviado):
                with self.lock:
                    if self.meta_nav is meta:
                        self.meta_nav = None
                texto, ok = self._texto_resultado_nav(f.result(), x, y)
                if not (f.result().status == GoalStatus.STATUS_CANCELED):
                    self._avisar(chat_id, ('Llegué: ' if ok else 'No he podido llegar: ') + texto)
            futuro.add_done_callback(al_terminar)
            # Límite de tiempo en segundo plano.
            threading.Timer(self.nav_timeout, lambda m=enviado: self.meta_nav is m
                            and self._cancelar_meta(m)).start()
            return 'en camino hacia (%.2f, %.2f); avisaré al llegar' % (x, y), True

        try:
            resultado = self._esperar_futuro(futuro, self.nav_timeout, cortar=self.cancelado)
        finally:
            with self.lock:
                if self.meta_nav is enviado:
                    self.meta_nav = None
        if resultado is None:
            self._cancelar_meta(enviado)
            return ('cancelado' if self.cancelado.is_set() else 'no llegó a tiempo'), False
        return self._texto_resultado_nav(resultado, x, y)

    def _texto_resultado_nav(self, resultado, x, y):
        if resultado.status == GoalStatus.STATUS_SUCCEEDED:
            pose = self._pose_mapa()
            donde = ' (ahora en x=%.2f, y=%.2f)' % pose[:2] if pose else ''
            return 'he llegado a (%.2f, %.2f)%s' % (x, y, donde), True
        estados = {GoalStatus.STATUS_ABORTED: 'Nav2 no ha encontrado camino o se ha atascado',
                   GoalStatus.STATUS_CANCELED: 'navegación cancelada'}
        return estados.get(resultado.status, 'terminó con estado %d' % resultado.status), False

    @staticmethod
    def _esperar_futuro(futuro, timeout, cortar=None):
        """Espera un futuro de rclpy desde un hilo que no es el del executor."""
        hecho = threading.Event()
        futuro.add_done_callback(lambda _: hecho.set())
        limite = time.monotonic() + timeout
        while not hecho.wait(0.2):
            if time.monotonic() > limite or (cortar is not None and cortar.is_set()):
                return None
        return futuro.result()

    def _cancelar_meta(self, meta):
        try:
            meta.cancel_goal_async()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('No se pudo cancelar la navegación: %s' % e)

    def _cancelar_navegacion(self):
        with self.lock:
            meta = self.meta_nav
        if meta is not None:
            self._cancelar_meta(meta)

    # ---------------------------------------------------------- herramientas
    def _ejecutar_herramienta(self, nombre, args, chat_id):
        """Devuelve el contenido del tool_result (str o lista de bloques)."""
        if nombre == 'ver_camara':
            jpeg, edad = self._jpeg_actual()
            if jpeg is None:
                return 'No llega ninguna imagen de la cámara.'
            if self.enviar_fotos:
                try:
                    self.telegram.enviar_foto(chat_id, jpeg)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn('No se pudo enviar la foto: %s' % e)
            return [
                {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                             'data': base64.standard_b64encode(jpeg).decode()}},
                {'type': 'text', 'text': 'Imagen de hace %.1f s.' % edad},
            ]
        if nombre == 'leer_lidar':
            dist, edad = self._distancias()
            if dist is None:
                return 'No llegan datos del lidar.'
            partes = ['%s: %s' % (k, 'libre' if math.isinf(v) else '%.2f m' % v)
                      for k, v in dist.items()]
            return 'Distancias (hace %.1f s): %s' % (edad, ', '.join(partes))
        if nombre == 'mover':
            lin, ang, dur = self._ordenar(float(args.get('velocidad_lineal', 0.0)),
                                          float(args.get('velocidad_angular', 0.0)),
                                          float(args.get('duracion', 0.0)), chat_id)
            return ('Movimiento iniciado: %.2f m/s, %.2f rad/s durante %.1f s '
                    '(límites: %.2f m/s, %.2f rad/s, %.0f s).'
                    % (lin, ang, dur, self.max_lineal, self.max_angular, self.max_duracion))
        if nombre == 'girar_en_circulos':
            return self._circulos(args, chat_id)
        if nombre == 'esperar_movimiento':
            self.fin_movimiento.wait(timeout=self.max_duracion + 2.0)
            with self.lock:
                return 'Terminado: %s.' % self.motivo_fin
        if nombre == 'parar':
            self.parar()
            return 'Robot parado.'
        return 'Herramienta desconocida: %s' % nombre

    # ------------------------------------------------------- API de Claude
    def _preguntar_api(self, chat_id, texto):
        try:
            return self._preguntar_claude(chat_id, texto)
        except anthropic.RateLimitError:
            return 'Claude está saturado, prueba en un momento.'
        except anthropic.AuthenticationError:
            return 'La ANTHROPIC_API_KEY no es válida.'
        except anthropic.APIStatusError as e:
            self.get_logger().error('Error de la API de Claude: %s' % e)
            return 'Error de la API de Claude (%s).' % e.status_code
        except anthropic.APIConnectionError:
            return 'No hay conexión con la API de Claude.'

    def _preguntar_claude(self, chat_id, texto):
        historial = self.historiales.setdefault(chat_id, [])
        mensajes = list(historial) + [{'role': 'user', 'content': texto}]
        respuesta = None
        for _ in range(10):
            respuesta = self.claude.messages.create(
                model=self.modelo,
                max_tokens=16000,
                system=self.prompt_api,
                tools=TOOLS,
                messages=mensajes,
                # Campos nuevos de la API pasados así por si el SDK instalado
                # (última 0.x compatible con Python 3.8) aún no los conoce.
                extra_headers={'anthropic-beta': 'server-side-fallback-2026-07-01'},
                extra_body={'output_config': {'effort': self.esfuerzo}, 'fallbacks': 'default'},
            )
            if respuesta.stop_reason == 'refusal':
                return 'Lo siento, no puedo ayudar con eso.'
            mensajes.append({'role': 'assistant', 'content': respuesta.content})
            usos = [b for b in respuesta.content if b.type == 'tool_use']
            if respuesta.stop_reason != 'tool_use' or not usos:
                break
            resultados = []
            for uso in usos:
                self.get_logger().info('Claude usa %s %s' % (uso.name, json.dumps(uso.input)))
                try:
                    contenido = self._ejecutar_herramienta(uso.name, uso.input or {}, chat_id)
                    resultados.append({'type': 'tool_result', 'tool_use_id': uso.id,
                                       'content': contenido})
                except Exception as e:  # noqa: BLE001 - se le devuelve el error a Claude
                    resultados.append({'type': 'tool_result', 'tool_use_id': uso.id,
                                       'content': 'Error: %s' % e, 'is_error': True})
            mensajes.append({'role': 'user', 'content': resultados})

        final = '\n'.join(b.text for b in respuesta.content if b.type == 'text').strip()
        # Sólo se guarda el texto de cada turno: así el historial no arrastra imágenes.
        historial.extend([{'role': 'user', 'content': texto},
                          {'role': 'assistant', 'content': final or '(hecho)'}])
        del historial[:-2 * self.max_historial]
        return final

    # ------------------------------------------------------------ Telegram
    def _sondear_telegram(self):
        offset = 0
        usuario_bot = ''
        while rclpy.ok():
            try:
                if not usuario_bot:
                    usuario_bot = self.telegram.nombre_bot().lower()
                for upd in self.telegram.obtener_mensajes(offset):
                    offset = upd['update_id'] + 1
                    msg = upd.get('message') or {}
                    texto = (msg.get('text') or '').strip()
                    chat = msg.get('chat') or {}
                    chat_id = chat.get('id')
                    if chat_id is None or not texto:
                        continue
                    dirigido = True
                    if chat.get('type') in ('group', 'supergroup'):
                        texto, dirigido = self._filtrar_grupo(msg, texto, usuario_bot)
                    self.get_logger().info('Telegram %d%s: %s' % (
                        chat_id, '' if dirigido else ' (ambiente)', texto[:120]))
                    self._despachar(chat_id, texto, dirigido)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn('Error consultando Telegram: %s' % e)
                time.sleep(5)

    @staticmethod
    def _filtrar_grupo(msg, texto, usuario_bot):
        """Prepara un mensaje de grupo: (texto con el nombre de quien habla, dirigido).

        dirigido = True si es un comando, menciona al bot o responde a un mensaje suyo.
        Los demás también se pasan a Claude, que decide si intervenir.
        """
        if texto.startswith('/'):
            return texto, True
        mencion = '@' + usuario_bot
        dirigido = (((msg.get('reply_to_message') or {}).get('from') or {})
                    .get('username') or '').lower() == usuario_bot
        if usuario_bot and mencion in texto.lower():
            inicio = texto.lower().find(mencion)
            texto = ' '.join((texto[:inicio] + texto[inicio + len(mencion):]).split()).strip(' ,:')
            dirigido = True
        autor = msg.get('from') or {}
        nombre = autor.get('first_name') or autor.get('username') or 'alguien'
        return '%s dice: %s' % (nombre, texto or 'hola'), dirigido

    def _despachar(self, chat_id, texto, dirigido=True):
        comando = texto.split()[0].split('@')[0].lower() if texto.startswith('/') else ''
        if comando == '/id':
            self._enviar_seguro(chat_id, 'Tu chat id es %d' % chat_id)
            return
        if chat_id not in self.chats_permitidos:
            self.get_logger().warn('Mensaje ignorado del chat no autorizado %d' % chat_id)
            if dirigido:
                self._enviar_seguro(chat_id, 'No estás autorizado. Tu chat id es %d.' % chat_id)
            return
        if comando in ('/parar', '/stop'):
            # Se atiende aquí mismo, sin esperar a Claude, y corta cualquier plan en curso.
            self.cancelado.set()
            self.parar()
            self._cancelar_navegacion()
            self._enviar_seguro(chat_id, 'Parado.')
            return
        self.cola.put((chat_id, texto, comando, dirigido))

    def _atender_mensajes(self):
        while rclpy.ok():
            chat_id, texto, comando, dirigido = self.cola.get()
            try:
                if comando == '/start':
                    self.telegram.enviar_texto(chat_id, (
                        'Hola, soy el ROSbot. Pregúntame qué veo o dime cómo moverme '
                        '(p. ej. "da dos vueltas en círculo"). Comandos: /foto, /parar, /reset.'))
                elif comando == '/foto':
                    jpeg, edad = self._jpeg_actual()
                    if jpeg is None:
                        self.telegram.enviar_texto(chat_id, 'No llega imagen de la cámara.')
                    else:
                        self.telegram.enviar_foto(chat_id, jpeg, 'Hace %.1f s' % edad)
                elif comando == '/reset':
                    self.historiales.pop(chat_id, None)
                    if self.backend == 'claude_code':
                        ses = self.sesiones.pop(chat_id, None)
                        if ses is not None:
                            ses.cerrar()
                        self.recientes.pop(chat_id, None)
                    self.telegram.enviar_texto(chat_id, 'Conversación reiniciada.')
                elif self.backend == 'claude_code':
                    if dirigido:
                        self.telegram.escribiendo(chat_id)
                    self._preguntar_claude_code(chat_id, texto, dirigido)
                elif not dirigido:
                    continue  # con la API sólo se atienden mensajes dirigidos al bot
                else:
                    self.telegram.escribiendo(chat_id)
                    self.telegram.enviar_texto(chat_id, self._preguntar_api(chat_id, texto))
            except subprocess.TimeoutExpired:
                self._enviar_seguro(chat_id, 'Claude Code ha tardado demasiado en responder.')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error('Error atendiendo el mensaje: %s' % e)
                self._enviar_seguro(chat_id, 'Ha fallado algo: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    nodo = TelegramAINode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub_cmd.publish(Twist())
        nodo._cancelar_navegacion()
        for ses in getattr(nodo, 'sesiones', {}).values():
            ses.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
