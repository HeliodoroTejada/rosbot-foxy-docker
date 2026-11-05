"""Claude Code abierto de forma permanente para responder rápido.

Arrancar `claude -p` en cada mensaje cuesta ~2 s. Aquí se arranca una vez por chat
en modo stream-json y se le van mandando los mensajes por la entrada estándar; cada
mensaje termina con una línea {"type": "result", ...} que trae el JSON pedido.
"""

import collections
import json
import os
import queue
import subprocess
import threading
import time


class SesionClaude:

    def __init__(self, ruta_claude, prompt_sistema, esquema, modelo='', esfuerzo='low',
                 timeout=120.0):
        self.cmd = [ruta_claude, '-p', '--input-format', 'stream-json',
                    '--output-format', 'stream-json', '--verbose',
                    '--strict-mcp-config',      # sin los conectores de la cuenta: arranca antes
                    '--tools', '',              # sin herramientas: sólo piensa y contesta
                    '--system-prompt', prompt_sistema,
                    '--json-schema', json.dumps(esquema)]
        if modelo:
            self.cmd += ['--model', modelo]
        if esfuerzo:
            self.cmd += ['--effort', esfuerzo]
        # Sin ANTHROPIC_API_KEY, Claude Code usa la cuenta con la que se inició sesión.
        self.env = {k: v for k, v in os.environ.items() if k != 'ANTHROPIC_API_KEY'}
        self.timeout = timeout
        self.proc = None
        self.turnos = 0

    def viva(self):
        return self.proc is not None and self.proc.poll() is None

    def cerrar(self):
        if self.proc is not None:
            try:
                self.proc.kill()
                self.proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                pass
        self.proc = None

    def _arrancar(self):
        self.cerrar()
        self.proc = subprocess.Popen(
            self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1, cwd='/tmp', env=self.env)
        self.lineas = queue.Queue()
        self.errores = collections.deque(maxlen=30)
        threading.Thread(target=self._leer, args=(self.proc.stdout, self.lineas.put),
                         daemon=True).start()
        threading.Thread(target=self._leer, args=(self.proc.stderr, self.errores.append),
                         daemon=True).start()
        self.turnos = 0

    @staticmethod
    def _leer(flujo, destino):
        for linea in flujo:
            destino(linea)
        destino(None)

    def preguntar(self, contenido):
        """Manda una lista de bloques (texto/imagen) y devuelve el JSON de la respuesta."""
        if not self.viva():
            self._arrancar()
        mensaje = {'type': 'user', 'message': {'role': 'user', 'content': contenido}}
        try:
            self.proc.stdin.write(json.dumps(mensaje) + '\n')
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError):
            self._arrancar()
            self.proc.stdin.write(json.dumps(mensaje) + '\n')
            self.proc.stdin.flush()

        limite = time.monotonic() + self.timeout
        while True:
            resto = limite - time.monotonic()
            if resto <= 0:
                self.cerrar()
                raise TimeoutError('Claude Code ha tardado demasiado en responder.')
            try:
                linea = self.lineas.get(timeout=resto)
            except queue.Empty:
                continue
            if linea is None:
                detalle = ''.join(e for e in self.errores if e)[-300:]
                self.cerrar()
                raise RuntimeError('Claude Code se ha cerrado. %s' % detalle.strip())
            try:
                datos = json.loads(linea)
            except ValueError:
                continue
            if datos.get('type') != 'result':
                continue
            self.turnos += 1
            if datos.get('is_error'):
                raise RuntimeError('Claude Code: %s' % (datos.get('result') or datos.get('subtype')))
            salida = datos.get('structured_output')
            if isinstance(salida, dict):
                return salida
            texto = datos.get('result') or ''
            inicio, fin = texto.find('{'), texto.rfind('}')
            try:
                return json.loads(texto[inicio:fin + 1])
            except ValueError:
                return {'respuesta': texto.strip(), 'acciones': []}
