"""
Ponte ROS2 ↔ WebSocket para o modo TREKKING.

Responsabilidades:
  * Subscribe /trekking/state (String JSON) → emite 'trekking_state' (~10 Hz).
  * Publica /trekking/cmd (String JSON) para mandar comandos ao trekking_runner.
  * Persiste rotas em disco (maps/routes/trekking/<nome>.json).

Roda no mesmo processo do Flask, com executor ROS próprio em thread daemon
(mesmo padrão do MapBridge).
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


log = logging.getLogger(__name__)


class TrekkingBridge:

    def __init__(self, socketio, maps_dir: str):
        self._sock = socketio
        self._routes_dir = os.path.join(maps_dir, 'routes', 'trekking')
        os.makedirs(self._routes_dir, exist_ok=True)

        if not rclpy.ok():
            rclpy.init()

        self._running = False
        self._node: Node = rclpy.create_node('web_trekking_bridge')

        self._node.create_subscription(
            String, '/trekking/state', self._on_state, 10
        )
        self._cmd_pub = self._node.create_publisher(
            String, '/trekking/cmd', 10
        )

        self._last_state: Optional[dict] = None
        self._state_lock = threading.Lock()

        # Throttle do estado pro browser — o runner publica a 10 Hz; reemite
        # tudo seria ok, mas se tiver muitos cones a serialização cresce.
        # Mantemos 10 Hz mesmo (vai bem na LAN).
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='trekking_bridge_spin'
        )
        self._spin_thread.start()

        log.info(f'[TrekkingBridge] inicializado (routes_dir={self._routes_dir})')

    def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                log.warning(f'[TrekkingBridge] erro no spin: {e}')

    def shutdown(self):
        # Ordem importante: para o loop → encerra executor (drena callbacks
        # em voo) → só então destroy_node. Sem isso, a thread daemon dá
        # alguns ms de spin sobre um nó já destruído e cospe RCLError no log.
        self._running = False
        try:
            if hasattr(self, '_spin_thread') and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass

    # ---- Callbacks ROS ----
    def _on_state(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            log.debug(f'[TrekkingBridge] state JSON inválido: {e}')
            return
        with self._state_lock:
            self._last_state = data
        try:
            self._sock.emit('trekking_state', data, namespace='/')
        except Exception as e:
            log.debug(f'[TrekkingBridge] emit falhou: {e}')

    # ---- API pública (chamada pelo app.py) ----
    def send_cmd(self, cmd: str, **kwargs) -> dict:
        payload = {'cmd': cmd, **kwargs}
        msg = String(data=json.dumps(payload))
        try:
            self._cmd_pub.publish(msg)
            return {'ok': True}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    # ---- Rotas em disco ----
    def _safe_name(self, name) -> str:
        # Mesma regra do MapBridge._safe_name — alfanumérico + '-_', fallback 'rota'.
        return ''.join(c for c in (name or '') if c.isalnum() or c in '-_') or 'rota'

    def save_route(self, name: str, waypoints: list = None) -> dict:
        wps = waypoints
        if wps is None and self._last_state:
            wps = self._last_state.get('waypoints') or []
        if not wps:
            return {'ok': False, 'error': 'nenhum waypoint para salvar'}
        safe = self._safe_name(name)
        path = os.path.join(self._routes_dir, safe + '.json')
        # Atomic write (tempfile no mesmo diretório + os.replace) — evita JSON
        # parcial em disco caso o processo morra no meio do dump.
        import tempfile
        try:
            fd, tmp_path = tempfile.mkstemp(prefix='.' + safe + '.', suffix='.json.tmp', dir=self._routes_dir)
            try:
                with os.fdopen(fd, 'w') as f:
                    json.dump({
                        'name': safe,
                        'waypoints': wps,
                        'saved_ts': time.time(),
                    }, f, indent=2)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, path)
            except Exception:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
            log.info(f'[TrekkingBridge] rota salva: {path}')
            return {'ok': True, 'name': safe}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def load_route(self, name: str) -> dict:
        safe = self._safe_name(name)
        path = os.path.join(self._routes_dir, safe + '.json')
        try:
            with open(path) as f:
                data = json.load(f)
            wps = data.get('waypoints') or []
            # Envia ao runner via /trekking/cmd
            self.send_cmd('load_waypoints', waypoints=wps)
            return {'ok': True, 'name': safe, 'count': len(wps), 'waypoints': wps}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def list_routes(self) -> dict:
        try:
            names = []
            for f in sorted(os.listdir(self._routes_dir)):
                if f.endswith('.json'):
                    names.append(f[:-5])
            return {'ok': True, 'routes': names}
        except Exception as e:
            return {'ok': False, 'routes': [], 'error': str(e)}
