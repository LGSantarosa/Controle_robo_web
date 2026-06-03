"""
Ponte ROS2 → WebSocket para mapa, pose do robô e navegação.

Responsabilidades:
  * Subscribe /map (OccupancyGrid) — converte para PNG e emite 'map_update'
    pelo Socket.IO (tanto no modo SLAM ao vivo quanto no NAV2 com mapa estático).
  * Subscribe TF map→base_link — emite 'robot_pose' a ~10 Hz.
  * Subscribe /plan (Path) — emite 'plan_update' quando o Nav2 calcula uma rota.
  * Publica PoseStamped em /goal_pose quando o cliente clica no mapa.
  * Salva o mapa em disco via map_saver_cli (usa o /map atual, SLAM ou NAV2).

Usa o contexto global do rclpy (já inicializado pelo ROS2Controller).
Roda um executor próprio em thread daemon pra processar callbacks.
"""
from __future__ import annotations

import base64
import logging
import math
import os
import subprocess
import threading
import time
from io import BytesIO
from typing import Optional

import numpy as np
from PIL import Image

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers

# slam_toolbox só está no ambiente quando o pacote está instalado (Pi/dev com
# ROS). Os modos teleop/nav2 não dependem dele — import tolerante.
try:
    from slam_toolbox.srv import LoopClosure, ToggleInteractive
except ImportError:  # pragma: no cover
    LoopClosure = ToggleInteractive = None


log = logging.getLogger(__name__)


def _occupancy_to_png_b64(grid: OccupancyGrid) -> str:
    """Converte um OccupancyGrid em PNG grayscale (base64).

    Convenção OccupancyGrid: -1 desconhecido, 0 livre, 1..100 ocupado (graus
    de confiança). Antes valores 1..49 caíam em "desconhecido" (cinza 205),
    perdendo nuances do inflation layer. Agora mapeia linearmente — 0→255
    (branco), 100→0 (preto), -1 (desconhecido) fica em 205.
    """
    w = grid.info.width
    h = grid.info.height
    arr = np.array(grid.data, dtype=np.int16).reshape((h, w))
    img = np.full((h, w), 205, dtype=np.uint8)
    known = arr >= 0
    occ = arr.clip(0, 100).astype(np.uint16)
    img[known] = (255 - (occ[known] * 255 // 100)).astype(np.uint8)
    # ROS: y cresce pra cima; PNG: y cresce pra baixo — inverte linhas.
    img = np.flipud(img)
    pil = Image.fromarray(img, mode='L')
    buf = BytesIO()
    pil.save(buf, format='PNG', optimize=True)
    return base64.b64encode(buf.getvalue()).decode('ascii')


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Converte quaternion (xyzw) em yaw (radianos)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quat(yaw: float) -> tuple:
    """Converte yaw em quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def build_initialpose(x, y, yaw, stamp):
    """Monta PoseWithCovarianceStamped (frame 'map') pra /initialpose.

    Covariância diagonal moderada: confiante mas não absoluta — no AMCL é a
    dispersão inicial das partículas; no slam_toolbox é quase ignorada (seta
    a pose direto).
    """
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quat(float(yaw))
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0] = 0.25      # var(x)  m²
    cov[7] = 0.25      # var(y)  m²
    cov[35] = 0.0685   # var(yaw) rad² (~15° 1σ)
    msg.pose.covariance = cov
    return msg


def build_interactive_feedback(x, y, yaw, marker_name, stamp, frame='map'):
    """Monta o InteractiveMarkerFeedback que relocaliza no slam_toolbox.

    O slam_toolbox (loop_closure_assistant::processInteractiveFeedback) trata
    o evento MOUSE_UP: lê x,y de `mouse_point` (precisa de mouse_point_valid)
    e o yaw de `pose.orientation`, registrando o nó `marker_name` como movido.
    A re-otimização só acontece quando o serviço manual_loop_closure é chamado.
    """
    msg = InteractiveMarkerFeedback()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.marker_name = str(marker_name)
    msg.event_type = InteractiveMarkerFeedback.MOUSE_UP
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quat(float(yaw))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    msg.mouse_point.x = float(x)
    msg.mouse_point.y = float(y)
    msg.mouse_point_valid = True
    return msg


def latest_marker_name(names):
    """Escolhe o marker do nó mais recente (= pose atual) entre `names`.

    Os markers do slam_toolbox são nomeados pelo id do nó do pose-graph; o
    maior id é o nó mais recente. Comparação numérica (não lexical: '10' > '2')
    e nomes não-inteiros são ignorados. Retorna None se não houver nenhum.
    """
    best_name, best_val = None, None
    for n in names:
        try:
            v = int(n)
        except (TypeError, ValueError):
            continue
        if best_val is None or v > best_val:
            best_name, best_val = str(n), v
    return best_name


class MapBridge:
    """Gerencia todos os tópicos ROS2 relacionados a mapa/navegação."""

    POSE_PUBLISH_HZ = 10.0

    def __init__(self, socketio, mode: str, maps_dir: str):
        self._sock = socketio
        self._mode = mode          # 'teleop' | 'slam' | 'nav2'
        self._maps_dir = maps_dir
        os.makedirs(maps_dir, exist_ok=True)

        if not rclpy.ok():
            rclpy.init()

        self._node: Node = rclpy.create_node('web_map_bridge')

        # No modo sim, force use_sim_time para alinhar o TF Buffer com /clock.
        if os.environ.get('ROBOT_SIM', 'false').lower() == 'true':
            from rclpy.parameter import Parameter
            self._node.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])

        # /map é publicado com durability transient_local (latched).
        # Sem isso o subscriber não recebe a mensagem retida.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._node.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos
        )
        self._node.create_subscription(
            Path, '/plan', self._on_plan, 10
        )
        self._goal_pub = self._node.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        # Relocalização manual no NAV2: /initialpose é consumido pelo AMCL
        # (re-semeia as partículas). OBS: o slam_toolbox em MAPEAMENTO NÃO
        # escuta /initialpose — o caminho do SLAM é via interactive markers
        # (abaixo), não por aqui.
        self._initialpose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Relocalização manual no SLAM: liga o modo interativo do slam_toolbox,
        # move o nó mais recente do pose-graph (= pose atual) publicando um
        # InteractiveMarkerFeedback(MOUSE_UP) em /slam_toolbox/feedback e
        # re-otimiza o grafo via o serviço manual_loop_closure. Markers são
        # lidos sob demanda pelo serviço get_interactive_markers.
        self._slam_feedback_pub = None
        self._toggle_interactive_srv = None
        self._loop_closure_srv = None
        self._get_markers_srv = None
        self._interactive_enabled = False
        if mode == 'slam' and ToggleInteractive is not None:
            self._slam_feedback_pub = self._node.create_publisher(
                InteractiveMarkerFeedback, '/slam_toolbox/feedback', 10
            )
            self._toggle_interactive_srv = self._node.create_client(
                ToggleInteractive, '/slam_toolbox/toggle_interactive_mode'
            )
            self._loop_closure_srv = self._node.create_client(
                LoopClosure, '/slam_toolbox/manual_loop_closure'
            )
            self._get_markers_srv = self._node.create_client(
                GetInteractiveMarkers, '/slam_toolbox/get_interactive_markers'
            )

        # TF map→base_link para rastrear o robô no mapa.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        # Serviço de limpeza do costmap local — chamado entre waypoints para
        # evitar que o robô fique preso em células de custo alto após parar.
        self._clear_costmap_srv = self._node.create_client(
            Empty, '/local_costmap/clear_entirely_local_costmap'
        )

        # Action client do Nav2 — usado pelo runner de waypoints pra saber
        # quando o Nav2 realmente termina um goal (SUCCEEDED/ABORTED/CANCELED)
        # em vez de inferir chegada por distância/yaw.
        self._nav_action = ActionClient(
            self._node, NavigateToPose, 'navigate_to_pose'
        )

        # Guarda o último metadata do mapa para converter clique pixel→mundo
        # no cliente (o cliente já recebe isso no map_update).
        self._last_map_info: Optional[dict] = None
        # Cache do payload completo do último map_update ({info, png_b64}).
        # Permite reemitir pra clientes que conectam depois que o map_server
        # (latched) já foi processado — sem isso, a UI fica em "aguardando /map".
        self._last_map_payload: Optional[dict] = None

        # Waypoint navigation state.
        # _wp_lock protege mutações em _wp_list/_wp_loop/_wp_active/_wp_current_idx
        # e o handle do goal corrente. Sem ele o request HTTP que dispara
        # start/stop_waypoints corre com o _wp_runner (thread daemon) e com
        # os callbacks do executor ROS — race observável em troca rápida de rotas.
        self._wp_lock: threading.Lock = threading.Lock()
        self._wp_list: list = []
        self._wp_loop: bool = False
        self._wp_stop: threading.Event = threading.Event()
        self._wp_thread: Optional[threading.Thread] = None
        self._wp_current_idx: int = 0
        self._wp_active: bool = False
        # Estado do goal corrente na NavigateToPose action. Escrito pelos
        # callbacks (thread do executor ROS) e lido pelo _wp_runner.
        self._wp_goal_done: threading.Event = threading.Event()
        self._wp_goal_status: Optional[int] = None
        self._wp_goal_handle = None

        # Executor próprio — permite spin dos callbacks sem bloquear o Flask.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True

        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='map_bridge_spin'
        )
        self._spin_thread.start()

        self._pose_thread = threading.Thread(
            target=self._pose_loop, daemon=True, name='map_bridge_pose'
        )
        self._pose_thread.start()

        log.info(f"[MapBridge] inicializado (modo={mode}, maps_dir={maps_dir})")

    # ---- Loop do executor ROS2 ----
    def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                log.warning(f"[MapBridge] erro no spin: {e}")

    # ---- Loop de polling de pose (TF map→base_link) ----
    def _pose_loop(self):
        period = 1.0 / self.POSE_PUBLISH_HZ
        while self._running:
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                yaw = _quat_to_yaw(
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                )
                self._sock.emit(
                    'robot_pose',
                    {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()},
                    namespace='/',
                )
            except TransformException:
                # Ainda não tem TF — normal antes do SLAM/AMCL convergir.
                pass
            except Exception as e:
                log.debug(f"[MapBridge] pose loop: {e}")
            time.sleep(period)

    # ---- Callbacks ROS2 ----
    def _on_map(self, msg: OccupancyGrid):
        try:
            png_b64 = _occupancy_to_png_b64(msg)
            info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'origin_yaw': _quat_to_yaw(
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w,
                ),
                'stamp': time.time(),
            }
            self._last_map_info = info
            payload = {'info': info, 'png_b64': png_b64}
            self._last_map_payload = payload
            self._sock.emit('map_update', payload, namespace='/')
            log.info(
                f"[MapBridge] /map {msg.info.width}x{msg.info.height} "
                f"@ {msg.info.resolution:.3f} m/px emitido"
            )
        except Exception as e:
            log.warning(f"[MapBridge] erro convertendo /map: {e}")

    def _on_plan(self, msg: Path):
        try:
            pts = [
                {'x': p.pose.position.x, 'y': p.pose.position.y}
                for p in msg.poses
            ]
            self._sock.emit('plan_update', {'points': pts}, namespace='/')
        except Exception as e:
            log.debug(f"[MapBridge] erro no /plan: {e}")

    # ---- API pública ----
    def get_last_map_payload(self) -> Optional[dict]:
        """Retorna o último {info, png_b64} recebido, ou None se nada ainda."""
        return self._last_map_payload

    # ---- Waypoint navigation ----

    def get_waypoints_state(self) -> dict:
        """Estado atual dos waypoints — emitido para clientes que reconectam."""
        with self._wp_lock:
            return {
                'waypoints': list(self._wp_list),
                'loop':      self._wp_loop,
                'active':    self._wp_active,
                'index':     self._wp_current_idx,
                'total':     len(self._wp_list),
            }

    def start_waypoints(self, waypoints: list, loop: bool = False) -> dict:
        if not waypoints:
            return {'ok': False, 'error': 'lista de waypoints vazia'}
        self.stop_waypoints()
        with self._wp_lock:
            self._wp_list = waypoints
            self._wp_loop = loop
            self._wp_active = True
            self._wp_current_idx = 0
            self._wp_stop.clear()
            self._wp_thread = threading.Thread(
                target=self._wp_runner, daemon=True, name='waypoint_runner'
            )
            self._wp_thread.start()
        log.info(f"[MapBridge] waypoints: {len(waypoints)} pontos, loop={loop}")
        return {'ok': True}

    def stop_waypoints(self) -> dict:
        self._wp_stop.set()
        # Cancela o goal ativo no Nav2 (se houver) pra não deixar o robô
        # continuar indo até o último alvo depois de parar a rota.
        with self._wp_lock:
            handle = self._wp_goal_handle
            thread = self._wp_thread
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as e:
                log.debug(f"[MapBridge] erro ao cancelar goal: {e}")
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        with self._wp_lock:
            self._wp_thread = None
            self._wp_active = False
        self._sock.emit('waypoint_status', {'active': False, 'index': 0, 'total': 0})
        return {'ok': True}

    @staticmethod
    def _safe_name(name) -> str:
        """Sanitiza nome de rota/mapa. Rejeita strings vazias com fallback 'rota'."""
        return ''.join(c for c in (name or '') if c.isalnum() or c in '-_') or 'rota'

    def save_route(self, name: str, waypoints: list = None) -> dict:
        wps = waypoints if waypoints is not None else self._wp_list
        if not wps:
            return {'ok': False, 'error': 'nenhum waypoint para salvar'}
        safe = self._safe_name(name)
        routes_dir = os.path.join(self._maps_dir, 'routes')
        os.makedirs(routes_dir, exist_ok=True)
        path = os.path.join(routes_dir, safe + '.json')
        # Escrita atômica: arquivo temporário no mesmo diretório + os.replace.
        # Sem isso, um crash no meio do dump deixa JSON inválido em disco.
        import json as _json
        import tempfile
        try:
            fd, tmp_path = tempfile.mkstemp(prefix='.' + safe + '.', suffix='.json.tmp', dir=routes_dir)
            try:
                with os.fdopen(fd, 'w') as f:
                    _json.dump({'name': safe, 'waypoints': wps}, f, indent=2)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, path)
            except Exception:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
            log.info(f"[MapBridge] rota salva: {path}")
            return {'ok': True, 'name': safe}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def load_route(self, name: str) -> dict:
        safe = self._safe_name(name)
        path = os.path.join(self._maps_dir, 'routes', safe + '.json')
        try:
            import json as _json
            with open(path) as f:
                data = _json.load(f)
            with self._wp_lock:
                self._wp_list = data['waypoints']
                self._wp_loop = False
                self._wp_active = False
                self._wp_current_idx = 0
                wp_count = len(self._wp_list)
                wp_snapshot = list(self._wp_list)
            log.info(f"[MapBridge] rota carregada: {path} ({wp_count} pontos)")
            return {'ok': True, 'name': safe, 'waypoints': wp_snapshot}
        except FileNotFoundError:
            return {'ok': False, 'error': f'rota "{safe}" não encontrada'}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def list_routes(self) -> dict:
        routes_dir = os.path.join(self._maps_dir, 'routes')
        try:
            names = sorted(
                f[:-5] for f in os.listdir(routes_dir) if f.endswith('.json')
            )
        except FileNotFoundError:
            names = []
        return {'ok': True, 'routes': names}

    def _wp_send_goal_action(self, x: float, y: float, yaw: float = 0.0):
        """Envia um goal via NavigateToPose (action). Os callbacks
        `_on_goal_response` e `_on_goal_result` atualizam `_wp_goal_status`
        e sinalizam `_wp_goal_done` quando o Nav2 termina (SUCCEEDED,
        ABORTED, CANCELED)."""
        self._wp_goal_done.clear()
        self._wp_goal_status = None
        self._wp_goal_handle = None

        if not self._nav_action.wait_for_server(timeout_sec=2.0):
            log.warning("[MapBridge] navigate_to_pose action server indisponível")
            self._wp_goal_status = GoalStatus.STATUS_ABORTED
            self._wp_goal_done.set()
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self._nav_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        log.info(f"[MapBridge] NavigateToPose → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            log.warning(f"[MapBridge] erro no send_goal: {e}")
            self._wp_goal_status = GoalStatus.STATUS_ABORTED
            self._wp_goal_done.set()
            return
        if not handle.accepted:
            log.warning("[MapBridge] goal rejeitado pelo Nav2")
            self._wp_goal_status = GoalStatus.STATUS_ABORTED
            self._wp_goal_done.set()
            return
        self._wp_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            result = future.result()
            self._wp_goal_status = result.status
        except Exception as e:
            log.warning(f"[MapBridge] erro no get_result: {e}")
            self._wp_goal_status = GoalStatus.STATUS_ABORTED
        self._wp_goal_handle = None
        self._wp_goal_done.set()

    def _wp_runner(self):
        total        = len(self._wp_list)
        TIMEOUT      = 120.0  # rede de segurança — Nav2 já tem progress_checker
        MAX_RETRIES  = 2      # tentativas extras quando o Nav2 aborta

        def _send(i: int):
            self._wp_current_idx = i
            wp = self._wp_list[i]
            self._sock.emit('waypoint_status', {'active': True, 'index': i, 'total': total})
            # Limpa o costmap local antes de enviar o próximo goal — evita
            # que o robô fique preso em células de custo alto após parar.
            if self._clear_costmap_srv.wait_for_service(timeout_sec=1.0):
                self._clear_costmap_srv.call_async(Empty.Request())
            time.sleep(0.5)
            if not self._wp_stop.is_set():
                self._wp_send_goal_action(wp['x'], wp['y'], wp.get('yaw', 0.0))

        def _advance() -> bool:
            """Avança idx; retorna False quando a rota terminou (sem loop)."""
            nonlocal idx, retries
            retries = 0
            idx += 1
            if idx >= total:
                if self._wp_loop:
                    idx = 0
                    return True
                return False
            return True

        idx = 0
        retries = 0
        _send(idx)
        t0 = time.monotonic()

        while not self._wp_stop.is_set():
            # Espera o Nav2 responder. Se demorar além de TIMEOUT, cancela e
            # pula — cobre o caso do action server travar sem devolver status.
            if not self._wp_goal_done.wait(timeout=0.5):
                if time.monotonic() - t0 > TIMEOUT:
                    log.warning(f"[MapBridge] timeout {TIMEOUT}s waypoint {idx + 1}/{total} — cancelando")
                    if self._wp_goal_handle is not None:
                        try:
                            self._wp_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    self._sock.emit('waypoint_status', {
                        'active': True, 'index': idx, 'total': total, 'timeout': True,
                    })
                    if not _advance():
                        break
                    _send(idx)
                    t0 = time.monotonic()
                continue

            status = self._wp_goal_status

            if status == GoalStatus.STATUS_SUCCEEDED:
                log.info(f"[MapBridge] waypoint {idx + 1}/{total} concluído")
                if not _advance():
                    break
                _send(idx)
                t0 = time.monotonic()

            elif status == GoalStatus.STATUS_ABORTED:
                retries += 1
                if retries <= MAX_RETRIES:
                    log.warning(f"[MapBridge] waypoint {idx + 1}/{total} abortado — tentativa {retries}/{MAX_RETRIES}")
                    self._sock.emit('waypoint_status', {
                        'active': True, 'index': idx, 'total': total, 'retry': retries,
                    })
                    time.sleep(2.0)  # dá tempo do Nav2 se recuperar
                    _send(idx)
                    t0 = time.monotonic()
                else:
                    log.warning(f"[MapBridge] pulando waypoint {idx + 1}/{total} após {MAX_RETRIES} tentativas")
                    self._sock.emit('waypoint_status', {
                        'active': True, 'index': idx, 'total': total, 'skipped': True,
                    })
                    if not _advance():
                        break
                    _send(idx)
                    t0 = time.monotonic()

            elif status == GoalStatus.STATUS_CANCELED:
                # Cancelado externamente (stop_waypoints) — sai do loop.
                break

            else:
                log.warning(f"[MapBridge] status inesperado do goal: {status}")
                if not _advance():
                    break
                _send(idx)
                t0 = time.monotonic()

        with self._wp_lock:
            self._wp_active = False
            loop_was_on = self._wp_loop
        if idx >= total and not loop_was_on:
            self._sock.emit('waypoint_status', {
                'active': False, 'index': total, 'total': total, 'done': True,
            })
        else:
            self._sock.emit('waypoint_status', {'active': False, 'index': idx, 'total': total})

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> dict:
        """Publica PoseStamped em /goal_pose (frame 'map')."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(float(yaw))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._goal_pub.publish(msg)
        log.info(f"[MapBridge] /goal_pose → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        return {'ok': True, 'x': x, 'y': y, 'yaw': yaw}

    def set_pose(self, x: float, y: float, yaw: float = 0.0) -> dict:
        """Relocaliza o robô no mapa (frame 'map'). Caminho conforme o modo:

        * nav2  → publica PoseWithCovarianceStamped em /initialpose; o AMCL
          re-semeia as partículas em torno da pose.
        * slam  → move o nó atual do pose-graph do slam_toolbox via interactive
          markers + manual_loop_closure (ver _set_pose_slam), porque em
          mapeamento o slam_toolbox NÃO escuta /initialpose.
        """
        if self._mode == 'slam':
            return self._set_pose_slam(x, y, yaw)
        stamp = self._node.get_clock().now().to_msg()
        msg = build_initialpose(x, y, yaw, stamp)
        self._initialpose_pub.publish(msg)
        log.info(f"[MapBridge] /initialpose → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        return {'ok': True, 'x': x, 'y': y, 'yaw': yaw}

    def _call_service_sync(self, client, request, timeout: float = 3.0):
        """Chama um serviço e espera a resposta. Seguro fora da thread do
        executor (o _spin_loop roda em paralelo e completa o future)."""
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _set_pose_slam(self, x: float, y: float, yaw: float) -> dict:
        """Relocaliza no slam_toolbox (mapeamento) via interactive markers.

        1) garante o modo interativo ligado (markers passam a existir);
        2) acha o nó mais recente (= pose atual) via get_interactive_markers;
        3) publica feedback MOUSE_UP movendo esse nó para (x, y, yaw);
        4) chama manual_loop_closure → CorrectPoses() aplica/re-otimiza.
        """
        if ToggleInteractive is None:
            return {'ok': False, 'error': 'slam_toolbox indisponível no ambiente'}

        # 1. liga o modo interativo (uma vez) — sem ele não há markers.
        if not self._interactive_enabled:
            if self._call_service_sync(
                self._toggle_interactive_srv, ToggleInteractive.Request()
            ) is None:
                return {'ok': False, 'error': 'toggle_interactive_mode indisponível'}
            self._interactive_enabled = True
            time.sleep(1.0)  # deixa o slam_toolbox montar os markers

        # 2. nó mais recente do pose-graph (tenta por alguns segundos).
        name = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            resp = self._call_service_sync(
                self._get_markers_srv, GetInteractiveMarkers.Request()
            )
            if resp is not None:
                name = latest_marker_name([m.name for m in resp.markers])
            if name is not None:
                break
            time.sleep(0.3)
        if name is None:
            return {'ok': False, 'error': 'sem nós no pose-graph (mapa ainda vazio?)'}

        # 3. move o nó: feedback MOUSE_UP com a pose nova.
        stamp = self._node.get_clock().now().to_msg()
        self._slam_feedback_pub.publish(
            build_interactive_feedback(x, y, yaw, name, stamp)
        )
        # 4. aplica/re-otimiza o grafo com o nó movido.
        self._call_service_sync(self._loop_closure_srv, LoopClosure.Request())
        log.info(
            f"[MapBridge] slam set_pose: nó {name} → "
            f"({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        )
        return {'ok': True, 'x': x, 'y': y, 'yaw': yaw, 'node': name}

    def save_map(self, name: str) -> dict:
        """Salva o mapa atual em disco via map_saver_cli."""
        safe = self._safe_name(name)
        base_path = os.path.join(self._maps_dir, safe)
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', base_path,
            '--ros-args', '-p', 'map_subscribe_transient_local:=true',
        ]
        try:
            proc = subprocess.run(
                cmd, capture_output=True, text=True, timeout=30
            )
        except subprocess.TimeoutExpired:
            return {'ok': False, 'error': 'timeout ao salvar mapa'}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

        ok = proc.returncode == 0 and os.path.isfile(base_path + '.yaml')
        out = (proc.stdout + proc.stderr).strip().splitlines()
        tail = '\n'.join(out[-10:])
        if ok:
            log.info(f"[MapBridge] mapa salvo em {base_path}.yaml")
            return {
                'ok': True,
                'yaml': base_path + '.yaml',
                'pgm': base_path + '.pgm',
                'name': safe,
            }
        log.warning(f"[MapBridge] falha ao salvar mapa: {tail}")
        return {'ok': False, 'error': tail or f'exit {proc.returncode}'}

    def shutdown(self):
        self._running = False
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:
            pass
