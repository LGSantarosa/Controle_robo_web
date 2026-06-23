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
import json
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
from nav2_msgs.action import (
    ComputePathToPose,
    NavigateToPose,
    NavigateThroughPoses,
)
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from door_geom import (
    door_on_path,
    door_on_segment,
    pre_door_waypoint,
)


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


def yaw_delta(desired, current):
    """Rotação (rad, em [-π, π]) que leva o yaw `current` ao `desired`.

    Invariante de frame: aplicar este delta ao yaw em odom equivale a girar o
    ponteiro do robô por `delta` no frame map (map→odom é fixo no instante).
    Por isso a web (que conhece o yaw em map) manda só o delta, e o
    pose_estimator soma no yaw da odometria de roda.
    """
    d = float(desired) - float(current)
    return math.atan2(math.sin(d), math.cos(d))


class DoorStore:
    """Portas marcadas pelo usuário (2 batentes por porta), persistidas em
    maps/<mapa>.doors.json. Consumidas pelo door_crossing/scan_sanitizer via
    /doors. Spec: docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md
    """
    MIN_W, MAX_W = 0.4, 2.0

    def __init__(self, path: str):
        self.path = path
        self.doors = []
        try:
            with open(path, encoding='utf-8') as f:
                self.doors = json.load(f).get('doors', [])
        except (OSError, ValueError):
            self.doors = []

    def _save(self):
        with open(self.path, 'w', encoding='utf-8') as f:
            json.dump({'doors': self.doors}, f, indent=1)

    def add(self, a, b) -> dict:
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        w = math.hypot(bx - ax, by - ay)
        if not (self.MIN_W <= w <= self.MAX_W):
            raise ValueError(
                f'vão de {w:.2f} m fora da faixa {self.MIN_W}-{self.MAX_W} m')
        new_id = max((d['id'] for d in self.doors), default=0) + 1
        door = {'id': new_id, 'a': [ax, ay], 'b': [bx, by]}
        self.doors.append(door)
        self._save()
        return door

    def remove(self, door_id) -> bool:
        n = len(self.doors)
        self.doors = [d for d in self.doors if d['id'] != door_id]
        if len(self.doors) != n:
            self._save()
            return True
        return False

    def payload(self) -> str:
        return json.dumps({'doors': self.doors})


class MapBridge:
    """Gerencia todos os tópicos ROS2 relacionados a mapa/navegação."""

    POSE_PUBLISH_HZ = 10.0
    SCAN_PUBLISH_HZ = 5.0
    PRE_DOOR_CLEARANCE = 0.30   # m — folga mínima do ponto-pré-porta até parede
                                # (> inflation_radius 0.25 p/ o planner não recusar)

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
        # /scan ao vivo -> overlay azul no mapa (debug de localização: ver se o
        # scan encaixa nas paredes do mapa com o robô parado).
        self._last_scan_emit = 0.0
        self._node.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
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

        # Portas marcadas (travessia): arquivo ao lado do mapa carregado.
        # /doors é latched (transient_local) — door_crossing/scan_sanitizer
        # recebem o estado atual mesmo subindo depois do app.
        map_file = os.environ.get('ROBOT_MAP_FILE', '')
        stem = os.path.splitext(os.path.basename(map_file))[0] or 'doors'
        self._doors = DoorStore(os.path.join(maps_dir, f'{stem}.doors.json'))
        doors_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._doors_pub = self._node.create_publisher(String, '/doors',
                                                      doors_qos)
        self._doors_pub.publish(String(data=self._doors.payload()))
        # Estado da travessia -> chip na UI
        self._node.create_subscription(String, '/door_zone',
                                       self._on_door_zone, doors_qos)

        # Correção manual de DIREÇÃO no SLAM (robô sem IMU): o slam_toolbox em
        # mapeamento não relocaliza por /initialpose, e mexer no pose-graph
        # deforma o mapa. Em vez disso, publicamos um delta de yaw em
        # trekking/yaw_fix; o pose_estimator gira o ponteiro da odometria de
        # roda e o scan-matcher do slam re-converge — sem tocar o mapa.
        self._yaw_fix_pub = None
        if mode == 'slam':
            self._yaw_fix_pub = self._node.create_publisher(
                Float64, 'trekking/yaw_fix', 10
            )
        # Último yaw do robô em map (do _pose_loop) — base pro delta do yaw_fix.
        self._last_robot_yaw: Optional[float] = None

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
        # 2026-06-18: rota com ponto-pré-porta. Quando o destino fica do outro
        # lado de uma porta marcada, manda [ponto-pré-porta, destino] -> o nav2 já
        # entrega o robô reto e longe na frente da porta, e o door só cruza.
        self._nav_through = ActionClient(
            self._node, NavigateThroughPoses, 'navigate_through_poses'
        )
        # 2026-06-19: calcula o caminho do nav2 SEM mover o robô. Usado pra
        # decidir se a rota cruza uma porta marcada (o /plan REAL curva até a
        # abertura; a reta robô->destino pode passar longe do vão) e, se cruzar,
        # inserir o ponto-pré-porta ANTES de mandar executar.
        self._compute_path = ActionClient(
            self._node, ComputePathToPose, 'compute_path_to_pose'
        )
        self._last_robot_xy: Optional[tuple] = None   # (x, y) do robô em map

        # Guarda o último metadata do mapa para converter clique pixel→mundo
        # no cliente (o cliente já recebe isso no map_update).
        self._last_map_info: Optional[dict] = None
        # Grade de ocupação crua (pra checar folga do ponto-pré-porta). PNG é só
        # pra exibir; aqui guardamos os números pra geometria.
        self._grid = None                  # np.int8 [h, w]: 0 livre, 100 ocupado, -1 desconhecido
        self._grid_meta = None             # (res, origin_x, origin_y, w, h)
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
                self._last_robot_yaw = yaw
                self._last_robot_xy = (x, y)
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
            self._grid = np.asarray(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width)
            self._grid_meta = (msg.info.resolution,
                               msg.info.origin.position.x,
                               msg.info.origin.position.y,
                               msg.info.width, msg.info.height)
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

    def _on_scan(self, msg: LaserScan):
        # Converte o /scan pro frame 'map' e emite pontos pro overlay azul.
        # Throttle (SCAN_PUBLISH_HZ) + downsample pra não afogar o websocket.
        now = time.time()
        if now - self._last_scan_emit < 1.0 / self.SCAN_PUBLISH_HZ:
            return
        try:
            # TF no INSTANTE do scan (não o atual): em giro rápido o scan é de
            # alguns ms atrás; usar o stamp dele alinha os pontos com a pose da
            # hora -> sem o "borrado/girando". Fallback pro mais recente se o
            # buffer não tiver esse stamp exato.
            tf = self._tf_buffer.lookup_transform(
                'map', msg.header.frame_id, msg.header.stamp
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', msg.header.frame_id, rclpy.time.Time()
                )
            except Exception:
                return  # sem TF ainda -> sem overlay
        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        lyaw = _quat_to_yaw(
            tf.transform.rotation.x, tf.transform.rotation.y,
            tf.transform.rotation.z, tf.transform.rotation.w,
        )
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        idx = np.arange(n)
        ang = msg.angle_min + idx * msg.angle_increment + lyaw
        step = max(1, n // 320)   # LD06 ~450 pts -> ~320 (mais denso = legível)
        sel = (np.isfinite(ranges) & (ranges >= msg.range_min)
               & (ranges <= msg.range_max) & (idx % step == 0))
        self._last_scan_emit = now
        if not sel.any():
            return
        rr = ranges[sel]
        xs = (lx + rr * np.cos(ang[sel])).round(3).tolist()
        ys = (ly + rr * np.sin(ang[sel])).round(3).tolist()
        self._sock.emit('scan_update', {'xs': xs, 'ys': ys}, namespace='/')

    # ---- Folga do ponto-pré-porta (option A 2026-06-23) -------------------
    def _point_clear(self, x: float, y: float, clearance: float) -> bool:
        """True se não há célula OCUPADA a menos de `clearance` m de (x,y).
        Sem grade ou ponto fora do mapa -> True (não bloqueia)."""
        if self._grid is None:
            return True
        res, ox, oy, w, h = self._grid_meta
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        rad = int(math.ceil(clearance / res))
        r0, r1 = max(0, row - rad), min(h, row + rad + 1)
        c0, c1 = max(0, col - rad), min(w, col + rad + 1)
        if r0 >= r1 or c0 >= c1:
            return True
        occ = self._grid[r0:r1, c0:c1] >= 50      # 100=ocupado; -1/0 não
        if not occ.any():
            return True
        rr, cc = np.nonzero(occ)
        dr = (rr + r0 - row) * res
        dc = (cc + c0 - col) * res
        return bool(np.all(dr * dr + dc * dc > clearance * clearance))

    def _clear_pre_door_point(self, door, wx, wy):
        """Se o ponto-pré-porta caiu colado em parede (parede LATERAL aperta o
        skid-steer), desliza EM DIREÇÃO À PORTA ao longo do eixo até achar folga
        (a garganta do vão é mais aberta). Mantém original se nada servir."""
        cl = self.PRE_DOOR_CLEARANCE
        if self._point_clear(wx, wy, cl):
            return wx, wy
        ax, ay = door['a']
        bx, by = door['b']
        cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
        dist = math.hypot(cx - wx, cy - wy)
        if dist < 1e-6:
            return wx, wy
        ux, uy = (cx - wx) / dist, (cy - wy) / dist
        d, step, min_standoff = 0.05, 0.05, 0.3
        while d <= dist - min_standoff:
            nx, ny = wx + ux * d, wy + uy * d
            if self._point_clear(nx, ny, cl):
                log.info(f"[MapBridge] ponto-pré-porta colado em parede -> "
                         f"deslocado {d:.2f}m p/ a porta ({nx:.2f},{ny:.2f})")
                return nx, ny
            d += step
        log.warning("[MapBridge] ponto-pré-porta sem folga no eixo; "
                    "mantido original")
        return wx, wy

    # ---- API pública ----
    def get_last_map_payload(self) -> Optional[dict]:
        """Retorna o último {info, png_b64} recebido, ou None se nada ainda."""
        return self._last_map_payload

    # ---- Portas (travessia door_crossing) ----

    def door_cmd(self, data: dict) -> dict:
        try:
            if 'add' in data:
                d = self._doors.add(data['add']['a'], data['add']['b'])
            elif 'del' in data:
                if not self._doors.remove(int(data['del'])):
                    return {'ok': False, 'error': 'porta não encontrada'}
                d = None
            else:
                return {'ok': False, 'error': 'cmd desconhecido'}
        except (ValueError, KeyError, TypeError) as e:
            return {'ok': False, 'error': str(e)}
        self._doors_pub.publish(String(data=self._doors.payload()))
        self._sock.emit('doors_update', self._doors.payload())
        return {'ok': True, 'door': d}

    def get_doors_payload(self) -> str:
        return self._doors.payload()

    def _on_door_zone(self, msg):
        self._sock.emit('door_zone', msg.data)

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

    def _plan_path_xy(self, start_xy, goal_xy, timeout=4.0):
        """Pede ao nav2 o caminho (ComputePathToPose) de start_xy a goal_xy SEM
        executar. Devolve lista de (x,y) ou None (indisponível/timeout/vazio)."""
        if not self._compute_path.wait_for_server(timeout_sec=1.0):
            log.warning("[MapBridge] compute_path_to_pose indisponível")
            return None
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.start = self._pose_stamped(start_xy[0], start_xy[1], 0.0)
        goal.goal = self._pose_stamped(goal_xy[0], goal_xy[1], 0.0)
        done = threading.Event()
        box = {'path': None}

        def _on_result(fut):
            try:
                box['path'] = fut.result().result.path
            except Exception as e:
                log.warning(f"[MapBridge] erro no compute_path result: {e}")
            done.set()

        def _on_resp(fut):
            try:
                h = fut.result()
            except Exception as e:
                log.warning(f"[MapBridge] erro no compute_path: {e}")
                done.set()
                return
            if not h.accepted:
                log.warning("[MapBridge] compute_path rejeitado pelo Nav2")
                done.set()
                return
            h.get_result_async().add_done_callback(_on_result)

        self._compute_path.send_goal_async(goal).add_done_callback(_on_resp)
        if not done.wait(timeout=timeout):
            log.warning("[MapBridge] compute_path timeout")
            return None
        path = box['path']
        if path is None or not path.poses:
            return None
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    def _expand_route_via_plan(self, start_xy, waypoints):
        """Insere o ponto-PRÉ-PORTA antes de cada trecho cujo CAMINHO do nav2
        cruza uma porta marcada -> o nav2 entrega o robô reto/longe na frente da
        porta e o door_crossing só alinha+cruza. Usa o /plan real
        (compute_path_to_pose); se o cálculo falhar num trecho, cai no teste de
        reta como rede de segurança. Web é a ÚNICA que manda goal (sem preempção:
        decide ANTES de executar)."""
        doors = self._doors.doors
        if start_xy is None or not doors:
            return list(waypoints)
        out = []
        prev = tuple(start_xy)
        for wp in waypoints:
            to = (wp['x'], wp['y'])
            path = self._plan_path_xy(prev, to)
            door = (door_on_path(path, doors) if path is not None
                    else door_on_segment(prev, to, doors))
            if door is not None:
                wx, wy, wyaw = pre_door_waypoint(door['a'], door['b'], prev)
                wx, wy = self._clear_pre_door_point(door, wx, wy)
                out.append({'x': wx, 'y': wy, 'yaw': wyaw})
                log.info(f"[MapBridge] porta {door['id']} no caminho "
                         f"{prev}->{to} -> ponto-pré-porta "
                         f"({wx:.2f},{wy:.2f}) inserido")
            out.append(dict(wp))
            prev = to
        return out

    def start_waypoints(self, waypoints: list, loop: bool = False) -> dict:
        if not waypoints:
            return {'ok': False, 'error': 'lista de waypoints vazia'}
        self.stop_waypoints()
        # Se o CAMINHO do nav2 cruza uma porta marcada, insere o ponto-PRÉ-PORTA
        # antes do destino daquele trecho (decidido pelo /plan real, ANTES de
        # executar -> sem guerra de preempção).
        n_in = len(waypoints)
        waypoints = self._expand_route_via_plan(self._last_robot_xy, waypoints)
        if len(waypoints) != n_in:
            log.info(f"[MapBridge] rota expandida {n_in} -> {len(waypoints)} "
                     f"pontos (ponto-pré-porta inserido); robot={self._last_robot_xy}")
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
        TIMEOUT      = 3600.0  # 2026-06-15: 120->3600 (~não cancela). Era ISTO que
                               # cancelava o goal no meio da travessia ("CANCELED
                               # em 120.4s"). O usuário não quer que o robô desista
                               # do ponto; parar = cancelar pela UI.
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

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
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
        return msg

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> dict:
        """Manda o robô pro destino. Se o trajeto RETO cruza uma porta marcada,
        põe o ponto-PRÉ-PORTA na rota (navigate_through_poses [pré-porta, destino])
        -> o nav2 entrega o robô reto e longe na frente da porta, e o door só
        alinha+cruza. Senão, publica /goal_pose normal (clique-pra-ir)."""
        robot = self._last_robot_xy
        door = (door_on_segment(robot, (x, y), self._doors.doors)
                if robot is not None else None)
        log.info(f"[MapBridge] send_goal dest=({x:.2f},{y:.2f}) robot={robot} "
                 f"portas={len(self._doors.doors)} "
                 f"-> door={door['id'] if door else None}")
        if door is not None:
            wx, wy, wyaw = pre_door_waypoint(door['a'], door['b'], robot)
            wx, wy = self._clear_pre_door_point(door, wx, wy)
            poses = [self._pose_stamped(wx, wy, wyaw),
                     self._pose_stamped(x, y, yaw)]
            if not self._nav_through.wait_for_server(timeout_sec=2.0):
                log.warning("[MapBridge] navigate_through_poses indisponível "
                            "-> caindo no /goal_pose direto")
            else:
                g = NavigateThroughPoses.Goal()
                g.poses = poses
                self._nav_through.send_goal_async(g)
                log.info(f"[MapBridge] porta {door['id']} no caminho -> rota "
                         f"[pré-porta ({wx:.2f},{wy:.2f}), destino ({x:.2f},{y:.2f})]")
                return {'ok': True, 'x': x, 'y': y, 'yaw': yaw, 'via_door': door['id']}
        # caminho livre (ou sem pose/porta): destino direto
        self._goal_pub.publish(self._pose_stamped(x, y, yaw))
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

    def _set_pose_slam(self, x: float, y: float, yaw: float) -> dict:
        """SLAM (mapeamento): corrige só a DIREÇÃO (yaw) do robô.

        Posição não dá pra resetar sem deformar o mapa (o slam_toolbox em
        mapeamento só relocaliza mexendo no pose-graph, que re-otimiza e
        distorce o mapa). Então usamos apenas o yaw desejado (a direção que o
        usuário aponta): calculamos o delta vs o yaw atual do robô em map e
        publicamos em trekking/yaw_fix. O pose_estimator gira o ponteiro da
        odometria de roda e o scan-matcher do slam re-converge — mapa intacto.
        x, y são ignorados de propósito.
        """
        if self._yaw_fix_pub is None:
            return {'ok': False, 'error': 'yaw_fix indisponível neste modo'}
        cur = self._last_robot_yaw
        if cur is None:
            return {'ok': False, 'error': 'sem pose do robô ainda (TF map→base_link)'}
        delta = yaw_delta(yaw, cur)
        self._yaw_fix_pub.publish(Float64(data=float(delta)))
        log.info(
            f"[MapBridge] slam yaw_fix: Δyaw={delta:+.3f} rad "
            f"(alvo map={yaw:+.3f}, atual={cur:+.3f})"
        )
        return {'ok': True, 'yaw': yaw, 'delta_yaw': delta, 'mode': 'yaw_only'}

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
