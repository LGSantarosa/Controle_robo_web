# Aplicação Flask com Socket.IO para receber eventos do navegador
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from controllers.robot_controller import RobotController, ROS2Controller
import logging
import math
import os
import json
import secrets
import signal
import sys
import time
import atexit
from logging.handlers import RotatingFileHandler

# Modo de operação — setado pelo launch.sh via env var. Valores: 'teleop',
# 'slam', 'nav2' ou 'trekking'. Controla quais componentes do SocketIO ficam ativos.
ROBOT_MODE = os.environ.get('ROBOT_MODE', 'teleop').lower()
MAPS_DIR = os.environ.get('ROBOT_MAPS_DIR', os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'maps')
))

# Controle de movimento pela web. Default 'off' (PLANO_HEADLESS_2026-05-22 Fase 2):
# o movimento agora é nativo no ROS (PS4/WASD arbitrados pelo twist_mux). Com
# 'off' os handlers de direção (key_event/gamepad_event/set_speed) viram no-op e o
# ROS2Controller NÃO publica em /web_vel — defesa em profundidade contra um
# bug futuro que reintroduzisse publish() sem checagem (achado B20 anterior).
# Reative com WEB_TELEOP=on (flag --web-teleop do launch.sh) p/ dirigir pelo web.
WEB_TELEOP = os.environ.get('WEB_TELEOP', 'off').lower() == 'on'

# Controlador ROS2 — publica em /web_vel (geometry_msgs/Twist, mux prio 50).
# Pré-requisito: source install/setup.bash antes de iniciar o servidor.
# enable_publish=WEB_TELEOP: com teleop web off, o nó sobe (rclpy vivo) mas o
# republicador a 50 Hz não inicia e _publish vira no-op.
#
# Fallback pro EchoController quando rclpy não está disponível (usuário
# rodando app.py sem source install/setup.bash) — preserva a UI viva pra
# desenvolvimento web; launch.sh sempre sourceia, então caminho feliz é igual.
try:
    controller: RobotController = ROS2Controller(enable_publish=WEB_TELEOP)
except Exception as _e_ctrl:
    logging.getLogger(__name__).warning(
        f"[app] ROS2Controller falhou ({_e_ctrl}); caindo para EchoController. "
        f"Comandos vão só logar — source install/setup.bash antes do app.py "
        f"pra ter publicação real."
    )
    from controllers.robot_controller import EchoController
    controller = EchoController()

# Ponte de mapa/pose/navegação — opcional (só sobe se rclpy importou OK).
map_bridge = None
# Coletor de métricas Nav2 — grava CSV por tentativa pra servir de base
# quando formos ajustar parâmetros do stack de navegação.
nav_metrics = None
# Ponte ROS↔Web do modo TREKKING (pose fundida, waypoints, cones, comandos)
trekking_bridge = None

# rclpy.init() instala seus próprios handlers de SIGINT/SIGTERM que engolem o
# Ctrl+C (o processo fica preso esperando o executor do ROS2 que nunca acorda).
# Sobrescreve com handlers Python que fazem shutdown limpo e saem imediatamente.
_shutting_down = False

def _shutdown_all():
    # Cada bridge destrói só o próprio nó; rclpy.shutdown() é chamado uma
    # única vez no fim, depois que todas as bridges saíram dos callbacks.
    # Sem essa ordem o contexto global cai no meio de um spin_once() e as
    # bridges restantes lançam RCLError.
    try:
        if nav_metrics is not None:
            nav_metrics.shutdown()
    except Exception:
        pass
    try:
        if map_bridge is not None:
            map_bridge.shutdown()
    except Exception:
        pass
    try:
        if trekking_bridge is not None:
            trekking_bridge.shutdown()
    except Exception:
        pass
    try:
        if hasattr(controller, 'shutdown'):
            controller.shutdown()
    except Exception:
        pass
    try:
        import rclpy as _rclpy
        if _rclpy.ok():
            _rclpy.shutdown()
    except Exception:
        pass


def _force_shutdown_full(signum, _frame):
    global _shutting_down
    if _shutting_down:
        os._exit(1)
    _shutting_down = True
    print(f"\n[app] Sinal {signum} recebido, encerrando...", flush=True)
    _shutdown_all()
    os._exit(0)


signal.signal(signal.SIGINT,  _force_shutdown_full)
signal.signal(signal.SIGTERM, _force_shutdown_full)

# Encerra tudo ao sair
atexit.register(_shutdown_all)


def _validate_xy(x, y, *, max_abs: float = 1000.0):
    """Converte e valida coordenadas (rejeita NaN, Inf e absurdos).

    `max_abs` evita que um cliente comprometido mande coordenadas tipo 1e18
    e quebre o Nav2 ou trave o filtro do AMCL. Mapas reais cabem em ±1000m.
    """
    fx = float(x)
    fy = float(y)
    if not (math.isfinite(fx) and math.isfinite(fy)):
        raise ValueError("coordenadas precisam ser finitas")
    if abs(fx) > max_abs or abs(fy) > max_abs:
        raise ValueError(f"coordenadas fora do range ±{max_abs} m")
    return fx, fy


def _validate_yaw(yaw) -> float:
    fy = float(yaw)
    if not math.isfinite(fy):
        raise ValueError("yaw precisa ser finito")
    return fy

# Instancia a aplicação web
app = Flask(__name__)
# SECRET_KEY: prefere FLASK_SECRET_KEY do ambiente (deploy persistente).
# Sem env: gera valor aleatório por processo — sessões não sobrevivem a
# restart, mas isso é OK pra LAN/dev e impede o `change-me` hardcoded de
# ir pra prod.
app.config['SECRET_KEY'] = os.environ.get('FLASK_SECRET_KEY') or secrets.token_hex(32)

# CORS: por padrão restringe à mesma origem (mais seguro). Para liberar a
# UI a clientes de outras máquinas da LAN, exporte CORS_ORIGIN com a
# lista de origens permitidas (separadas por vírgula), ou "*" para tudo.
_cors_env = os.environ.get('CORS_ORIGIN', '').strip()
if _cors_env == '*':
    _cors_origins = '*'
elif _cors_env:
    _cors_origins = [o.strip() for o in _cors_env.split(',') if o.strip()]
else:
    _cors_origins = []

# Cria o servidor Socket.IO (tempo real) com logs habilitados
socketio = SocketIO(
    app,
    cors_allowed_origins=_cors_origins,
    logger=False,
    engineio_logger=False,
    async_mode="threading",
)

# Configuração básica de logs no terminal
logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')
# Suprime o aviso de "production deployment" do Werkzeug (esperado para uso local)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

# ---- Ponte de mapa/navegação (ROS2 ↔ Socket.IO) ----
# Só inicializa se o modo precisar (slam/nav2). Em teleop puro, pula para
# economizar CPU. Falhas não derrubam o servidor — só logam.
if ROBOT_MODE in ('slam', 'nav2'):
    try:
        from map_service import MapBridge
        map_bridge = MapBridge(socketio=socketio, mode=ROBOT_MODE, maps_dir=MAPS_DIR)
    except Exception as e:
        logging.getLogger(__name__).warning(
            f"[app] Falha ao iniciar MapBridge: {e}. Mapa e navegação desabilitados."
        )
        map_bridge = None

# Ponte de trekking — só sobe no modo dedicado.
if ROBOT_MODE == 'trekking':
    try:
        from trekking_service import TrekkingBridge
        trekking_bridge = TrekkingBridge(socketio=socketio, maps_dir=MAPS_DIR)
    except Exception as e:
        logging.getLogger(__name__).warning(
            f"[app] Falha ao iniciar TrekkingBridge: {e}. Modo trekking desabilitado."
        )
        trekking_bridge = None

# Coletor de métricas só faz sentido em NAV2 (action navigate_to_pose ativa).
if ROBOT_MODE == 'nav2':
    try:
        from nav_metrics import NavMetricsCollector
        nav_metrics = NavMetricsCollector(
            log_dir=os.path.join(os.path.dirname(__file__), 'logs', 'nav_metrics')
        )
    except Exception as e:
        logging.getLogger(__name__).warning(
            f"[app] Falha ao iniciar NavMetricsCollector: {e}. Métricas desabilitadas."
        )
        nav_metrics = None

# Log de movimentos (JSON Lines) gravado em arquivo rotativo + arquivo legível.
# Paths absolutos baseados na pasta do app.py — sem isso, rodar de outro diretório
# (ex.: `python -m ...` ou via systemd) grava logs em CWD aleatório.
_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_LOGS_DIR = os.path.join(_APP_DIR, 'logs')
os.makedirs(_LOGS_DIR, exist_ok=True)
movement_logger = logging.getLogger('movements')
movement_logger.setLevel(logging.INFO)
if not movement_logger.handlers:
    _mh = RotatingFileHandler(os.path.join(_LOGS_DIR, 'movements.log'), maxBytes=1_048_576, backupCount=5)
    _mh.setFormatter(logging.Formatter('%(message)s'))
    movement_logger.addHandler(_mh)
    # Logger adicional com linhas legíveis em português
    movement_human = logging.getLogger('movements_human')
    movement_human.setLevel(logging.INFO)
    _mht = RotatingFileHandler(os.path.join(_LOGS_DIR, 'movements.txt'), maxBytes=1_048_576, backupCount=5)
    _mht.setFormatter(logging.Formatter('%(message)s'))
    movement_human.addHandler(_mht)
else:
    movement_human = logging.getLogger('movements_human')

@app.before_request
def _log_request_start():
    # Loga início de cada requisição HTTP — mas pula /socket.io/ pra não
    # inundar o terminal com o long-poll (uma requisição a cada ~25 s por cliente).
    if request.path.startswith('/socket.io/'):
        return
    try:
        app.logger.info(f"HTTP {request.method} {request.path} from {request.remote_addr} UA={request.headers.get('User-Agent','-')}")
    except Exception:
        pass

@app.after_request
def _log_request_end(response):
    if request.path.startswith('/socket.io/'):
        return response
    try:
        app.logger.info(f"HTTP {response.status_code} {request.method} {request.path}")
    except Exception:
        pass
    return response

@app.route('/')
def index():
    # Página principal com a interface do controle
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    # Evento de conexão de um cliente Socket.IO
    app.logger.info(f"Client connected: addr={request.remote_addr} sid={request.sid}")
    emit('server_status', {'message': 'connected'})
    # Informa o modo atual para o cliente decidir qual UI mostrar
    emit('mode_info', {
        'mode': ROBOT_MODE,
        'has_map': map_bridge is not None,
        'has_trekking': trekking_bridge is not None,
        'web_teleop': WEB_TELEOP,
    })
    # Reemite o último map_update cacheado — o /map do map_server é latched
    # (publicado 1x no activate), então clientes que conectam depois dessa
    # primeira emissão ficariam em "aguardando /map" sem isto.
    if map_bridge is not None:
        cached = map_bridge.get_last_map_payload()
        if cached is not None:
            emit('map_update', cached)
        # Restaura waypoints para clientes que reconectaram (F5)
        wp_state = map_bridge.get_waypoints_state()
        if wp_state['waypoints']:
            emit('waypoints_restored', wp_state)


@socketio.on('nav_goal')
def handle_nav_goal(data):
    """Cliente clicou no mapa: publica PoseStamped em /goal_pose."""
    if map_bridge is None:
        emit('nav_goal_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    if ROBOT_MODE != 'nav2':
        emit('nav_goal_ack', {'ok': False, 'error': 'clique-para-ir só funciona em modo NAV2'})
        return
    try:
        x, y = _validate_xy(data.get('x'), data.get('y'))
        yaw = _validate_yaw(data.get('yaw', 0.0))
        result = map_bridge.send_goal(x, y, yaw)
        app.logger.info(f"nav_goal from {request.remote_addr}: ({x:.2f}, {y:.2f})")
        emit('nav_goal_ack', result)
    except Exception as e:
        emit('nav_goal_ack', {'ok': False, 'error': str(e)})


@socketio.on('set_pose')
def handle_set_pose(data):
    """Cliente definiu a pose real no mapa: publica /initialpose (relocaliza)."""
    if map_bridge is None:
        emit('set_pose_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    if ROBOT_MODE not in ('slam', 'nav2'):
        emit('set_pose_ack', {'ok': False, 'error': 'definir pose só vale em SLAM ou NAV2'})
        return
    try:
        x, y = _validate_xy(data.get('x'), data.get('y'))
        yaw = _validate_yaw(data.get('yaw', 0.0))
        result = map_bridge.set_pose(x, y, yaw)
        app.logger.info(f"set_pose from {request.remote_addr}: ({x:.2f}, {y:.2f}, {yaw:.2f})")
        emit('set_pose_ack', result)
    except Exception as e:
        emit('set_pose_ack', {'ok': False, 'error': str(e)})


@socketio.on('start_waypoints')
def handle_start_waypoints(data):
    if map_bridge is None:
        emit('waypoints_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    if ROBOT_MODE != 'nav2':
        emit('waypoints_ack', {'ok': False, 'error': 'waypoints só funcionam em modo NAV2'})
        return
    raw_wps = (data or {}).get('waypoints', [])
    if not isinstance(raw_wps, list):
        emit('waypoints_ack', {'ok': False, 'error': 'waypoints precisa ser lista'})
        return
    try:
        waypoints = []
        for w in raw_wps:
            if not isinstance(w, dict):
                raise ValueError("cada waypoint precisa ser objeto")
            wx, wy = _validate_xy(w.get('x'), w.get('y'))
            wyaw = _validate_yaw(w.get('yaw', 0.0))
            waypoints.append({'x': wx, 'y': wy, 'yaw': wyaw})
    except (TypeError, ValueError) as e:
        emit('waypoints_ack', {'ok': False, 'error': f'waypoint inválido: {e}'})
        return
    loop = bool((data or {}).get('loop', False))
    result = map_bridge.start_waypoints(waypoints, loop)
    emit('waypoints_ack', result)


@socketio.on('stop_waypoints')
def handle_stop_waypoints():
    if map_bridge is None:
        return
    map_bridge.stop_waypoints()


@socketio.on('save_route')
def handle_save_route(data):
    if map_bridge is None:
        emit('save_route_ack', {'ok': False, 'error': 'indisponível'})
        return
    name = (data or {}).get('name', 'rota')
    waypoints = (data or {}).get('waypoints')  # lista vinda do frontend
    result = map_bridge.save_route(name, waypoints)
    emit('save_route_ack', result)


@socketio.on('load_route')
def handle_load_route(data):
    if map_bridge is None:
        emit('load_route_ack', {'ok': False, 'error': 'indisponível'})
        return
    name = (data or {}).get('name', '')
    result = map_bridge.load_route(name)
    emit('load_route_ack', result)


@socketio.on('list_routes')
def handle_list_routes():
    if map_bridge is None:
        emit('list_routes_ack', {'ok': True, 'routes': []})
        return
    emit('list_routes_ack', map_bridge.list_routes())


@socketio.on('save_map')
def handle_save_map(data):
    """Cliente apertou 'Salvar mapa' — chama map_saver_cli."""
    if map_bridge is None:
        emit('save_map_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    name = (data or {}).get('name', 'sala')
    app.logger.info(f"save_map from {request.remote_addr}: name={name}")
    result = map_bridge.save_map(name)
    emit('save_map_ack', result)

@socketio.on('disconnect')
def handle_disconnect():
    # Evento de desconexão de um cliente Socket.IO. Força stop pra não
    # deixar o republicador a 50 Hz mandando o último web_vel enquanto
    # ninguém está conectado (cliente que cai com tecla segurada).
    app.logger.info(f"Client disconnected: addr={request.remote_addr} sid={request.sid}")
    try:
        if hasattr(controller, 'force_stop'):
            controller.force_stop()
    except Exception as e:
        app.logger.debug(f"force_stop falhou: {e}")

@socketio.on('key_event')
def handle_key_event(data):
    # Recebe um evento de tecla do cliente
    # Esperado: { type: 'down'|'up', key: 'KeyW'|'ArrowUp'|..., code: 'KeyW', repeat: bool, seq?: int }
    if not WEB_TELEOP:
        # Movimento pela web desabilitado (Fase 2) — não chama o controller.
        emit('ack', {
            'ok': False,
            'error': 'controle desabilitado — use PS4/WASD',
            'seq': (data or {}).get('seq'),
        }, room=request.sid)
        return
    try:
        app.logger.debug(f"key_event from {request.remote_addr}: {data}")
        # Encaminha o evento para o controlador do robô
        result = controller.handle_key_event(data)
        # Monta o registro padrão do evento para arquivo
        entry = {
            'ts': time.time(),
            'addr': request.remote_addr,
            'sid': request.sid,
            'type': data.get('type'),
            'code': data.get('code') or data.get('key'),
            'repeat': bool(data.get('repeat', False)),
        }
        if isinstance(result, dict):
            entry.update({
                'action': result.get('action'),
                'command': result.get('command'),
            })
        # Grava linha JSON no arquivo de movimentos
        movement_logger.info(json.dumps(entry, ensure_ascii=False))
        # Grava linha legível (português) no arquivo textual
        try:
            ts_readable = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(entry['ts']))
            if entry.get('action') and entry.get('command'):
                cmd_pt = {'forward': 'frente', 'backward': 'ré', 'left': 'esquerda', 'right': 'direita', 'stop': 'parar'}
                act_pt = {'start': 'Iniciar', 'stop': 'Parar'}
                movement_human.info(f"[{ts_readable}] {entry['addr']} {act_pt.get(entry['action'], entry['action'])} {cmd_pt.get(entry['command'], entry['command'])} (code={entry['code']}) sid={entry['sid']}")
            else:
                movement_human.info(f"[{ts_readable}] {entry['addr']} {entry['type']} {entry['code']} sid={entry['sid']}")
        except Exception:
            pass
        # Espelha no terminal uma versão humana do movimento
        try:
            human = entry.get('action') and entry.get('command')
            if human:
                cmd_pt = {'forward': 'frente', 'backward': 'ré', 'left': 'esquerda', 'right': 'direita', 'stop': 'parar'}
                act_pt = {'start': 'Iniciar', 'stop': 'Parar'}
                app.logger.info(f"[Mov] {act_pt.get(entry['action'], entry['action'])} {cmd_pt.get(entry['command'], entry['command'])} (code={entry['code']}) from {entry['addr']}")
            else:
                app.logger.info(f"[Mov] {entry['type']} {entry['code']} from {entry['addr']}")
        except Exception:
            pass

        # Envia ACK para o cliente (usado na UI para indicar "Recebido")
        emit('ack', {
            'ok': True,
            'seq': data.get('seq'),
            'type': entry['type'],
            'code': entry['code'],
            'action': entry.get('action'),
            'command': entry.get('command'),
        }, room=request.sid)
        # Eco opcional para debug na página (mantido comentado)
        # emit('server_echo', {'received': data}, room=request.sid)
    except Exception as e:
        # Em caso de erro, retorna ACK negativo com mensagem
        emit('ack', {
            'ok': False,
            'error': str(e),
            'seq': data.get('seq'),
            'type': data.get('type'),
            'code': data.get('code') or data.get('key'),
        }, room=request.sid)

@socketio.on('gamepad_event')
def handle_gamepad_event(data):
    # Recebe evento de gamepad (controle PS4/Xbox) com valores analógicos
    if not WEB_TELEOP:
        # Movimento pela web desabilitado (Fase 2) — o PS4 entra nativo no ROS.
        emit('gamepad_ack', {
            'ok': False,
            'error': 'controle desabilitado — use PS4/WASD',
        }, room=request.sid)
        return
    try:
        app.logger.debug(f"gamepad_event from {request.remote_addr}: type={data.get('type')} L={data.get('linear','?')} A={data.get('angular','?')}")
        result = controller.handle_gamepad_event(data)

        entry = {
            'ts': time.time(),
            'addr': request.remote_addr,
            'sid': request.sid,
            'input': 'gamepad',
            'type': data.get('type'),
        }
        if data.get('type') == 'axis':
            entry['linear'] = data.get('linear')
            entry['angular'] = data.get('angular')
        elif data.get('type') == 'button':
            entry['button'] = data.get('button')
            entry['pressed'] = data.get('pressed')

        if isinstance(result, dict):
            entry.update({
                'action': result.get('action'),
                'command': result.get('command'),
                'left_speed': result.get('left_speed'),
                'right_speed': result.get('right_speed'),
            })

        movement_logger.info(json.dumps(entry, ensure_ascii=False))

        # Log legível
        try:
            ts_readable = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(entry['ts']))
            if entry.get('action') and entry.get('command'):
                cmd_pt = {'forward': 'frente', 'backward': 'ré', 'left': 'esquerda', 'right': 'direita', 'stop': 'parar'}
                act_pt = {'start': 'Iniciar', 'stop': 'Parar'}
                extra = ''
                if entry.get('left_speed') is not None:
                    extra = f" L={entry['left_speed']:.0f} R={entry['right_speed']:.0f}"
                movement_human.info(f"[{ts_readable}] {entry['addr']} [GAMEPAD] {act_pt.get(entry['action'], entry['action'])} {cmd_pt.get(entry['command'], entry['command'])}{extra} sid={entry['sid']}")
            else:
                movement_human.info(f"[{ts_readable}] {entry['addr']} [GAMEPAD] {entry['type']} btn={entry.get('button','-')} sid={entry['sid']}")
        except Exception:
            pass

        # Log no terminal
        try:
            if result and result.get('command'):
                cmd_pt = {'forward': 'frente', 'backward': 'ré', 'left': 'esquerda', 'right': 'direita', 'stop': 'parar'}
                act_pt = {'start': 'Iniciar', 'stop': 'Parar'}
                app.logger.debug(f"[Gamepad] {act_pt.get(result['action'], result['action'])} {cmd_pt.get(result['command'], result['command'])} L={result.get('left_speed',0):.0f} R={result.get('right_speed',0):.0f} from {entry['addr']}")
        except Exception:
            pass

        emit('gamepad_ack', {
            'ok': True,
            'command': result.get('command') if result else None,
            'action': result.get('action') if result else None,
            'left_speed': result.get('left_speed') if result else None,
            'right_speed': result.get('right_speed') if result else None,
            'emergency': result.get('emergency') if result else None,
        }, room=request.sid)
    except Exception as e:
        emit('gamepad_ack', {
            'ok': False,
            'error': str(e),
        }, room=request.sid)

@socketio.on('set_speed')
def handle_set_speed(data):
    # Altera o multiplicador de velocidade do robô
    if not WEB_TELEOP:
        # Movimento pela web desabilitado (Fase 2) — velocidade vem do teleop_ps4.yaml.
        emit('speed_update', {
            'ok': False,
            'error': 'controle desabilitado — use PS4/WASD',
        }, room=request.sid)
        return
    try:
        mult = float((data or {}).get('multiplier', 1.0))
        if not math.isfinite(mult):
            raise ValueError("multiplier precisa ser finito")
        effective = controller.set_speed_multiplier(mult)
        app.logger.info(f"set_speed from {request.remote_addr}: mult={mult:.2f} → effective={effective:.2f}")
        # Broadcast só no caminho feliz — todos os clientes precisam reagir
        # ao novo multiplicador (UI compartilhada).
        emit('speed_update', {
            'ok': True,
            'multiplier': effective,
            'linear_speed': controller.linear_speed,
            'angular_speed': controller.angular_speed,
        }, broadcast=True)
    except Exception as e:
        # Erro: responde só ao cliente que mandou — broadcast aqui mostraria
        # "Não recebido" pra todo mundo só porque um cliente mandou lixo.
        emit('speed_update', {'ok': False, 'error': str(e)}, room=request.sid)

@socketio.on('client_hello')
def handle_client_hello(payload):
    # Handshake simples para depuração (cliente informa dados básicos)
    app.logger.info(f"client_hello from {request.remote_addr} sid={request.sid} payload={payload}")
    emit('server_hello', {
        'sid': request.sid,
        'msg': 'hello from server',
    })

# ---------------- Trekking (ponto-a-ponto) ----------------

# Espelha o `_on_cmd` do trekking_runner.py — qualquer mudança lá precisa vir
# pra cá, senão app aceita comandos que o runner ignora silenciosamente.
_TREKKING_CMDS = {
    'reset', 'record', 'save_point', 'play', 'stop',
    'load_waypoints', 'clear', 'set_cone',
}
# Apenas estes kwargs passam para o runner — rejeita o resto pra não acabar
# como vetor de injeção (`os.system` numa lib futura, etc.).
_TREKKING_KWARGS = {'waypoints', 'v_max', 'kp_heading', 'kd_heading',
                    'idx', 'cone_x', 'cone_y', 'clear'}

@socketio.on('trekking_cmd')
def handle_trekking_cmd(data):
    """Comandos do painel trekking → /trekking/cmd."""
    if trekking_bridge is None:
        emit('trekking_ack', {'ok': False, 'error': 'trekking indisponível neste modo'})
        return
    cmd = str((data or {}).get('cmd', '')).lower()
    if cmd not in _TREKKING_CMDS:
        emit('trekking_ack', {'ok': False, 'error': f'cmd desconhecido: {cmd}'})
        return
    kwargs = {k: v for k, v in (data or {}).items() if k in _TREKKING_KWARGS}
    result = trekking_bridge.send_cmd(cmd, **kwargs)
    app.logger.info(f"trekking_cmd from {request.remote_addr}: {cmd} {list(kwargs)}")
    emit('trekking_ack', {'cmd': cmd, **result})


@socketio.on('trekking_save_route')
def handle_trekking_save_route(data):
    if trekking_bridge is None:
        emit('trekking_save_ack', {'ok': False, 'error': 'indisponível'})
        return
    name = (data or {}).get('name', 'rota')
    waypoints = (data or {}).get('waypoints')  # opcional — se ausente, usa último estado
    emit('trekking_save_ack', trekking_bridge.save_route(name, waypoints))


@socketio.on('trekking_load_route')
def handle_trekking_load_route(data):
    if trekking_bridge is None:
        emit('trekking_load_ack', {'ok': False, 'error': 'indisponível'})
        return
    name = (data or {}).get('name', '')
    emit('trekking_load_ack', trekking_bridge.load_route(name))


@socketio.on('trekking_list_routes')
def handle_trekking_list_routes():
    if trekking_bridge is None:
        emit('trekking_routes', {'ok': True, 'routes': []})
        return
    emit('trekking_routes', trekking_bridge.list_routes())


if __name__ == '__main__':
    # Sobe o servidor acessível na rede local (0.0.0.0:5000)
    app.logger.info("Starting Socket.IO server on 0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
