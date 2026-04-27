# Aplicação Flask com Socket.IO para receber eventos do navegador
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
from controllers.robot_controller import RobotController, ROS2Controller
import logging
import os
import json
import signal
import sys
import time
import atexit
from logging.handlers import RotatingFileHandler

# Modo de operação — setado pelo launch.sh via env var. Valores: 'teleop',
# 'slam' ou 'nav2'. Controla quais componentes do SocketIO ficam ativos.
ROBOT_MODE = os.environ.get('ROBOT_MODE', 'teleop').lower()
MAPS_DIR = os.environ.get('ROBOT_MAPS_DIR', os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'maps')
))

# Controlador ROS2 — publica em /cmd_vel (geometry_msgs/Twist).
# Pré-requisito: source ~/ros2_ws/install/setup.bash antes de iniciar o servidor.
controller: RobotController = ROS2Controller()

# Ponte de mapa/pose/navegação — opcional (só sobe se rclpy importou OK).
map_bridge = None
# Coletor de métricas Nav2 — grava CSV por tentativa pra servir de base
# quando formos ajustar parâmetros do stack de navegação.
nav_metrics = None
# Stream da câmera RGB-D do robô — JPEG via Socket.IO em ~5 Hz.
camera_bridge = None

# rclpy.init() instala seus próprios handlers de SIGINT/SIGTERM que engolem o
# Ctrl+C (o processo fica preso esperando o executor do ROS2 que nunca acorda).
# Sobrescreve com handlers Python que fazem shutdown limpo e saem imediatamente.
_shutting_down = False

def _shutdown_all():
    try:
        if camera_bridge is not None:
            camera_bridge.shutdown()
    except Exception:
        pass
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
        if hasattr(controller, 'shutdown'):
            controller.shutdown()
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

# Instancia a aplicação web
app = Flask(__name__)
app.config['SECRET_KEY'] = 'change-me'
# Cria o servidor Socket.IO (tempo real) com logs habilitados
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
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

# Stream da câmera — em qualquer modo (sim ou real) onde tiver o tópico.
# Se /camera/image não existir, o subscriber só fica ocioso, sem custo.
if ROBOT_MODE in ('slam', 'nav2', 'teleop'):
    try:
        from camera_bridge import CameraBridge
        camera_bridge = CameraBridge(socketio=socketio)
    except Exception as e:
        logging.getLogger(__name__).warning(
            f"[app] Falha ao iniciar CameraBridge: {e}. Câmera desabilitada."
        )
        camera_bridge = None


# Log de movimentos (JSON Lines) gravado em arquivo rotativo + arquivo legível
os.makedirs('logs', exist_ok=True)
movement_logger = logging.getLogger('movements')
movement_logger.setLevel(logging.INFO)
if not movement_logger.handlers:
    _mh = RotatingFileHandler('logs/movements.log', maxBytes=1_048_576, backupCount=5)
    _mh.setFormatter(logging.Formatter('%(message)s'))
    movement_logger.addHandler(_mh)
    # Logger adicional com linhas legíveis em português
    movement_human = logging.getLogger('movements_human')
    movement_human.setLevel(logging.INFO)
    _mht = RotatingFileHandler('logs/movements.txt', maxBytes=1_048_576, backupCount=5)
    _mht.setFormatter(logging.Formatter('%(message)s'))
    movement_human.addHandler(_mht)
else:
    movement_human = logging.getLogger('movements_human')

@app.before_request
def _log_request_start():
    # Loga início de cada requisição HTTP (método, rota, IP e user-agent)
    try:
        app.logger.info(f"HTTP {request.method} {request.path} from {request.remote_addr} UA={request.headers.get('User-Agent','-')}")
    except Exception:
        pass

@app.after_request
def _log_request_end(response):
    # Loga fim de cada requisição HTTP com status
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
        x = float(data.get('x'))
        y = float(data.get('y'))
        yaw = float(data.get('yaw', 0.0))
        result = map_bridge.send_goal(x, y, yaw)
        app.logger.info(f"nav_goal from {request.remote_addr}: ({x:.2f}, {y:.2f})")
        emit('nav_goal_ack', result)
    except Exception as e:
        emit('nav_goal_ack', {'ok': False, 'error': str(e)})


@socketio.on('start_waypoints')
def handle_start_waypoints(data):
    if map_bridge is None:
        emit('waypoints_ack', {'ok': False, 'error': 'mapa indisponível neste modo'})
        return
    if ROBOT_MODE != 'nav2':
        emit('waypoints_ack', {'ok': False, 'error': 'waypoints só funcionam em modo NAV2'})
        return
    waypoints = (data or {}).get('waypoints', [])
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
    # Evento de desconexão de um cliente Socket.IO
    app.logger.info(f"Client disconnected: addr={request.remote_addr} sid={request.sid}")

@socketio.on('key_event')
def handle_key_event(data):
    # Recebe um evento de tecla do cliente
    # Esperado: { type: 'down'|'up', key: 'KeyW'|'ArrowUp'|..., code: 'KeyW', repeat: bool, seq?: int }
    try:
        app.logger.info(f"key_event from {request.remote_addr}: {data}")
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
        }, broadcast=False)
        # Eco opcional para debug na página (mantido comentado)
        # emit('server_echo', {'received': data}, broadcast=False)
    except Exception as e:
        # Em caso de erro, retorna ACK negativo com mensagem
        emit('ack', {
            'ok': False,
            'error': str(e),
            'seq': data.get('seq'),
            'type': data.get('type'),
            'code': data.get('code') or data.get('key'),
        }, broadcast=False)

@socketio.on('gamepad_event')
def handle_gamepad_event(data):
    # Recebe evento de gamepad (controle PS4/Xbox) com valores analógicos
    try:
        app.logger.info(f"gamepad_event from {request.remote_addr}: type={data.get('type')} L={data.get('linear','?')} A={data.get('angular','?')}")
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
                app.logger.info(f"[Gamepad] {act_pt.get(result['action'], result['action'])} {cmd_pt.get(result['command'], result['command'])} L={result.get('left_speed',0):.0f} R={result.get('right_speed',0):.0f} from {entry['addr']}")
        except Exception:
            pass

        emit('gamepad_ack', {
            'ok': True,
            'command': result.get('command') if result else None,
            'action': result.get('action') if result else None,
            'left_speed': result.get('left_speed') if result else None,
            'right_speed': result.get('right_speed') if result else None,
            'emergency': result.get('emergency') if result else None,
        }, broadcast=False)
    except Exception as e:
        emit('gamepad_ack', {
            'ok': False,
            'error': str(e),
        }, broadcast=False)

@socketio.on('set_speed')
def handle_set_speed(data):
    # Altera o multiplicador de velocidade do robô
    try:
        mult = float(data.get('multiplier', 1.0))
        effective = controller.set_speed_multiplier(mult)
        app.logger.info(f"set_speed from {request.remote_addr}: mult={mult:.2f} → effective={effective:.2f}")
        emit('speed_update', {
            'ok': True,
            'multiplier': effective,
            'linear_speed': controller._linear_speed,
            'angular_speed': controller._angular_speed,
        }, broadcast=True)
    except Exception as e:
        emit('speed_update', {'ok': False, 'error': str(e)}, broadcast=False)

@socketio.on('client_hello')
def handle_client_hello(payload):
    # Handshake simples para depuração (cliente informa dados básicos)
    app.logger.info(f"client_hello from {request.remote_addr} sid={request.sid} payload={payload}")
    emit('server_hello', {
        'sid': request.sid,
        'msg': 'hello from server',
    })

if __name__ == '__main__':
    # Sobe o servidor acessível na rede local (0.0.0.0:5000)
    app.logger.info("Starting Socket.IO server on 0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
