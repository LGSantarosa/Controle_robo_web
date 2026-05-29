from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

# Este módulo define a interface do controlador do robô,
# uma implementação de echo (EchoController) para testes sem robô,
# e o controlador real (ROS2Controller) que publica no tópico ROS2.

class RobotController(ABC):
    @abstractmethod
    def handle_key_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Processa um evento de teclado vindo do cliente remoto.
        Exemplo de evento:
        {
            'type': 'down' | 'up',
            'key': 'ArrowUp' | 'KeyW' | ...,
            'code': 'ArrowUp' | 'KeyW' | ...,
            'repeat': bool,
        }

        Deve retornar um dicionário opcional com a forma:
        { 'command': 'forward'|'backward'|'left'|'right'|'stop', 'action': 'start'|'stop', 'code': 'KeyW' }
        """
        raise NotImplementedError

    def handle_gamepad_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Processa um evento de gamepad (controle PS4/Xbox) com valores analógicos.
        Exemplo de evento:
        {
            'type': 'axis',
            'linear': float,    # -1.0 (ré) a 1.0 (frente) — eixo Y do stick esquerdo
            'angular': float,   # -1.0 (esquerda) a 1.0 (direita) — eixo X do stick esquerdo
        }
        ou:
        {
            'type': 'button',
            'button': str,      # nome do botão (ex: 'cross', 'circle', 'l2', 'r2')
            'value': float,     # 0.0 a 1.0 para triggers, 0 ou 1 para botões digitais
            'pressed': bool,
        }

        Retorna um dicionário com:
        { 'command': str, 'action': str, 'linear': float, 'angular': float, 'left_speed': float, 'right_speed': float }
        """
        return None


class EchoController(RobotController):
    def __init__(self) -> None:
        # Conjunto de teclas atualmente pressionadas (controle simples de estado)
        self.pressed = set()

    def handle_key_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        etype = event.get('type')
        code = event.get('code') or event.get('key')
        repeat = event.get('repeat', False)

        if etype == 'down' and not repeat:
            self.pressed.add(code)
        elif etype == 'up':
            self.pressed.discard(code)

        mapping = {
            'KeyW': 'forward', 'KeyS': 'backward',
            'KeyA': 'left',    'KeyD': 'right',
            'Space': 'stop',
            'ArrowUp': 'forward',   'ArrowDown': 'backward',
            'ArrowLeft': 'left',    'ArrowRight': 'right',
        }

        cmd = mapping.get(code)
        if cmd:
            action = 'start' if etype == 'down' else 'stop'
            print(f"[EchoController] {action} {cmd} (code={code})")
            return {'command': cmd, 'action': action, 'code': code}
        else:
            print(f"[EchoController] {etype} {code}")
            return None

    def handle_gamepad_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        etype = event.get('type')
        if etype == 'axis':
            linear = float(event.get('linear', 0))
            angular = float(event.get('angular', 0))
            if abs(linear) < 0.05 and abs(angular) < 0.05:
                cmd = 'stop'
            elif abs(linear) >= abs(angular):
                cmd = 'forward' if linear > 0 else 'backward'
            else:
                cmd = 'right' if angular > 0 else 'left'
            action = 'stop' if cmd == 'stop' else 'start'
            print(f"[EchoController] gamepad {action} {cmd} (L={linear:.2f} A={angular:.2f})")
            return {'command': cmd, 'action': action, 'linear': linear, 'angular': angular,
                    'left_speed': 0, 'right_speed': 0}
        elif etype == 'button':
            btn = event.get('button', '')
            pressed = event.get('pressed', False)
            print(f"[EchoController] gamepad button {btn} {'pressed' if pressed else 'released'}")
            if btn == 'cross' and pressed:
                return {'command': 'stop', 'action': 'stop', 'linear': 0, 'angular': 0,
                        'left_speed': 0, 'right_speed': 0}
        return None


class ROS2Controller(RobotController):
    """
    Controlador real que publica em /web_vel via ROS2 para integração com Nav2.

    Tópico de saída: /web_vel (geometry_msgs/Twist) — entrada de priority 50 do
    twist_mux. O mux arbitra com joy_vel (PS4, prio 100), key_vel (WASD, 90) e
    nav_vel (Nav2/trekking, 10). Saída do mux: /cmd_vel, consumido pelo
    cmd_vel_to_wheels.

    Velocidades em unidades SI:
        BASE_LINEAR_SPEED  = velocidade linear base (m/s)
        BASE_ANGULAR_SPEED = velocidade angular base (rad/s)

    Pré-requisito: ROS2 instalado e workspace com robot_nav compilado.
    Execute antes de iniciar o servidor:
        source install/setup.bash
    """

    # Velocidades base em unidades SI.
    # Multiplicador de velocidade escala esses valores (0.5x–4.0x).
    # Normal (1.0x) = 0.3 m/s / 6.0 rad/s.
    # Boost (□ 2.0x) = 0.6 m/s / 12 rad/s (cliparia em ±1000 no wheel cmd).
    #
    # Angular ficou em 6.0 rad/s por um problema FÍSICO, NÃO de controle:
    # o robô NÃO TEM SUSPENSÃO, então as 4 rodas não apoiam uniformemente no
    # chão e falta fricção suficiente/confiável — em velocidade baixa o robô
    # não roda direito no eixo (as rodas aliviadas patinam). Com 6.0 rad/s o
    # comando bate ~±600 (60% PWM) e vence o atrito das rodas carregadas.
    # Boost satura mas é proposital (giro rápido pra correção fina).
    #
    # IMPORTANTE: isto NÃO tem relação com o bug de direção da placa traseira
    # (cabos L/R trocados) corrigido em 2026-05-30 (commit 7115b09 / mega_bridge
    # _fb_map). Aquele era de feedback/odometria; este é tração por falta de
    # suspensão. Não confundir os dois ao mexer aqui.
    BASE_LINEAR_SPEED: float = 0.3   # m/s
    BASE_ANGULAR_SPEED: float = 6.0  # rad/s

    # Limites do multiplicador de velocidade.
    # MIN bate com o `min` do slider em index.html (0.5) e permite que o
    # preset "Ajuste fino" do gamepad (0.75×) e do botão (○) passem sem
    # clipagem silenciosa.
    SPEED_MULT_MIN: float = 0.5
    SPEED_MULT_MAX: float = 4.0

    # Mapeamento tecla → direção semântica
    _KEY_MAP: Dict[str, str] = {
        'KeyW': 'forward',    'ArrowUp': 'forward',
        'KeyS': 'backward',   'ArrowDown': 'backward',
        'KeyA': 'left',       'ArrowLeft': 'left',
        'KeyD': 'right',      'ArrowRight': 'right',
        'Space': 'stop',
    }

    def __init__(self, enable_publish: bool = True) -> None:
        import rclpy
        import threading
        import time
        from rclpy.node import Node

        # enable_publish=False (WEB_TELEOP off, PLANO_HEADLESS_2026-05-22 Fase 2):
        # o nó sobe (rclpy/publisher vivos pra não complicar o ciclo de vida do
        # rclpy compartilhado com as bridges), mas o republicador a 50 Hz NÃO
        # inicia e _publish vira no-op. Saída agora é /web_vel (mux prio 50),
        # então mesmo se vazasse não competiria com a saída do mux em /cmd_vel.
        self._publish_enabled: bool = enable_publish
        self.pressed: set = set()
        self._emergency_stop: bool = False
        self._speed_multiplier: float = 1.0
        self._last_gamepad_linear: float = 0.0
        self._last_gamepad_angular: float = 0.0
        self._last_printed: tuple = (None, None)
        self._rclpy = rclpy
        self._time = time

        if not rclpy.ok():
            rclpy.init()

        self._node: Node = rclpy.create_node('web_robot_controller')

        # Publica geometry_msgs/Twist para integração com Nav2
        from geometry_msgs.msg import Twist
        self._Twist = Twist

        self._publisher = self._node.create_publisher(
            Twist,
            '/web_vel',
            qos_profile=10,
        )

        # Republicador a 50 Hz. O firmware da MEGA tem watchdog de 500 ms
        # (SETPOINT_TIMEOUT_MS em firmware/mega_bridge/src/main.cpp): sem
        # novo setpoint nesse intervalo, zera os motores. O navegador manda
        # apenas um evento por mousedown/keyup — sem republicar, cada
        # clique vira um pulso curto.
        self._pub_stop = threading.Event()
        if self._publish_enabled:
            self._pub_thread = threading.Thread(
                target=self._publish_loop, daemon=True, name='cmd_vel_republisher'
            )
            self._pub_thread.start()
            print("[ROS2Controller] Nó inicializado. Publicando em /web_vel @ 50 Hz (mux prio 50)")
        else:
            self._pub_thread = None
            print("[ROS2Controller] WEB_TELEOP=off — nó vivo, SEM publicar em "
                  "/web_vel (movimento via PS4/WASD; web é só visualização).")

    def force_stop(self) -> None:
        """Zera teclas pressionadas + último eixo de gamepad e publica Twist(0).

        Chamado quando um cliente Socket.IO cai com tecla segurada: sem isso
        o republicador a 50 Hz continua mandando o último /web_vel até o
        cliente reconectar (ou o watchdog do firmware estourar 500 ms).
        """
        try:
            self.pressed.clear()
            self._last_gamepad_linear = 0.0
            self._last_gamepad_angular = 0.0
            self._publish(0.0, 0.0)
        except Exception:
            pass

    def shutdown(self) -> None:
        """Encerra o nó ROS2 (sem mexer no contexto global do rclpy).

        Quem chama `rclpy.shutdown()` é o `_shutdown_all` em `app.py`, depois
        que todas as bridges (`MapBridge`, `TrekkingBridge`, `NavMetricsCollector`)
        terminaram. Se este método derrubasse o contexto, as outras bridges
        morreriam no meio de um callback.
        """
        try:
            self._pub_stop.set()
            if self._pub_thread is not None:
                self._pub_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._node.destroy_node()
            print("[ROS2Controller] Nó encerrado.")
        except Exception as e:
            print(f"[ROS2Controller] Erro ao encerrar: {e}")

    def _publish(self, linear: float, angular: float) -> None:
        # WEB_TELEOP off: no-op. Cobre tudo (force_stop no disconnect inclusive),
        # senão o Twist(0) do force_stop voltaria a publicar na saída do twist_mux.
        if not self._publish_enabled:
            return
        msg = self._Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._publisher.publish(msg)
        # Log só quando o valor muda — senão a republicação a 50 Hz polui o stdout.
        rounded = (round(linear, 3), round(angular, 3))
        if rounded != self._last_printed:
            print(f"[ROS2Controller] web_vel → linear={linear:+.3f} m/s  angular={angular:+.3f} rad/s")
            self._last_printed = rounded

    def _publish_loop(self) -> None:
        PERIOD = 0.02  # 50 Hz
        while not self._pub_stop.is_set():
            if self._emergency_stop:
                self._publish(0.0, 0.0)
            elif self.pressed:
                linear, angular = self._compute_cmd_vel()
                self._publish(linear, angular)
            elif abs(self._last_gamepad_linear) > 0.01 or abs(self._last_gamepad_angular) > 0.01:
                linear = self._last_gamepad_linear * self.linear_speed
                angular = -self._last_gamepad_angular * self.angular_speed
                self._publish(linear, angular)
            # else: nada ativo — deixa o watchdog do firmware zerar.
            self._time.sleep(PERIOD)

    @property
    def linear_speed(self) -> float:
        return self.BASE_LINEAR_SPEED * self._speed_multiplier

    @property
    def angular_speed(self) -> float:
        return self.BASE_ANGULAR_SPEED * self._speed_multiplier

    def set_speed_multiplier(self, mult: float) -> float:
        """Define o multiplicador de velocidade e republica imediatamente."""
        self._speed_multiplier = max(self.SPEED_MULT_MIN, min(self.SPEED_MULT_MAX, mult))
        print(f"[ROS2Controller] Multiplicador de velocidade: {self._speed_multiplier:.2f}x "
              f"(linear={self.linear_speed:.0f}, angular={self.angular_speed:.0f})")

        # Republica com a velocidade nova se estiver em movimento
        if not self._emergency_stop:
            if self.pressed:
                # Modo teclado — recalcula com teclas pressionadas
                linear, angular = self._compute_cmd_vel()
                self._publish(linear, angular)
            elif abs(self._last_gamepad_linear) > 0.01 or abs(self._last_gamepad_angular) > 0.01:
                # Modo gamepad — recalcula com último eixo
                linear = self._last_gamepad_linear * self.linear_speed
                angular = -self._last_gamepad_angular * self.angular_speed
                self._publish(linear, angular)

        return self._speed_multiplier

    def _compute_cmd_vel(self) -> tuple:
        """
        Calcula linear (m/s) e angular (rad/s) com base nas teclas pressionadas.
        Suporta movimento composto (ex.: frente + direita ao mesmo tempo).
        Retorna (linear, angular) para publicar em /cmd_vel.
        """
        fwd = any(k in self.pressed for k in ('KeyW', 'ArrowUp'))
        bwd = any(k in self.pressed for k in ('KeyS', 'ArrowDown'))
        lft = any(k in self.pressed for k in ('KeyA', 'ArrowLeft'))
        rgt = any(k in self.pressed for k in ('KeyD', 'ArrowRight'))

        # +1 frente, -1 ré
        lin = (1.0 if fwd else 0.0) - (1.0 if bwd else 0.0)
        # +1 esquerda, -1 direita (convenção ROS: anti-horário positivo)
        ang = (1.0 if lft else 0.0) - (1.0 if rgt else 0.0)

        return lin * self.linear_speed, ang * self.angular_speed

    def handle_key_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        etype  = event.get('type')
        code   = event.get('code') or event.get('key')
        repeat = event.get('repeat', False)

        # Trava de emergência ativa — ignora tudo e mantém parado
        if self._emergency_stop:
            self._publish(0.0, 0.0)
            return {'command': 'stop', 'action': 'stop', 'code': code}

        cmd = self._KEY_MAP.get(code)
        if not cmd:
            # Tecla sem mapeamento — ignora
            return None

        # Atualiza conjunto de teclas pressionadas
        if etype == 'down' and not repeat:
            if cmd == 'stop':
                self.pressed.clear()
            else:
                self.pressed.add(code)
        elif etype == 'up':
            self.pressed.discard(code)

        # Calcula e publica velocidades resultantes
        linear, angular = self._compute_cmd_vel()
        self._publish(linear, angular)

        action = 'start' if etype == 'down' else 'stop'
        return {'command': cmd, 'action': action, 'code': code}

    def handle_gamepad_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        etype = event.get('type')

        if etype == 'button':
            btn = event.get('button', '')
            is_pressed = event.get('pressed', False)
            # Cross (X) = trava de emergência: enquanto segurado, nada se move
            if btn == 'cross':
                self._emergency_stop = is_pressed
                if is_pressed:
                    self.pressed.clear()
                    self._publish(0.0, 0.0)
                    print("[ROS2Controller] TRAVA DE EMERGÊNCIA ATIVADA (X)")
                else:
                    self._publish(0.0, 0.0)
                    print("[ROS2Controller] Trava de emergência desativada")
                return {'command': 'stop', 'action': 'stop', 'linear': 0, 'angular': 0,
                        'left_speed': 0, 'right_speed': 0, 'emergency': is_pressed}
            # Square / Circle = controle de velocidade (tratado no cliente via set_speed)
            return None

        if etype == 'axis':
            # Trava ativa — ignora joystick e mantém parado
            if self._emergency_stop:
                self._publish(0.0, 0.0)
                return {'command': 'stop', 'action': 'stop', 'linear': 0, 'angular': 0,
                        'left_speed': 0, 'right_speed': 0, 'emergency': True}

            gp_linear = float(event.get('linear', 0))
            gp_angular = float(event.get('angular', 0))

            # Aplica dead zone
            if abs(gp_linear) < 0.05:
                gp_linear = 0.0
            if abs(gp_angular) < 0.05:
                gp_angular = 0.0

            # Salva para republicação instantânea ao mudar velocidade
            self._last_gamepad_linear = gp_linear
            self._last_gamepad_angular = gp_angular

            # Converte joystick para m/s e rad/s (angular: direita positiva no gamepad = negativo no ROS)
            linear = gp_linear * self.linear_speed
            angular = -gp_angular * self.angular_speed
            self._publish(linear, angular)

            # Determina comando semântico para log
            if abs(gp_linear) < 0.05 and abs(gp_angular) < 0.05:
                cmd = 'stop'
            elif abs(gp_linear) >= abs(gp_angular):
                cmd = 'forward' if gp_linear > 0 else 'backward'
            else:
                cmd = 'right' if gp_angular > 0 else 'left'

            action = 'stop' if cmd == 'stop' else 'start'
            return {'command': cmd, 'action': action, 'linear': linear, 'angular': angular,
                    'left_speed': 0, 'right_speed': 0}

        return None
