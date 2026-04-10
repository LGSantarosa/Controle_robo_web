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
    Controlador real que publica cmd_vel via ROS2 para integração com Nav2.

    Tópico de saída: /cmd_vel (geometry_msgs/Twist)
    O Nav2 Collision Monitor intercepta /cmd_vel, filtra obstáculos e
    publica /cmd_vel_filtered. O nó cmd_vel_to_wheels converte para
    /wheel_vel_setpoints que o driver do hoverboard consome.

    Velocidades em unidades SI:
        BASE_LINEAR_SPEED  = velocidade linear base (m/s)
        BASE_ANGULAR_SPEED = velocidade angular base (rad/s)

    Pré-requisito: ROS2 instalado e workspace com robot_nav compilado.
    Execute antes de iniciar o servidor:
        source ~/ros2_ws/install/setup.bash
    """

    # Velocidades base em unidades SI.
    # Multiplicador de velocidade escala esses valores (0.5x–4.0x).
    # Normal (1.0x) = 0.3 m/s / 0.5 rad/s.
    # Boost (□ 2.0x) = 0.6 m/s / 1.0 rad/s.
    BASE_LINEAR_SPEED: float = 0.3   # m/s
    BASE_ANGULAR_SPEED: float = 0.5  # rad/s

    # Limites do multiplicador de velocidade
    SPEED_MULT_MIN: float = 0.8
    SPEED_MULT_MAX: float = 4.0

    # Mapeamento tecla → direção semântica
    _KEY_MAP: Dict[str, str] = {
        'KeyW': 'forward',    'ArrowUp': 'forward',
        'KeyS': 'backward',   'ArrowDown': 'backward',
        'KeyA': 'left',       'ArrowLeft': 'left',
        'KeyD': 'right',      'ArrowRight': 'right',
        'Space': 'stop',
    }

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node

        self.pressed: set = set()
        self._emergency_stop: bool = False
        self._speed_multiplier: float = 1.0
        self._last_gamepad_linear: float = 0.0
        self._last_gamepad_angular: float = 0.0
        self._rclpy = rclpy

        if not rclpy.ok():
            rclpy.init()

        self._node: Node = rclpy.create_node('web_robot_controller')

        # Publica geometry_msgs/Twist para integração com Nav2
        from geometry_msgs.msg import Twist
        self._Twist = Twist

        self._publisher = self._node.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=10,
        )

        print("[ROS2Controller] Nó inicializado. Publicando em /cmd_vel (Nav2)")

    def shutdown(self) -> None:
        """Encerra o nó ROS2 corretamente."""
        try:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
            print("[ROS2Controller] Nó encerrado.")
        except Exception as e:
            print(f"[ROS2Controller] Erro ao encerrar: {e}")

    def _publish(self, linear: float, angular: float) -> None:
        msg = self._Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._publisher.publish(msg)
        print(f"[ROS2Controller] cmd_vel → linear={linear:+.3f} m/s  angular={angular:+.3f} rad/s")

    @property
    def _linear_speed(self) -> float:
        return self.BASE_LINEAR_SPEED * self._speed_multiplier

    @property
    def _angular_speed(self) -> float:
        return self.BASE_ANGULAR_SPEED * self._speed_multiplier

    def set_speed_multiplier(self, mult: float) -> float:
        """Define o multiplicador de velocidade e republica imediatamente."""
        self._speed_multiplier = max(self.SPEED_MULT_MIN, min(self.SPEED_MULT_MAX, mult))
        print(f"[ROS2Controller] Multiplicador de velocidade: {self._speed_multiplier:.2f}x "
              f"(linear={self._linear_speed:.0f}, angular={self._angular_speed:.0f})")

        # Republica com a velocidade nova se estiver em movimento
        if not self._emergency_stop:
            if self.pressed:
                # Modo teclado — recalcula com teclas pressionadas
                linear, angular = self._compute_cmd_vel()
                self._publish(linear, angular)
            elif abs(self._last_gamepad_linear) > 0.01 or abs(self._last_gamepad_angular) > 0.01:
                # Modo gamepad — recalcula com último eixo
                linear = self._last_gamepad_linear * self._linear_speed
                angular = -self._last_gamepad_angular * self._angular_speed
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

        return lin * self._linear_speed, ang * self._angular_speed

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
            linear = gp_linear * self._linear_speed
            angular = -gp_angular * self._angular_speed
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
