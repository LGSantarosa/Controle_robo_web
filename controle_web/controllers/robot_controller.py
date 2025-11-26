from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

# Este módulo define a interface do controlador do robô e
# uma implementação de exemplo (EchoController) que apenas mapeia
# teclas para comandos lógicos (frente, ré, esquerda, direita, parar).

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

        Observação para um controlador REAL:
        - Aqui é o lugar para enviar o comando ao robô (ex.: via UDP/TCP/Serial/ROS).
        - Implementar controle de estado (iniciar/parar) conforme o protocolo do robô.
        - Tratar repetição (repeat) se necessário.
        - Retornar a tupla de ação/comando para registrar logs e feedback ao cliente.
        """
        raise NotImplementedError

class EchoController(RobotController):
    def __init__(self) -> None:
        # Conjunto de teclas atualmente pressionadas (controle simples de estado)
        self.pressed = set()

    def handle_key_event(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        # Extrai campos principais do evento
        etype = event.get('type')
        code = event.get('code') or event.get('key')
        repeat = event.get('repeat', False)

        # Atualiza estado local de teclas pressionadas
        if etype == 'down' and not repeat:
            self.pressed.add(code)
        elif etype == 'up':
            self.pressed.discard(code)

        # Mapeia teclas para comandos semânticos do robô (ex.: WASD + setas + espaço)
        mapping = {
            'KeyW': 'forward',
            'KeyS': 'backward',
            'KeyA': 'left',
            'KeyD': 'right',
            'Space': 'stop',
            'ArrowUp': 'forward',
            'ArrowDown': 'backward',
            'ArrowLeft': 'left',
            'ArrowRight': 'right',
        }

        cmd = mapping.get(code)
        if cmd:
            # Ação "start" quando tecla é pressionada; "stop" quando solta
            action = 'start' if etype == 'down' else 'stop'
            print(f"[Controller] {action} {cmd} (code={code})")
            return {'command': cmd, 'action': action, 'code': code}
        else:
            # Tecla sem mapeamento explícito
            print(f"[Controller] {etype} {code}")
            return None
