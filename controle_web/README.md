# Controle Web do Robô (Flask + Socket.IO)

Sistema web para receber comandos de teclado de outros computadores na mesma rede e encaminhá-los para um controlador (ex.: robô).

## Requisitos
- Python 3.10+

## Instalação
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Execução
```bash
python app.py
```
O servidor sobe em `http://0.0.0.0:5000`. Em outro computador da mesma rede, acesse `http://SEU_IP_LOCAL:5000` (ex.: `http://192.168.0.10:5000`).

Para descobrir seu IP local no Linux:
```bash
hostname -I
```

## Uso
- Na página, pressione WASD ou Setas para enviar comandos. Barra de espaço = stop.
- Também há botões para toque (mobile).

## Integração com o robô
A lógica de roteamento de comandos está em `controllers/robot_controller.py`.

- `RobotController`: interface abstrata.
- `EchoController`: implementação de exemplo (apenas imprime os comandos).

Para integrar com o robô:
1. Crie uma classe que implemente `RobotController` (ex.: `MyRobotController`).
2. Converta teclas em comandos de alto nível (forward, left, etc.).
3. Envie os comandos para o robô (ex.: via serial, TCP/UDP, ROS, etc.).
4. Troque a instância do controlador em `app.py`:
```python
from controllers.robot_controller import MyRobotController
controller = MyRobotController(...)
```

## Segurança e rede
- Este exemplo permite CORS de qualquer origem e expõe em `0.0.0.0`. Em produção, restrinja origem, autenticação e rede conforme necessário.
