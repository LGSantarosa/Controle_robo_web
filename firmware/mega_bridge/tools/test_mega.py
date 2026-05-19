#!/usr/bin/env python3
"""
Teste end-to-end MEGA ↔ placa hoverboard, SEM ROS2.

Replica exatamente o protocolo que o `mega_bridge.py` usa, mas em script
único pra você validar a transmissão antes de subir o stack ROS inteiro.
Útil pra testar com só uma placa conectada (ex.: só a frente no notebook).

Caminho testado:
    PC --USB-> MEGA --Serial1-> placa FRENTE
    PC <-USB-- MEGA <-Serial1-- placa FRENTE  (feedback)

Uso:
    python3 firmware/mega_bridge/tools/test_mega.py
    python3 firmware/mega_bridge/tools/test_mega.py --port /dev/ttyACM0
    python3 firmware/mega_bridge/tools/test_mega.py --speed 500 --duration 5

Pré-requisitos:
    - Firmware da MEGA flasheado (cd firmware/mega_bridge && pio run -t upload)
    - MEGA plugada via USB → /dev/mega (rodar sudo ./setup_udev.sh primeiro)
    - Placa(s) hoverboard com bateria conectada
    - Cabo Serial1 (TX=18, RX=19, GND) ligado na placa frente
      (verde=RX da placa, azul=TX da placa, preto=GND)

O firmware tem watchdog de 500 ms: se o PC parar de mandar setpoint, ele
zera os motores. Por isso o script manda comando a 50 Hz contínuo.
"""

import argparse
import struct
import sys
import time
from collections import deque


# Mesmas constantes que o firmware (firmware/mega_bridge/include/protocol.h)
START0 = 0xAA
START1 = 0x55

FT_SET_SPEED = 0x01
FT_STATE     = 0x81
FT_IMU       = 0x82
FT_FLOW      = 0x83


def xor8(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


def build_frame(ft: int, payload: bytes) -> bytes:
    """[0xAA 0x55] [ft] [len] [payload] [xor8(ft+len+payload)]"""
    header = bytes([ft, len(payload)])
    chk = xor8(header + payload)
    return bytes([START0, START1]) + header + payload + bytes([chk])


def build_set_speed(steer_front: int, speed_front: int,
                    steer_rear: int, speed_rear: int) -> bytes:
    """Mesmo payload que mega_bridge.py._on_setpoint produz."""
    payload = struct.pack('<hhhh', steer_front, speed_front,
                          steer_rear, speed_rear)
    return build_frame(FT_SET_SPEED, payload)


class Decoder:
    """Decodificador de frames vindos da MEGA — copiado de mega_bridge.py."""

    def __init__(self):
        self._st = 0
        self._type = 0
        self._len = 0
        self._got = 0
        self._buf = bytearray(64)

    def feed(self, b: int):
        if self._st == 0:
            if b == START0:
                self._st = 1
            return None
        if self._st == 1:
            self._st = 2 if b == START1 else 0
            return None
        if self._st == 2:
            self._type = b
            self._st = 3
            return None
        if self._st == 3:
            self._len = b
            self._got = 0
            if self._len > 64:
                self._st = 0
                return None
            self._st = 5 if self._len == 0 else 4
            return None
        if self._st == 4:
            self._buf[self._got] = b
            self._got += 1
            if self._got >= self._len:
                self._st = 5
            return None
        if self._st == 5:
            self._st = 0
            expected = (self._type ^ self._len) & 0xFF
            for i in range(self._len):
                expected ^= self._buf[i]
            if expected == b:
                return self._type, bytes(self._buf[: self._len])
            return None
        return None


class Stats:
    """Acumulador simples pra mostrar o que chegou da MEGA."""

    def __init__(self):
        self.state_count = 0
        self.imu_count = 0
        self.flow_count = 0
        self.last_state = None
        self.last_rpms = deque(maxlen=20)

    def feed_state(self, p: bytes):
        if len(p) != 16:
            return
        rpm_FL, rpm_FR, rpm_RL, rpm_RR, batF, batR = struct.unpack('<hhhhhh', p[:12])
        btn = p[14]
        self.state_count += 1
        self.last_state = {
            'rpm_FL': rpm_FL, 'rpm_FR': rpm_FR,
            'rpm_RL': rpm_RL, 'rpm_RR': rpm_RR,
            'batF': batF / 100.0, 'batR': batR / 100.0,
            'btn': btn,
        }
        self.last_rpms.append((rpm_FL, rpm_FR, rpm_RL, rpm_RR))

    def feed_imu(self, _p: bytes):
        self.imu_count += 1

    def feed_flow(self, _p: bytes):
        self.flow_count += 1

    def print_line(self, label: str):
        s = self.last_state
        if s is None:
            print(f"  [{label}] aguardando primeiro frame STATE da MEGA...")
            return
        print(
            f"  [{label}] "
            f"FL={s['rpm_FL']:+5d} FR={s['rpm_FR']:+5d}  "
            f"RL={s['rpm_RL']:+5d} RR={s['rpm_RR']:+5d}  "
            f"batF={s['batF']:.2f}V batR={s['batR']:.2f}V  "
            f"btn={s['btn']}"
        )


def drain_rx(ser, decoder: Decoder, stats: Stats) -> int:
    """Lê tudo que está no buffer e roteia pros stats. Retorna nº de frames."""
    n_frames = 0
    pending = ser.in_waiting
    if pending == 0:
        return 0
    data = ser.read(pending)
    for b in data:
        frame = decoder.feed(b)
        if frame is None:
            continue
        n_frames += 1
        ft, payload = frame
        if ft == FT_STATE:
            stats.feed_state(payload)
        elif ft == FT_IMU:
            stats.feed_imu(payload)
        elif ft == FT_FLOW:
            stats.feed_flow(payload)
    return n_frames


def send_loop(ser, frame: bytes, duration: float, label: str,
              stats: Stats, decoder: Decoder, period: float = 0.02):
    """Manda o mesmo frame a ~50 Hz durante `duration` segundos.

    Print 5 linhas de status (a cada 20% da duração) e mantém o RX drenando
    pra acumular os frames de feedback que a MEGA está respondendo.
    """
    t_start = time.time()
    t_end = t_start + duration
    t_next_print = t_start
    print_period = max(duration / 5.0, 0.5)
    while time.time() < t_end:
        ser.write(frame)
        drain_rx(ser, decoder, stats)
        if time.time() >= t_next_print:
            stats.print_line(label)
            t_next_print += print_period
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description="Teste end-to-end MEGA ↔ placa")
    ap.add_argument('--port', default='/dev/mega',
                    help='Porta serial da MEGA (default: /dev/mega)')
    ap.add_argument('--baud', type=int, default=230400,
                    help='Baud rate USB MEGA (default: 230400)')
    ap.add_argument('--speed', type=int, default=300,
                    help='Comando speed pra fase "avança" (default: 300)')
    ap.add_argument('--duration', type=float, default=3.0,
                    help='Duração da fase "avança" em s (default: 3)')
    ap.add_argument('--front-only', action='store_true',
                    help='Só comanda a placa da frente; manda 0 pra trás. '
                         'Use quando a placa traseira não está conectada.')
    ap.add_argument('--skip-input', action='store_true',
                    help='Não pede confirmação antes de mandar speed > 0. '
                         'CUIDADO: o robô pode sair andando.')
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("ERRO: pyserial não instalado.  sudo apt install python3-serial")
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"ERRO ao abrir {args.port}: {e}")
        print()
        print("Checklist:")
        print(f"  1) MEGA está plugada?     ls -la {args.port}")
        print(f"  2) Symlink /dev/mega OK?  sudo ./setup_udev.sh")
        print(f"  3) Permissão de grupo?    sudo usermod -aG dialout $USER  (logout depois)")
        sys.exit(1)

    print(f"Porta {args.port} aberta @ {args.baud} baud")
    # Reset por DTR + bootloader + Adafruit_BNO055::begin() (timeout I²C se a
    # IMU não estiver plugada) somam ~1.8 s antes da MEGA emitir o primeiro
    # frame. 2.5 s deixa margem confortável.
    print("Aguardando 2.5s pra MEGA estabilizar (DTR reset + init dos sensores)...")
    time.sleep(2.5)
    ser.reset_input_buffer()

    decoder = Decoder()
    stats = Stats()

    # ----- Fase 1: SP=0 por 2s -----
    print()
    print("Fase 1/3: setpoint=0 por 2s (motores devem ficar parados)")
    frame_zero = build_set_speed(0, 0, 0, 0)
    send_loop(ser, frame_zero, 2.0, "ZERO", stats, decoder)

    if stats.state_count == 0:
        print()
        print("ERRO: nenhum frame STATE recebido da MEGA em 2s.")
        print("Diagnóstico:")
        print(f"  - Firmware flasheado? cd firmware/mega_bridge && pio run -t upload")
        print(f"  - Reset físico da MEGA pode resolver (botão na placa).")
        print(f"  - pio device monitor -b {args.baud} mostra o que ela está enviando.")
        ser.close()
        sys.exit(2)

    print(f"  → {stats.state_count} frames STATE recebidos em 2s "
          f"(esperado ~100 a 50 Hz)")

    # Diagnóstico de transmissão MEGA ↔ placa:
    # Se a placa NÃO está respondendo (cabo, bateria, board errado), o
    # firmware (após o fix C6) publica RPMs = 0 e batF = 0.
    last = stats.last_state
    front_alive = last is not None and last['batF'] > 1.0  # > 1V = placa viva
    rear_alive  = last is not None and last['batR'] > 1.0

    print()
    print("Diagnóstico das placas (com base no batVoltage do feedback):")
    print(f"  Placa FRENTE: {'OK (responde feedback)' if front_alive else 'NÃO RESPONDE'}"
          f"  batF={last['batF']:.2f}V")
    print(f"  Placa TRÁS:   {'OK (responde feedback)' if rear_alive else 'NÃO RESPONDE'}"
          f"  batR={last['batR']:.2f}V")
    print()
    if not front_alive and not rear_alive:
        print("AVISO: nenhuma placa está respondendo. Cheque:")
        print("  - Bateria conectada e carregada?")
        print("  - Cabo Serial1 (frente) ou Serial2 (trás) ligado?")
        print("  - Verde=RX da placa no TX da MEGA (pino 18 ou 16)?")
        print("  - Azul=TX da placa no RX da MEGA (pino 19 ou 17)?")
        print("  - Preto=GND amarrado no GND da MEGA?")
    elif not front_alive:
        print("AVISO: só a placa TRÁS responde. Se você queria testar a")
        print("       FRENTE, cheque o cabo do Serial1.")
    elif not rear_alive:
        print("OK: testando só com placa FRENTE. (Vai mostrar RPM=0 e batR=0")
        print("    para os campos da traseira — esperado.)")

    # ----- Fase 2: avança -----
    if args.speed == 0:
        print("\nSpeed=0 (--speed 0), pulando fase de avanço.")
    else:
        if not args.skip_input:
            print()
            print(f">>> PRÓXIMA FASE vai mandar speed={args.speed} por {args.duration}s.")
            print(">>> SEGURE O ROBÔ ou coloque as rodas no ar. <<<")
            try:
                input("Pressione Enter pra continuar (Ctrl+C cancela)...")
            except KeyboardInterrupt:
                print("\nCancelado.")
                ser.close()
                return

        print()
        print(f"Fase 2/3: speed={args.speed}, steer=0 por {args.duration}s")
        if args.front_only:
            frame_go = build_set_speed(0, args.speed, 0, 0)
            print("  (modo --front-only: trás recebe 0)")
        else:
            frame_go = build_set_speed(0, args.speed, 0, args.speed)
        send_loop(ser, frame_go, args.duration, "GO  ", stats, decoder)

        # Aferição: as RPMs subiram?
        recent = list(stats.last_rpms)[-10:]
        if recent:
            avg_FL = sum(r[0] for r in recent) / len(recent)
            avg_FR = sum(r[1] for r in recent) / len(recent)
            avg_RL = sum(r[2] for r in recent) / len(recent)
            avg_RR = sum(r[3] for r in recent) / len(recent)
            print()
            print("RPM médio nas últimas 10 amostras:")
            print(f"  FL={avg_FL:+7.1f}  FR={avg_FR:+7.1f}  "
                  f"RL={avg_RL:+7.1f}  RR={avg_RR:+7.1f}")
            if abs(avg_FL) < 50 and abs(avg_FR) < 50 and front_alive:
                print("  AVISO: placa frente respondia mas RPM não subiu.")
                print("         Pode ser: motor desconectado, sentido invertido,")
                print("         escala muito baixa, ou bateria fraca.")

    # ----- Fase 3: para -----
    print()
    print("Fase 3/3: setpoint=0 por 1s (parando)")
    send_loop(ser, frame_zero, 1.0, "STOP", stats, decoder)

    # Sumário final
    print()
    print("=" * 60)
    print(f"Frames recebidos:  STATE={stats.state_count}  "
          f"IMU={stats.imu_count}  FLOW={stats.flow_count}")
    last = stats.last_state
    if last:
        print(f"Último STATE:      FL={last['rpm_FL']}  FR={last['rpm_FR']}  "
              f"RL={last['rpm_RL']}  RR={last['rpm_RR']}")
        print(f"Baterias:          frente={last['batF']:.2f}V  "
              f"trás={last['batR']:.2f}V")
        print(f"Botão:             {'pressionado' if last['btn'] else 'solto'}")
    print("=" * 60)
    ser.close()


if __name__ == '__main__':
    main()
