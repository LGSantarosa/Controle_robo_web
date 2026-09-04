#!/usr/bin/env python3
"""Mostra qual botão/eixo do joystick é qual número, lendo /dev/input/jsN direto.

Por que existe: os números de botão mudam entre controles e entre drivers — o
mesmo LB pode ser 4 (driver xpad) ou 6 (HID genérico por Bluetooth). O
teleop_twist_joy usa esses números crus (`enable_button`), então chutar errado
significa dead-man que não solta ou robô que não anda. Aqui a gente mede.

Sem dependências: lê o protocolo do joydev na mão (8 bytes por evento).

Uso:
    ./scripts/js_mapping.py                 # usa /dev/input/js0
    ./scripts/js_mapping.py /dev/input/js1
    ./scripts/js_mapping.py --quiet         # só botões, ignora ruído de eixo

Ctrl+C encerra. O que interessa pro projeto:
    - qual número dá o LB   → enable_button      (dead-man)
    - qual número dá o RB   → enable_turbo_button
    - qual eixo é o analógico esquerdo Y (frente/ré) e X (giro)
"""

import struct
import sys

# struct do joydev (linux/joystick.h): __u32 time, __s16 value, __u8 type, __u8 number
EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# Abaixo disso, movimento de eixo é ruído/drift do analógico parado.
AXIS_NOISE = 0.15


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    quiet = "--quiet" in sys.argv or "-q" in sys.argv
    device = args[0] if args else "/dev/input/js0"

    try:
        fh = open(device, "rb")
    except FileNotFoundError:
        print(f"ERRO: {device} não existe. Joysticks presentes:")
        import glob

        found = sorted(glob.glob("/dev/input/js*"))
        print("  " + (", ".join(found) if found else "(nenhum — controle conectado?)"))
        return 1
    except PermissionError:
        print(f"ERRO: sem permissão em {device}. Tente com sudo, ou adicione o")
        print("      usuário ao grupo 'input': sudo usermod -aG input $USER")
        return 1

    print(f"Lendo {device} — aperte os botões (Ctrl+C encerra).")
    print("Dica: aperte LB e depois RB; são esses dois números que vão no YAML.\n")

    with fh:
        while True:
            data = fh.read(EVENT_SIZE)
            if not data or len(data) < EVENT_SIZE:
                print("\n(stream do joystick acabou — controle desconectou?)")
                return 1

            _, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)

            # Eventos de sincronia inicial: o driver despeja o estado de tudo
            # ao abrir o device. Não é o usuário apertando nada.
            if ev_type & JS_EVENT_INIT:
                continue

            if ev_type & JS_EVENT_BUTTON:
                estado = "PRESSIONADO" if value else "solto"
                print(f"botão {number:>2}  {estado}")
            elif ev_type & JS_EVENT_AXIS:
                norm = value / 32767.0
                if quiet or abs(norm) < AXIS_NOISE:
                    continue
                print(f"eixo  {number:>2}  {norm:+.2f}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
