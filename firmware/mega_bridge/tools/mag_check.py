#!/usr/bin/env python3
"""
mag_check.py — calibra e valida o magnetômetro AK8963 (Fase 1 do yaw absoluto).

Lê o /dev/mega cru (mesma serial da MEGA, 230400) e decodifica os frames:
  FT_MAG (0x84): mx,my,mz int16 DECI-µT (÷10 = µT), frame do gyro, ASA aplicado.
  FT_IMU (0x82): gx,gy,gz,ax,ay,az int16 MILLI (÷1000), p/ comparar o heading do
                 mag com o yaw INTEGRADO do gyro (critério 2 da validação).

NÃO toca em nada da navegação — é só leitura crua p/ eu (assistente) avaliar.

Uso:
  # 1) CALIBRAR: gira o robô ~360° devagar durante a coleta
  python3 mag_check.py collect 20

  # 2) VALIDAR: parado / girando 90° / com MOTORES ligados
  python3 mag_check.py check 15

Gera/usa mag_calib.json e mag_raw.csv no diretório atual.
"""
import sys, time, json, struct, math

import serial

PORT = "/dev/mega"
BAUD = 230400
CALIB_FILE = "mag_calib.json"


def _stream(seconds, want_gyro=False):
    """Itera (t, kind, payload) decodificando AA55 frames por `seconds`."""
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(1.5)            # abrir reseta a MEGA (DTR) + recalibra bias; assenta
    s.reset_input_buffer()
    buf = bytearray(); t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            buf += s.read(256); now = time.time()
            while len(buf) >= 5:
                i = buf.find(b"\xaa\x55")
                if i < 0 or len(buf) < i + 5:
                    break
                ft = buf[i + 2]; ln = buf[i + 3]
                if len(buf) < i + 4 + ln + 1:
                    break
                pl = bytes(buf[i + 4:i + 4 + ln])
                if ft == 0x84 and ln == 6:
                    mx, my, mz = struct.unpack("<3h", pl)
                    yield (now - t0, "mag", (mx / 10.0, my / 10.0, mz / 10.0))
                elif want_gyro and ft == 0x82 and ln == 12:
                    g = struct.unpack("<6h", pl)
                    yield (now - t0, "gyro", (g[2] / 1000.0,))  # só gz (rad/s)
                del buf[:i + 4 + ln + 1]
    finally:
        s.close()


def collect(seconds):
    xs, ys, zs = [], [], []
    rows = []
    for t, kind, v in _stream(seconds):
        if kind == "mag":
            mx, my, mz = v
            xs.append(mx); ys.append(my); zs.append(mz)
            rows.append((t, mx, my, mz))
    if not xs:
        print("NENHUM frame FT_MAG (0x84). O AK8963 inicializou? (magOk no firmware)")
        return
    with open("mag_raw.csv", "w") as f:
        f.write("t,mx,my,mz\n")
        for r in rows:
            f.write("%.3f,%.2f,%.2f,%.2f\n" % r)

    def stats(a):
        return min(a), max(a), (min(a) + max(a)) / 2, (max(a) - min(a)) / 2
    sx, sy, sz = stats(xs), stats(ys), stats(zs)
    off = [sx[2], sy[2], sz[2]]                 # hard-iron: centro
    rad = [sx[3], sy[3], sz[3]]                 # raio por eixo
    avg = sum(rad) / 3.0
    scale = [avg / r if r > 1e-6 else 1.0 for r in rad]  # soft-iron simples
    json.dump({"offset": off, "scale": scale}, open(CALIB_FILE, "w"), indent=2)

    print("=== COLETA: %d amostras de mag (%.1f Hz) ===" % (len(xs), len(xs) / seconds))
    print("eixo  min     max     offset  raio")
    for nm, st in (("mx", sx), ("my", sy), ("mz", sz)):
        print("%s  %7.1f %7.1f %7.1f %7.1f" % (nm, st[0], st[1], st[2], st[3]))
    print("hard-iron offset =", [round(o, 1) for o in off])
    print("soft-iron scale  =", [round(s, 3) for s in scale])
    print("raio medio = %.1f µT (Earth ~25-65 µT esperado)" % avg)
    print("-> salvo %s + mag_raw.csv. Agora rode: mag_check.py check 15" % CALIB_FILE)


def check(seconds):
    try:
        cal = json.load(open(CALIB_FILE))
    except FileNotFoundError:
        print("sem %s — rode 'collect' primeiro (gira 360°)." % CALIB_FILE)
        return
    off, scale = cal["offset"], cal["scale"]

    def heading(mx, my):
        cx = (mx - off[0]) * scale[0]
        cy = (my - off[1]) * scale[1]
        return math.degrees(math.atan2(cy, cx))

    yaw_gyro = 0.0; tprev = None
    win_h = []; tw = 0.0; last_h = None
    print(" t(s)  mag_head(°)  gyro_yaw(°)  (devem mover JUNTOS num giro)")
    for t, kind, v in _stream(seconds, want_gyro=True):
        if kind == "gyro":
            if tprev is not None:
                yaw_gyro += v[0] * (t - tprev)
            tprev = t
        elif kind == "mag":
            last_h = heading(v[0], v[1]); win_h.append(last_h)
        if t - tw >= 0.5 and last_h is not None:
            # variação do heading na janela (estabilidade)
            sp = (max(win_h) - min(win_h)) if win_h else 0.0
            print("%4.1f   %7.1f      %7.1f      (var_mag=%.1f°)"
                  % (t, last_h, math.degrees(yaw_gyro), sp))
            win_h = []; tw = t
    print("FIM (parado: var_mag pequena = bom; girando: mag_head e gyro_yaw juntos)")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("collect", "check"):
        print(__doc__); return
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    (collect if sys.argv[1] == "collect" else check)(seconds)


if __name__ == "__main__":
    main()
