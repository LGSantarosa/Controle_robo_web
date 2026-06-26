#!/usr/bin/env python3
"""Mede o ATRASO de transporte do boneco/scan (BO "boneco atrasado").

Conecta no Socket.IO do controle_web como um cliente Python e mede, por evento,
`agora_cliente - server_ts` — ou seja, quanto o payload demorou do emit (servidor)
até chegar no cliente, ATRAVÉS do transporte real (websocket ou long-polling).
Também imprime QUAL transporte o engineio negociou (é a peça que define se a viz
é "tempo real" ou travada) e a TAXA de chegada.

Uso (no venv do projeto):
    controle_web/.venv/bin/python controle_web/measure_web_lag.py [url] [segundos]
    # default: http://127.0.0.1:5000  por 15 s

Notas:
- scan_update traz `_sts` (server wall-ts do emit); robot_pose traz `ts`.
- Em localhost cliente e servidor compartilham o relógio -> a latência medida é
  transporte+fila puro. Sobre wifi (robô real) entra skew de NTP no valor
  absoluto, mas a TENDÊNCIA (latência crescente = backpressure) continua válida.
"""
import statistics as st
import sys
import time

import socketio


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:5000'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    sio = socketio.Client(logger=False, engineio_logger=False)
    lat = {'scan_update': [], 'robot_pose': []}
    recv_ts = {'scan_update': [], 'robot_pose': []}

    @sio.event
    def connect():
        print(f"conectado: transport={sio.eio.transport()} sid={sio.sid}")

    @sio.on('scan_update')
    def _scan(d):
        if isinstance(d, dict) and '_sts' in d:
            lat['scan_update'].append(time.time() - d['_sts'])
            recv_ts['scan_update'].append(time.time())

    @sio.on('robot_pose')
    def _pose(d):
        if isinstance(d, dict) and 'ts' in d:
            lat['robot_pose'].append(time.time() - d['ts'])
            recv_ts['robot_pose'].append(time.time())

    try:
        sio.connect(url, wait_timeout=5)
    except Exception as e:
        print(f"FALHA ao conectar em {url}: {e}")
        sys.exit(1)
    time.sleep(dur)
    # após o sleep, reimprime o transporte (pode ter feito upgrade polling->ws)
    print(f"transport final: {sio.eio.transport()}")
    sio.disconnect()

    for ev in ('scan_update', 'robot_pose'):
        v = lat[ev]
        if not v:
            print(f"{ev}: SEM eventos"); continue
        ts = recv_ts[ev]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        v_ms = sorted(x * 1000 for x in v)
        p90 = v_ms[int(0.9 * len(v_ms))]
        drift = (v[-1] - v[0]) * 1000  # latência cresceu? (backpressure)
        print(f"{ev}: n={len(v)} taxa~{rate:.1f}Hz  lat_ms "
              f"min={v_ms[0]:.0f} med={st.median(v_ms):.0f} p90={p90:.0f} "
              f"max={v_ms[-1]:.0f}  drift={drift:+.0f}ms")


if __name__ == '__main__':
    main()
