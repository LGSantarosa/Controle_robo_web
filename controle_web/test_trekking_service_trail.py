"""Round-trip da trilha densa pelo disco (save_route -> load_route).

A trilha e o UNICO dado da rota que nao vem do /trekking/state: ela chega por
um topico latched separado (seria absurdo mandar milhares de pontos a 10 Hz no
JSON de estado). Entao o caminho dela ate o disco e proprio — e e o que estes
testes cobrem. Rota antiga, gravada antes da trilha existir, tem que continuar
carregando.
"""
import json
import os
import tempfile

import pytest

from trekking_service import TrekkingBridge


def _bridge(tmpdir):
    """Instancia SEM ROS: save_route/load_route so tocam disco e o cmd pub."""
    b = TrekkingBridge.__new__(TrekkingBridge)
    b._routes_dir = tmpdir
    b._last_state = None
    b._last_trail = []
    import threading
    b._state_lock = threading.Lock()
    b._enviados = []
    b.send_cmd = lambda cmd, **kw: b._enviados.append((cmd, kw)) or {'ok': True}
    return b


WPS = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'has_cone': False,
        'cone_x': 0.0, 'cone_y': 0.0, 'cone_bearing': 0.0, 'trail_i': 3}]
TRAIL = [{'x': 0.0, 'y': 0.0, 'yaw': 0.0},
         {'x': 0.1, 'y': 0.0, 'yaw': 0.0},
         {'x': 0.2, 'y': 0.05, 'yaw': 0.2},
         {'x': 0.3, 'y': 0.1, 'yaw': 0.3}]


def test_salva_a_trilha_junto_dos_waypoints():
    with tempfile.TemporaryDirectory() as d:
        b = _bridge(d)
        b._last_trail = list(TRAIL)
        r = b.save_route('rota_teste', waypoints=WPS)
        assert r['ok'] and r['trail_n'] == 4
        salvo = json.load(open(os.path.join(d, 'rota_teste.json')))
        assert salvo['trail'] == TRAIL
        assert salvo['waypoints'][0]['trail_i'] == 3


def test_carrega_manda_a_trilha_pro_runner():
    with tempfile.TemporaryDirectory() as d:
        b = _bridge(d)
        b._last_trail = list(TRAIL)
        b.save_route('rota_teste', waypoints=WPS)
        b2 = _bridge(d)
        r = b2.load_route('rota_teste')
        assert r['ok'] and len(r['trail']) == 4
        cmd, kw = b2._enviados[-1]
        assert cmd == 'load_waypoints'
        assert kw['trail'] == TRAIL          # o runner precisa dela pro PLAY


def test_rota_ANTIGA_sem_trilha_continua_carregando():
    """Nao quebrar as rotas ja gravadas (2pontos/4pontos/rota1...)."""
    with tempfile.TemporaryDirectory() as d:
        antiga = {'name': 'velha', 'waypoints': WPS, 'saved_ts': 0}
        json.dump(antiga, open(os.path.join(d, 'velha.json'), 'w'))
        b = _bridge(d)
        r = b.load_route('velha')
        assert r['ok'] and r['count'] == 1 and r['trail'] == []


def test_salvar_sem_trilha_nao_explode():
    """Gravou os pontos com o topico da trilha ainda vazio -> salva do mesmo jeito."""
    with tempfile.TemporaryDirectory() as d:
        b = _bridge(d)
        r = b.save_route('so_pontos', waypoints=WPS)
        assert r['ok'] and r['trail_n'] == 0
        assert json.load(open(os.path.join(d, 'so_pontos.json')))['trail'] == []
