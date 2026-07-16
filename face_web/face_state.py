"""Lê o /tmp/motion_guard_face.json (motion_guard, fase 2 da cara) e vira
estado do olhar. SEM flask e SEM ROS de propósito: testável no pytest do
sistema e reaproveitável (o futuro MODO INTERAÇÃO lê o mesmo arquivo)."""
import json
import os

STALE_S = 1.5     # arquivo mais velho que isso = stack caída, sem pessoa
MAX_DEG = 90.0    # pessoa atrás da tela: ninguém vê a cara, ignora


def read_state(path, now, sign=1.0):
    try:
        if now - os.stat(path).st_mtime > STALE_S:
            return {'person': False}
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {'person': False}
    cbear = data.get('cbear_deg')
    if cbear is None or abs(cbear) > MAX_DEG:
        return {'person': False}
    return {'person': True, 'x': round(sign * cbear / MAX_DEG, 3)}
