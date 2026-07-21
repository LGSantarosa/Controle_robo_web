"""Lê o /tmp/motion_guard_face.json (motion_guard, fase 2 da cara) e vira
estado do olhar. SEM flask e SEM ROS de propósito: testável no pytest do
sistema e reaproveitável (o futuro MODO INTERAÇÃO lê o mesmo arquivo)."""
import json
import os

STALE_S = 1.5      # arquivo mais velho que isso = stack caída, sem pessoa
BEHIND_DEG = 100.0  # pessoa atrás da tela: ninguém vê a cara, ignora
FULL_DEG = 90.0     # pedido do dono 07-17: pessoa a 90° = olho colado na
                    # lateral; o meio do caminho quem engorda é o 1.6x do
                    # face.js (o crave a 45° de antes ficou brusco)


def read_state(path, now, sign=1.0):
    try:
        if now - os.stat(path).st_mtime > STALE_S:
            return {'person': False, 'blocked': False}
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {'person': False, 'blocked': False}
    # blocked = guard segurando o robô POR CAUSA da pessoa com rota ativa
    # (motion_guard só grava state com cmd fresco) -> cara pede licença.
    # SEMPRE reportado, mesmo sem alvo de olhar: pessoa PARADA some do
    # detector de movimento (cbear null) mas o guard segue 'blocked' por ela
    # -> a cara tem que CONTINUAR pedindo licença enquanto está travado.
    blocked = data.get('state') == 'blocked'
    cbear = data.get('cbear_deg')
    if cbear is None or abs(cbear) > BEHIND_DEG:
        return {'person': False, 'blocked': blocked}
    x = max(-1.0, min(1.0, cbear / FULL_DEG))
    return {'person': True, 'x': round(sign * x, 3), 'blocked': blocked}
