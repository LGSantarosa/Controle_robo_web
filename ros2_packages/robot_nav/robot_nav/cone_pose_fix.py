#!/usr/bin/env python3
"""
Regras puras (sem ROS) da correção de pose por cone-âncora no trekking.

Isolar isto do nó ROS permite testar com pytest direto e mantém o
trekking_runner/pose_estimator enxutos. Design:
docs/superpowers/specs/2026-06-01-correcao-pose-cone-trekking-design.md.
"""
import math


def cone_fix_delta(recorded, observed):
    """Deriva medida = cone gravado - cone observado, ambos (x, y) em odom.

    Numa sessão de trekking o frame odom é o mesmo da gravação ao percurso;
    a diferença entre onde o cone foi gravado e onde é visto agora é
    exatamente o quanto a pose derivou.
    """
    return (recorded[0] - observed[0], recorded[1] - observed[1])


def apply_pose_fix(x, y, dx, dy, gain, max_mag):
    """Aplica o delta à pose com ganho parcial; rejeita teleportes.

    Se |(dx, dy)| > max_mag a associação é suspeita (cone errado) e nada muda.
    Retorna (novo_x, novo_y, aceito: bool).
    """
    if math.hypot(dx, dy) > max_mag:
        return x, y, False
    return x + gain * dx, y + gain * dy, True


def cone_bearing(wp_x, wp_y, wp_yaw, cone_x, cone_y):
    """Bearing do cone relativo à pose GRAVADA do waypoint (rad, wrap ±π).

    Reproduz o que _save_point grava, pra que o gate angular do PLAY continue
    coerente após uma troca de cone via set_cone.
    """
    b = math.atan2(cone_y - wp_y, cone_x - wp_x) - wp_yaw
    return math.atan2(math.sin(b), math.cos(b))  # wrap_pi


class ConeFixConfirmer:
    """Gate temporal + unicidade antes de corrigir a pose.

    `update` retorna True na PRIMEIRA chamada em que o mesmo candidato
    (posição estável dentro de `stable_eps`) e ÚNICO (n_candidates <= 1) se
    manteve por `confirm_frames` chamadas seguidas. Cone parado confirma;
    objeto se movendo reseta; ambiguidade (n>1) nunca confirma. O chamador
    deve parar de chamar após o primeiro True e chamar reset() ao trocar de
    waypoint. `count` expõe o progresso pra telemetria.
    """

    def __init__(self, confirm_frames, stable_eps):
        self.confirm_frames = int(confirm_frames)
        self.stable_eps = float(stable_eps)
        self._pos = None
        self._count = 0

    @property
    def count(self):
        return self._count

    def reset(self):
        self._pos = None
        self._count = 0

    def update(self, match_pos, n_candidates):
        if match_pos is None or n_candidates > 1:
            self.reset()
            return False
        if (
            self._pos is not None
            and math.hypot(match_pos[0] - self._pos[0],
                           match_pos[1] - self._pos[1]) < self.stable_eps
        ):
            self._count += 1
        else:
            self._count = 1
        self._pos = match_pos
        return self._count >= self.confirm_frames
