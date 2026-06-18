"""Geometria de porta pra a web (sem dependência de ROS, testável isolado).

Espelha robot_nav.door_crossing: usado pra pôr o ponto-PRÉ-PORTA na rota quando
o destino fica do outro lado de uma porta marcada (2026-06-18). Duplicado de
propósito p/ a web (Flask) não depender do pacote ROS robot_nav.
"""
import math

DOOR_STANDOFF = 1.0   # m — distância do ponto-pré-porta antes do centro da porta


def _seg_cross(p1, p2, p3, p4) -> bool:
    """True se os segmentos p1-p2 e p3-p4 se cruzam de verdade."""
    def ccw(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = ccw(p3, p4, p1), ccw(p3, p4, p2)
    d3, d4 = ccw(p1, p2, p3), ccw(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def pre_door_waypoint(a, b, robot_xy, standoff=DOOR_STANDOFF):
    """Ponto-pré-porta (x, y, yaw): no eixo da porta, recuado `standoff` do centro
    no lado onde o robô está, de frente pra porta. `a`,`b` = os 2 batentes."""
    ax, ay = a
    bx, by = b
    cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
    w = math.hypot(bx - ax, by - ay)
    tx, ty = (bx - ax) / w, (by - ay) / w       # ao longo da parede
    nx, ny = -ty, tx                            # normal (atravessa o vão)
    rx, ry = robot_xy
    side = -1 if ((rx - cx) * nx + (ry - cy) * ny) > 0 else +1
    wx = cx - nx * side * standoff
    wy = cy - ny * side * standoff
    return wx, wy, math.atan2(side * ny, side * nx)


def door_on_segment(robot_xy, goal_xy, doors):
    """A porta marcada que o trajeto RETO robô->destino cruza (a 1ª que cruzar),
    ou None. Heurística simples p/ "preciso passar por esta porta". `doors` =
    lista de {'a':[x,y],'b':[x,y],...}."""
    for d in doors:
        if _seg_cross(robot_xy, goal_xy, tuple(d['a']), tuple(d['b'])):
            return d
    return None
