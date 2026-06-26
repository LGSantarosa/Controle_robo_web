"""Teste do _clear_pre_door_point: busca 2D que escapa de parede LATERAL.

MapBridge precisa de ROS/socketio pra instanciar, então usamos um stub leve que
só empresta os 2 métodos + a constante (eles só dependem de _grid/_grid_meta)."""
import numpy as np

from map_service import MapBridge


class _Stub:
    PRE_DOOR_CLEARANCE = MapBridge.PRE_DOOR_CLEARANCE
    _point_clear = MapBridge._point_clear
    _point_clearance = MapBridge._point_clearance
    _clear_pre_door_point = MapBridge._clear_pre_door_point


def _stub_with_wall_above(y_wall=0.3):
    """Grade 3x2 m (res 0.05) com PAREDE em y>=y_wall (lado +y). Lado -y aberto."""
    res, ox, oy, w, h = 0.05, -1.0, -1.0, 60, 40
    grid = np.zeros((h, w), dtype=np.int8)
    for row in range(h):
        if oy + row * res >= y_wall:
            grid[row, :] = 100
    s = _Stub()
    s._grid = grid
    s._grid_meta = (res, ox, oy, w, h)
    return s


# porta no eixo y (x=0), vão y∈[-0.46,0.46], centro (0,0); robô no lado +x.
DOOR = {'a': [0.0, 0.46], 'b': [0.0, -0.46]}


def test_ponto_ideal_livre_fica_igual():
    s = _stub_with_wall_above(0.3)
    # ideal bem longe da parede (-y) -> já livre, não mexe
    out = s._clear_pre_door_point(DOOR, 1.0, -0.6)
    assert out == (1.0, -0.6)


def test_colado_na_parede_lateral_escapa_pro_lado_aberto():
    s = _stub_with_wall_above(0.3)
    # ideal (1.0, 0.0): a 0.3 da parede em y=0.3 -> NÃO livre a 0.50
    assert not s._point_clear(1.0, 0.0, _Stub.PRE_DOOR_CLEARANCE)
    nx, ny = s._clear_pre_door_point(DOOR, 1.0, 0.0)
    # achou ponto livre, do lado aberto (-y) e do lado do robô (+x)
    assert (nx, ny) != (1.0, 0.0)
    assert ny < 0.0
    assert nx > 0.1
    assert s._point_clear(nx, ny, _Stub.PRE_DOOR_CLEARANCE)


def test_nao_cruza_pra_dentro_da_porta():
    s = _stub_with_wall_above(0.3)
    # o ponto escapado nunca pula pro lado -x (atravessar a porta)
    nx, ny = s._clear_pre_door_point(DOOR, 1.0, 0.0)
    assert nx > 0.0   # continua do lado do robô (+x)


def _stub_corridor(half=0.3):
    """Corredor estreito: paredes em y>=half E y<=-half. Nenhum ponto atinge
    0.50 de folga -> testa o fallback 'mais livre'."""
    res, ox, oy, w, h = 0.05, -1.0, -1.0, 60, 40
    grid = np.zeros((h, w), dtype=np.int8)
    for row in range(h):
        y = oy + row * res
        if y >= half or y <= -half:
            grid[row, :] = 100
    s = _Stub()
    s._grid = grid
    s._grid_meta = (res, ox, oy, w, h)
    return s


def test_sem_folga_ideal_cai_no_mais_livre_nao_no_original():
    s = _stub_corridor(0.3)   # corredor 0.6m: max folga ~0.3 < 0.50
    ideal = (1.0, 0.2)        # colado na parede de cima (folga ~0.1)
    assert not s._point_clear(*ideal, _Stub.PRE_DOOR_CLEARANCE)
    out = s._clear_pre_door_point(DOOR, *ideal)
    # NÃO volta pro original colado; vai pro mais livre (centro do corredor)
    assert out != ideal
    assert s._point_clearance(*out, 0.9) > s._point_clearance(*ideal, 0.9)
