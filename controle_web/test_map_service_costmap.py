import base64
from io import BytesIO

import numpy as np
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap

from map_service import (
    _costmap_msg_to_occupancy_grid,
    _costmap_to_png_rgba_b64,
    _grid_info,
)


def _grid(values, w, h, res=0.05, ox=0.0, oy=0.0):
    g = OccupancyGrid()
    g.info.width = w
    g.info.height = h
    g.info.resolution = res
    g.info.origin.position.x = ox
    g.info.origin.position.y = oy
    g.info.origin.orientation.w = 1.0
    g.data = values
    return g


def _decode_rgba(b64, w, h):
    img = Image.open(BytesIO(base64.b64decode(b64)))
    assert img.mode == 'RGBA'
    return np.array(img)  # (h, w, 4)


def test_free_and_unknown_are_transparent():
    # 0 = livre, -1 = desconhecido -> alpha 0 (não tampam o mapa)
    g = _grid([0, -1, 0, -1], 2, 2)
    rgba = _decode_rgba(_costmap_to_png_rgba_b64(g), 2, 2)
    assert (rgba[:, :, 3] == 0).all()


def test_lethal_is_opaque_magenta():
    # valor 100 (letal) = obstáculo -> magenta bem opaco
    g = _grid([100, 100, 100, 100], 2, 2)
    rgba = _decode_rgba(_costmap_to_png_rgba_b64(g), 2, 2)
    assert (rgba[:, :, 3] >= 200).all()
    # magenta: R alto, G baixo, B alto
    assert rgba[0, 0, 0] > 200 and rgba[0, 0, 1] < 50 and rgba[0, 0, 2] > 200


def test_inflation_is_translucent_gradient():
    # inflação (1..98) -> translúcido (alpha intermediário), nem 0 nem opaco
    g = _grid([1, 50, 98, 30], 2, 2)
    rgba = _decode_rgba(_costmap_to_png_rgba_b64(g), 2, 2)
    a = rgba[:, :, 3].ravel()
    assert (a > 0).all() and (a < 200).all()


def test_flipud_orientation():
    # linha de baixo do grid (y menor) vira linha de baixo do PNG.
    # grid row0 (data 0..1) = letal; row1 = livre. PNG é flipud -> letal embaixo.
    g = _grid([100, 100, 0, 0], 2, 2)
    rgba = _decode_rgba(_costmap_to_png_rgba_b64(g), 2, 2)
    assert (rgba[-1, :, 3] >= 200).all()   # base do PNG = letal
    assert (rgba[0, :, 3] == 0).all()      # topo do PNG = livre


def test_grid_info_origin_and_resolution():
    g = _grid([0] * 4, 2, 2, res=0.05, ox=-1.5, oy=2.0)
    info = _grid_info(g)
    assert info['width'] == 2 and info['height'] == 2
    assert abs(info['resolution'] - 0.05) < 1e-9
    assert abs(info['origin_x'] - (-1.5)) < 1e-9
    assert abs(info['origin_y'] - 2.0) < 1e-9
    assert abs(info['origin_yaw']) < 1e-9


def _costmap_msg(values, w, h, res=0.05, ox=0.0, oy=0.0):
    c = Costmap()
    c.metadata.size_x = w
    c.metadata.size_y = h
    c.metadata.resolution = res
    c.metadata.origin.position.x = ox
    c.metadata.origin.position.y = oy
    c.metadata.origin.orientation.w = 1.0
    c.data = bytes(values)
    return c


def test_costmap_msg_maps_raw_costs_to_occupancy_scale():
    # custo cru do costmap_2d (0..255) -> escala OccupancyGrid (-1/0..100)
    # 0 livre, 255 desconhecido, 254 letal, 253 inscrito, 1..252 inflação(1..98)
    c = _costmap_msg([0, 255, 254, 253], 2, 2)
    g = _costmap_msg_to_occupancy_grid(c)
    assert list(g.data) == [0, -1, 100, 99]


def test_costmap_msg_inflation_gradient_in_range():
    # extremos da inflação caem em 1..98 (o que a conversão PNG espera)
    c = _costmap_msg([1, 252], 2, 1)
    g = _costmap_msg_to_occupancy_grid(c)
    assert g.data[0] == 1 and g.data[1] == 98


def test_costmap_msg_preserves_metadata():
    c = _costmap_msg([0] * 6, 3, 2, res=0.1, ox=-2.0, oy=1.0)
    g = _costmap_msg_to_occupancy_grid(c)
    assert g.info.width == 3 and g.info.height == 2
    assert abs(g.info.resolution - 0.1) < 1e-9
    assert abs(g.info.origin.position.x - (-2.0)) < 1e-9
    assert abs(g.info.origin.position.y - 1.0) < 1e-9
    # encadeia com _grid_info/_costmap_to_png_rgba_b64 sem erro
    info = _grid_info(g)
    assert info['width'] == 3 and info['height'] == 2
