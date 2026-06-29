#!/usr/bin/env python3
"""
world2map.py — gera um mapa de ocupação (.pgm + .yaml) a partir de um mundo .sdf.

Pra QUÊ: no sim a geometria é conhecida (paredes/obstáculos no SDF), então dá pra
rasterizar um mapa PERFEITO alinhado ao frame do mundo (origin = canto do mundo) — o
robô localiza "de cara" sem precisar SLAMar. É o mapa que o `--sim --nav2` consome.

Suporta modelos ESTÁTICOS com collision box ou cylinder, SEM rotação (yaw=0). Cada
modelo vira pixels OCUPADOS; o resto é LIVRE; sem células "desconhecidas" (mapa fechado).

Uso:
    python3 tools/world2map.py worlds/sala_grande.sdf maps/sala_grande --res 0.05

Gera maps/sala_grande.pgm e maps/sala_grande.yaml.
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

# Valores PGM no padrão do nav2/map_server (occupied_thresh 0.65, free_thresh 0.25)
FREE = 254      # branco
OCCUPIED = 0    # preto


def _floats(text: str) -> list[float]:
    return [float(t) for t in text.split()]


def parse_models(sdf_path: str):
    """Retorna lista de obstáculos: ('box', cx, cy, sx, sy) ou ('cyl', cx, cy, r)."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise SystemExit("SDF sem <world>")

    obstacles = []
    for model in world.findall("model"):
        name = model.get("name", "")
        # ignora o chão (plane) — não é obstáculo
        if name == "ground_plane":
            continue
        link = model.find("link")
        if link is None:
            continue
        # pose do modelo (x y z roll pitch yaw); fallback (0,0,0,...)
        pose_el = model.find("pose")
        pose = _floats(pose_el.text) if pose_el is not None and pose_el.text else [0, 0, 0, 0, 0, 0]
        cx, cy = pose[0], pose[1]
        yaw = pose[5] if len(pose) >= 6 else 0.0
        if abs(yaw) > 1e-3:
            print(f"[AVISO] {name}: yaw={yaw:.3f} != 0 — rasterizado como AABB (sem rotacao).",
                  file=sys.stderr)

        for coll in link.findall("collision"):
            geom = coll.find("geometry")
            if geom is None:
                continue
            box = geom.find("box")
            cyl = geom.find("cylinder")
            if box is not None:
                sx, sy, _sz = _floats(box.find("size").text)
                obstacles.append(("box", cx, cy, sx, sy))
            elif cyl is not None:
                r = float(cyl.find("radius").text)
                obstacles.append(("cyl", cx, cy, r))
    return obstacles


def build_grid(obstacles, res: float, pad: float):
    # bounding box do mundo a partir dos obstaculos
    xs_min, xs_max, ys_min, ys_max = [], [], [], []
    for o in obstacles:
        if o[0] == "box":
            _, cx, cy, sx, sy = o
            xs_min.append(cx - sx / 2); xs_max.append(cx + sx / 2)
            ys_min.append(cy - sy / 2); ys_max.append(cy + sy / 2)
        else:
            _, cx, cy, r = o
            xs_min.append(cx - r); xs_max.append(cx + r)
            ys_min.append(cy - r); ys_max.append(cy + r)
    x_min = min(xs_min) - pad
    x_max = max(xs_max) + pad
    y_min = min(ys_min) - pad
    y_max = max(ys_max) + pad

    width = int(math.ceil((x_max - x_min) / res))
    height = int(math.ceil((y_max - y_min) / res))
    grid = np.full((height, width), FREE, dtype=np.uint8)

    def w2c(wx, wy):
        col = int(round((wx - x_min) / res))
        row_bottom = int(round((wy - y_min) / res))
        row = (height - 1) - row_bottom  # PGM: linha 0 = topo
        return row, col

    for o in obstacles:
        if o[0] == "box":
            _, cx, cy, sx, sy = o
            r0, c0 = w2c(cx - sx / 2, cy + sy / 2)  # canto sup-esq
            r1, c1 = w2c(cx + sx / 2, cy - sy / 2)  # canto inf-dir
            rmin, rmax = sorted((r0, r1))
            cmin, cmax = sorted((c0, c1))
            grid[max(0, rmin):min(height, rmax + 1),
                 max(0, cmin):min(width, cmax + 1)] = OCCUPIED
        else:
            _, cx, cy, rad = o
            rc, cc = w2c(cx, cy)
            rr = int(math.ceil(rad / res))
            for dr in range(-rr, rr + 1):
                for dc in range(-rr, rr + 1):
                    if (dr * res) ** 2 + (dc * res) ** 2 <= rad ** 2:
                        rr_i, cc_i = rc + dr, cc + dc
                        if 0 <= rr_i < height and 0 <= cc_i < width:
                            grid[rr_i, cc_i] = OCCUPIED

    return grid, x_min, y_min, width, height


def write_pgm(path: str, grid: np.ndarray):
    h, w = grid.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(grid.tobytes())


def write_yaml(path: str, pgm_name: str, res: float, x_min: float, y_min: float):
    with open(path, "w") as f:
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{x_min:.4f}, {y_min:.4f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("out_base", help="caminho base sem extensao, ex. maps/sala_grande")
    ap.add_argument("--res", type=float, default=0.05, help="m/pixel (default 0.05)")
    ap.add_argument("--pad", type=float, default=0.10, help="margem em metros ao redor (default 0.10)")
    args = ap.parse_args()

    obstacles = parse_models(args.sdf)
    if not obstacles:
        raise SystemExit("nenhum obstaculo encontrado no SDF")
    grid, x_min, y_min, w, h = build_grid(obstacles, args.res, args.pad)

    pgm_path = args.out_base + ".pgm"
    yaml_path = args.out_base + ".yaml"
    pgm_name = pgm_path.split("/")[-1]
    write_pgm(pgm_path, grid)
    write_yaml(yaml_path, pgm_name, args.res, x_min, y_min)

    occ = int((grid == OCCUPIED).sum())
    print(f"OK: {w}x{h} px @ {args.res} m/px | origin=({x_min:.2f},{y_min:.2f}) | "
          f"{occ} px ocupados ({100*occ/(w*h):.1f}%)")
    print(f"  -> {pgm_path}")
    print(f"  -> {yaml_path}")


if __name__ == "__main__":
    main()
