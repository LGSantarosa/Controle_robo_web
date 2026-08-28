#!/usr/bin/env python3
"""Confere se um mapa NOVO aguenta o robô — ANTES de rodar o nav2 nele.

Por que existe (2026-08-27): o `nav2_trekking` passou a declarar
`robot_radius: 0.32` no costmap (antes era o footprint quadrado, cujo raio
INSCRITO é 0.25). Isso é o fix que acabou com o robô raspar parede — mas tem um
efeito colateral duro: **o nav2 trata tudo abaixo do raio como BLOQUEIO**, então
a fresta mínima que o robô atravessa subiu de ~0.50 m para **~0.64 m**.

Num mapa apertado isso aparece como o robô simplesmente não achar caminho:
    GridBased plugin failed to plan: "Could not generate path between the given poses"
e o BT afunda em recovery. Foi exatamente o que aconteceu quando testei o raio
circunscrito exato (0.354): dois vãos de 0.70 m do sala_grande fecharam.

Este script responde em segundos, SÓ olhando a imagem do mapa (não sobe nada,
não toca no robô): o mapa continua inteiro com o robô deste tamanho?

Uso:
    python3 tools/mapa_passagens.py maps/meu_mapa_novo.yaml
    python3 tools/mapa_passagens.py maps/meu_mapa_novo.yaml --raio 0.32

Leitura do resultado:
  - "mapa INTEIRO" = nenhuma passagem fechou, pode rodar.
  - queda grande no "maior trecho" entre 0.25 e o raio novo = alguma passagem
    fechou; o relatório aponta as coordenadas das mais estreitas pra você ir
    olhar (ou alargar o mapa, ou baixar o raio).
"""
import argparse
import os
import sys

import numpy as np
from scipy import ndimage


def le_pgm(caminho):
    """Lê PGM binário (P5), pulando comentários do cabeçalho."""
    with open(caminho, 'rb') as f:
        dados = f.read()
    if dados[:2] != b'P5':
        raise ValueError(f'{caminho}: só PGM binário (P5) — este é {dados[:2]!r}')
    campos, i = [], 2
    while len(campos) < 3:
        while dados[i:i + 1].isspace():
            i += 1
        if dados[i:i + 1] == b'#':
            while dados[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not dados[j:j + 1].isspace():
            j += 1
        campos.append(int(dados[i:j]))
        i = j
    w, h, _maxval = campos
    i += 1                                    # 1 whitespace após maxval
    return np.frombuffer(dados[i:i + w * h], dtype=np.uint8).reshape(h, w)


def le_yaml(caminho):
    """resolution + image do .yaml do map_server (sem depender de pyyaml)."""
    res, img = 0.05, None
    for linha in open(caminho):
        linha = linha.split('#')[0].strip()
        if linha.startswith('resolution:'):
            res = float(linha.split(':', 1)[1])
        elif linha.startswith('image:'):
            img = linha.split(':', 1)[1].strip()
    if img is None:
        raise ValueError(f'{caminho}: sem campo "image:"')
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(caminho)), img)
    return res, img


def analisa(yaml_path, raios):
    res, img = le_yaml(yaml_path)
    a = le_pgm(img)
    ocupado = a < 100                          # preto = parede
    livre = a > 250                            # branco = livre (cinza = desconhecido)
    # distância de cada célula até a parede mais próxima, em METROS
    dist = ndimage.distance_transform_edt(~ocupado) * res

    print(f'mapa: {os.path.basename(yaml_path)}   {a.shape[1]}x{a.shape[0]} células '
          f'@ {res} m   ({100 * livre.mean():.0f}% livre)')
    print()
    print(f'  {"raio":>7}  {"fresta mín":>10}  {"trechos":>8}  {"maior trecho":>13}')
    base = None
    for r in raios:
        nav = livre & (dist >= r)
        lab, n = ndimage.label(nav)
        if n == 0:
            print(f'  {r:7.3f}  {2 * r:9.2f}m  {"-":>8}  NADA NAVEGÁVEL')
            continue
        tam = ndimage.sum(nav, lab, range(1, n + 1))
        frac = tam.max() / tam.sum()
        if base is None:
            base = frac
        alerta = ''
        if frac < base - 0.02:
            alerta = f'  ⚠️ PERDEU {100 * (base - frac):.0f}% vs raio {raios[0]}'
        print(f'  {r:7.3f}  {2 * r:9.2f}m  {n:8d}  {100 * frac:11.1f}%{alerta}')
    print()
    print('  "fresta mín" = corredor mais estreito que o robô ainda atravessa (2·raio).')
    print('  "trechos" alto é normal (ruído/salpico do SLAM); o que importa é o')
    print('  MAIOR TRECHO não desabar quando o raio sobe — isso é passagem fechando.')
    return dist, livre, res


def gargalos(dist, livre, res, raio, quantos=8):
    """Aponta os pontos de passagem mais apertados que o robô ainda usa."""
    nav = livre & (dist >= raio)
    lab, n = ndimage.label(nav)
    if n == 0:
        return
    tam = ndimage.sum(nav, lab, range(1, n + 1))
    maior = int(np.argmax(tam)) + 1
    # células do trecho principal onde a folga é mínima = os gargalos
    principal = (lab == maior)
    d = np.where(principal, dist, np.inf)
    print(f'  gargalos do trecho principal (raio {raio}), em célula (linha,coluna):')
    plano = d.ravel()
    ordem = np.argsort(plano)[:quantos * 400]
    vistos = []
    for idx in ordem:
        y, x = divmod(int(idx), d.shape[1])
        if any(abs(y - vy) < 20 and abs(x - vx) < 20 for vy, vx, _ in vistos):
            continue
        vistos.append((y, x, plano[idx]))
        if len(vistos) >= quantos:
            break
    for y, x, folga in vistos:
        print(f'     ({y:4d},{x:4d})  folga {folga:.2f} m  '
              f'-> corredor de ~{2 * folga:.2f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mapa', help='caminho do .yaml do mapa')
    ap.add_argument('--raio', type=float, default=0.32,
                    help='robot_radius do costmap (default 0.32, o do perfil ARENA)')
    args = ap.parse_args()
    if not os.path.exists(args.mapa):
        sys.exit(f'não achei {args.mapa}')
    raios = sorted({0.25, args.raio, 0.354})
    dist, livre, res = analisa(args.mapa, raios)
    print()
    gargalos(dist, livre, res, args.raio)


if __name__ == '__main__':
    main()
