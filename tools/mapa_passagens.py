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
    """resolution + image + origin do .yaml do map_server (sem depender de pyyaml)."""
    res, img, origin = 0.05, None, (0.0, 0.0)
    for linha in open(caminho):
        linha = linha.split('#')[0].strip()
        if linha.startswith('resolution:'):
            res = float(linha.split(':', 1)[1])
        elif linha.startswith('image:'):
            img = linha.split(':', 1)[1].strip()
        elif linha.startswith('origin:'):
            v = linha.split(':', 1)[1].strip().strip('[]').split(',')
            origin = (float(v[0]), float(v[1]))
    if img is None:
        raise ValueError(f'{caminho}: sem campo "image:"')
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(caminho)), img)
    return res, img, origin


def mundo_para_celula(x, y, res, origin, altura):
    """(x,y) em metros -> (linha, coluna) do PGM.

    Convencao do map_server: `origin` e' o canto INFERIOR-ESQUERDO da imagem, e a
    linha 0 do PGM e' o TOPO. Por isso o y inverte.
    """
    col = int(round((x - origin[0]) / res))
    lin = int(round((altura - 1) - (y - origin[1]) / res))
    return lin, col


def probes(dist, livre, res, origin, pares, raios):
    """Conectividade LOCAL: A e B caem no mesmo trecho navegavel, em cada raio?

    O relatorio de cima mede o MAIOR COMPONENTE, que pode continuar em 100%
    enquanto uma fresta especifica fechou (o pedaco perdido e' pequeno demais pra
    mover o percentual). Isto aqui responde a pergunta que interessa de verdade:
    "o robo ainda vai de A ate B com este raio?"
    """
    altura = dist.shape[0]
    print()
    print('  PROBES LOCAIS (conectividade A->B, o que o percentual acima esconde)')
    todos_ok = True
    for (ax, ay, bx, by, rotulo) in pares:
        la, ca = mundo_para_celula(ax, ay, res, origin, altura)
        lb, cb = mundo_para_celula(bx, by, res, origin, altura)
        fora = []
        for nome, (l, c) in (('A', (la, ca)), ('B', (lb, cb))):
            if not (0 <= l < altura and 0 <= c < dist.shape[1]):
                fora.append(nome)
        print(f'    {rotulo}: ({ax:.2f},{ay:.2f}) -> ({bx:.2f},{by:.2f})')
        if fora:
            print(f'       ⚠️ ponto {"/".join(fora)} fora do mapa — probe invalido')
            todos_ok = False
            continue
        for r in raios:
            nav = livre & (dist >= r)
            lab, _ = ndimage.label(nav)
            va, vb = lab[la, ca], lab[lb, cb]
            if va == 0 or vb == 0:
                qual = 'A' if va == 0 else ('B' if vb == 0 else 'A e B')
                print(f'       raio {r:.3f}: ✗ {qual} nao e navegavel '
                      f'(folga A={dist[la, ca]:.2f} B={dist[lb, cb]:.2f} m)')
            elif va == vb:
                print(f'       raio {r:.3f}: ✓ LIGADOS')
            else:
                print(f'       raio {r:.3f}: ✗ SEPARADOS (trechos {va} e {vb})')
    return todos_ok


def analisa(yaml_path, raios):
    res, img, origin = le_yaml(yaml_path)
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
    return dist, livre, res, origin


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


def autoteste():
    """Mapa sintetico com fresta MEDIDA: prova a conversao mundo->celula e o probe.

    10x10 m @ 0.05, parede vertical no meio com um vao de EXATAMENTE 0.60 m.
    Com raio 0.25 (precisa 0.50) os dois lados tem que ficar LIGADOS; com 0.32
    (precisa 0.64) tem que ficar SEPARADOS. E' o caso da arena, em miniatura.
    """
    import tempfile
    res, W, H = 0.05, 200, 200
    a = np.full((H, W), 255, dtype=np.uint8)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 0          # muros externos
    col = W // 2
    a[:, col - 1:col + 1] = 0                            # parede vertical
    vao = int(round(0.60 / res))                         # 12 celulas = 0.60 m
    meio = H // 2
    a[meio - vao // 2:meio - vao // 2 + vao, col - 1:col + 1] = 255
    d = tempfile.mkdtemp()
    pgm = os.path.join(d, 'auto.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(a.tobytes())
    yml = os.path.join(d, 'auto.yaml')
    open(yml, 'w').write(f'image: auto.pgm\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\n')

    raios = [0.25, 0.32]
    dist, livre, r_, origin = analisa(yml, raios)
    # um ponto de cada lado da parede, na altura do vao
    y_vao = (H - 1 - meio) * res
    pares = [(1.0, y_vao, 9.0, y_vao, 'atravessa a fresta de 0.60 m')]
    probes(dist, livre, r_, origin, pares, raios)

    nav25 = livre & (dist >= 0.25); nav32 = livre & (dist >= 0.32)
    l25, _ = ndimage.label(nav25); l32, _ = ndimage.label(nav32)
    la, ca = mundo_para_celula(1.0, y_vao, r_, origin, H)
    lb, cb = mundo_para_celula(9.0, y_vao, r_, origin, H)
    ok25 = l25[la, ca] != 0 and l25[la, ca] == l25[lb, cb]
    ok32 = l32[la, ca] != 0 and l32[lb, cb] != 0 and l32[la, ca] != l32[lb, cb]
    print()
    print(f'  [{"ok " if ok25 else "ERRO"}] raio 0.25 (precisa 0.50): LIGADOS')
    print(f'  [{"ok " if ok32 else "ERRO"}] raio 0.32 (precisa 0.64): SEPARADOS')
    bom = ok25 and ok32
    print('[autoteste] ' + ('TUDO CERTO' if bom else 'FALHOU'))
    return 0 if bom else 1


def folgas(dist, res, origin, pontos, raio):
    """Folga (meia-largura livre) NO ponto — mede a fresta em si, não a rota.

    O probe A->B diz se existe caminho; com contorno disponível ele diz "ligados"
    mesmo com a fresta fechada. Isto aqui mede o vão: `dist` é a distância até a
    parede mais próxima, então a passagem tem 2*dist de largura e o robô só cabe
    se dist >= robot_radius.
    """
    altura = dist.shape[0]
    print()
    print(f'  FOLGA NO VÃO (o robô passa se folga >= raio {raio:.3f})')
    for (x, y, rot) in pontos:
        l, c = mundo_para_celula(x, y, res, origin, altura)
        if not (0 <= l < altura and 0 <= c < dist.shape[1]):
            print(f'    {rot}: ⚠️ ({x:.2f},{y:.2f}) fora do mapa')
            continue
        d = dist[l, c]
        passa = d >= raio
        print(f'    {rot}: folga {d:.3f} m (corredor {2*d:.2f} m) -> '
              f'{"✓ PASSA" if passa else "✗ FECHADO"}')


def ponto_folga(txt):
    """'x,y[:rotulo]' -> (x, y, rotulo)"""
    partes = txt.split(':')
    a = [float(v) for v in partes[0].split(',')]
    return (a[0], a[1], partes[1] if len(partes) > 1 else partes[0])


def par_probe(txt):
    """'x1,y1:x2,y2[:rotulo]' -> (x1,y1,x2,y2,rotulo)"""
    partes = txt.split(':')
    a = [float(v) for v in partes[0].split(',')]
    b = [float(v) for v in partes[1].split(',')]
    rot = partes[2] if len(partes) > 2 else f'{partes[0]} -> {partes[1]}'
    return (a[0], a[1], b[0], b[1], rot)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mapa', nargs='?', help='caminho do .yaml do mapa')
    ap.add_argument('--raio', type=float, default=0.32,
                    help='robot_radius do costmap (default 0.32, o do perfil ARENA)')
    ap.add_argument('--probe', action='append', default=[], metavar='X1,Y1:X2,Y2[:rotulo]',
                    help='conectividade LOCAL entre dois pontos do MUNDO (metros). '
                         'Repetivel. O percentual do maior componente pode dizer '
                         '100%% com uma fresta fechada; isto nao.')
    ap.add_argument('--folga', action='append', default=[], metavar='X,Y[:rotulo]',
                    help='mede a folga NO PONTO (a fresta em si). O --probe diz se '
                         'existe rota; com contorno ele diz "ligados" mesmo com a '
                         'fresta fechada. Repetivel.')
    ap.add_argument('--autoteste', action='store_true',
                    help='mapa sintetico com fresta de 0.60 m conhecida')
    args = ap.parse_args()
    if args.autoteste:
        sys.exit(autoteste())
    if not args.mapa:
        ap.error('falta o mapa (ou use --autoteste)')
    if not os.path.exists(args.mapa):
        sys.exit(f'não achei {args.mapa}')
    raios = sorted({0.25, args.raio, 0.354})
    dist, livre, res, origin = analisa(args.mapa, raios)
    print()
    gargalos(dist, livre, res, args.raio)
    if args.probe:
        probes(dist, livre, res, origin, [par_probe(t) for t in args.probe], raios)
    if args.folga:
        folgas(dist, res, origin, [ponto_folga(t) for t in args.folga], args.raio)


if __name__ == '__main__':
    main()
