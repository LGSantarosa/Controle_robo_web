#!/usr/bin/env python3
"""Deriva os extratos de `docs/baselines/` a partir dos CSVs brutos de uma volta.

Por que existe (2026-08-31, achado do revisor): os extratos arquivados tornavam
as conclusões INSPECIONÁVEIS, mas não REPRODUZÍVEIS — eu os gerei com um script
de uma linha só, no shell, que não ficou no repo. Quem quisesse refazer a conta
teria que adivinhar o critério (o que conta como "samba", qual coluna é a folga).

Os CSVs brutos moram em `log/sim_ab/<tag>/`, que é `gitignore`d: este script não
os inventa, ele **falha** se não estiverem lá. O que ele garante é que o caminho
do bruto até o número arquivado está escrito e é rodável.

Uso:
    python3 tools/sim_ab/extrai_evidencia.py <destino> <tag> [<tag> ...]
    python3 tools/sim_ab/extrai_evidencia.py --autoteste

Exemplo (o que gerou a evidência das 3 voltas de repetição):
    python3 tools/sim_ab/extrai_evidencia.py \\
        docs/baselines/2026-08-31-arena-latchN latchN1 latchN2 latchN3
"""
import collections
import csv
import os
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BRUTO = os.path.join(RAIZ, 'log', 'sim_ab')

# O critério da "samba", num lugar só: sair de `goal_turn` de volta pro carrot
# COM O MESMO GOAL. A mesma transição com dist_goal de metros é goal novo — o
# latch soltando, que é legítimo. Ver DIARIO_ARENA §2B.4.
SAMBA_DIST_MAX = 0.5


def _w(f):
    """csv.writer com LF. O default é \\r\\n — foi o BO 31, e o BO 50 de novo."""
    return csv.writer(f, lineterminator='\n')


def _ler(tag, nome):
    p = os.path.join(BRUTO, tag, nome)
    if not os.path.exists(p):
        raise SystemExit('nao existe: %s\n(os CSVs brutos sao gitignored; '
                         'rode a volta antes, ver DIARIO_ARENA §4.5)' % p)
    with open(p) as f:
        return list(csv.DictReader(f))


def colisao(destino, tags):
    """Contato: menor folga por objeto e contagem de eventos, por volta."""
    with open(os.path.join(destino, 'colisao_3voltas.csv'), 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'objeto', 'folga_min_m',
                    'eventos_COLISAO', 'eventos_raspao'])
        for tag in tags:
            rows = _ler(tag, 'colisao.csv')
            ev = collections.Counter((r['obj'], r['evento'])
                                     for r in rows if r['evento'])
            best = {}
            for r in rows:
                o, v = r['obj'], float(r['folga_min'])
                if o not in best or v < best[o]:
                    best[o] = v
            for o, v in sorted(best.items(), key=lambda kv: kv[1]):
                w.writerow([tag, o, '%.4f' % v,
                            ev.get((o, 'COLISAO'), 0), ev.get((o, 'raspao'), 0)])


def transicoes(destino, tags):
    """Samba: toda transição de/para `goal_turn`, com o dist_goal que separa
    'saiu da chegada' (defeito) de 'goal novo' (o latch soltando)."""
    with open(os.path.join(destino, 'transicoes_goal_turn_3voltas.csv'),
              'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 't_rel', 'de', 'para',
                    'wz_antes', 'wz_depois', 'dist_goal'])
        for tag in tags:
            rr = _ler(tag, 'follow_debug.csv')
            t0 = float(rr[0]['t'])
            for a, b in zip(rr, rr[1:]):
                if a['state'] != b['state'] and 'goal_turn' in (a['state'], b['state']):
                    w.writerow([tag, '%.1f' % (float(b['t']) - t0),
                                a['state'], b['state'],
                                a['wz'], b['wz'], b['dist_goal']])


def unstuck(destino, tags):
    """Unstuck: cada troca de estado, com reason/stuck_s/nav_wants."""
    cols = ['volta', 't', 'state', 'reason', 'ang', 'nav_wants',
            'stuck_s', 'x', 'y']
    with open(os.path.join(destino, 'unstuck_disparos_3voltas.csv'),
              'w', newline='') as f:
        w = _w(f)
        w.writerow(cols)
        for tag in tags:
            prev = None
            for r in _ler(tag, 'unstuck.csv'):
                if r['state'] != prev:
                    w.writerow([tag] + [r.get(c, '') for c in cols[1:]])
                    prev = r['state']


def conta_samba(rows):
    """Quantas vezes o seguidor SAIU da chegada de volta pro carrot, no MESMO
    goal. É o número que a §2B.4/§2B.5 chama de samba."""
    return sum(1 for a, b in zip(rows, rows[1:])
               if a['state'] == 'goal_turn' and b['state'] == 'turning'
               and float(b['dist_goal']) < SAMBA_DIST_MAX)


def autoteste():
    """O critério da samba tem que separar defeito de goal novo."""
    casos = [
        ([{'state': 'goal_turn', 'dist_goal': '0.16'},
          {'state': 'turning', 'dist_goal': '0.17'}], 1,
         'saiu da chegada com o MESMO goal = samba'),
        ([{'state': 'goal_turn', 'dist_goal': '0.15'},
          {'state': 'turning', 'dist_goal': '6.81'}], 0,
         'dist_goal de metros = GOAL NOVO, o latch soltando'),
        ([{'state': 'turning', 'dist_goal': '0.15'},
          {'state': 'goal_turn', 'dist_goal': '0.14'}], 0,
         'ENTRAR na chegada nao e samba'),
        ([{'state': 'goal_turn', 'dist_goal': '0.16'},
          {'state': 'arrived', 'dist_goal': '0.17'}], 0,
         'goal_turn -> arrived eh o caminho feliz'),
    ]
    ruim = 0
    for rows, esperado, porque in casos:
        got = conta_samba(rows)
        ok = got == esperado
        ruim += not ok
        print('%s %-52s %d' % ('ok  ' if ok else 'FALHA', porque, got))
    return ruim


def main():
    if '--autoteste' in sys.argv:
        return 1 if autoteste() else 0
    if len(sys.argv) < 3:
        raise SystemExit('USO: extrai_evidencia.py <destino> <tag> [<tag> ...]')
    destino, tags = sys.argv[1], sys.argv[2:]
    if not os.path.isdir(destino):
        raise SystemExit('destino nao existe: %s' % destino)
    colisao(destino, tags)
    transicoes(destino, tags)
    unstuck(destino, tags)
    print('extratos de %s -> %s' % (', '.join(tags), destino))
    return 0


if __name__ == '__main__':
    sys.exit(main())
