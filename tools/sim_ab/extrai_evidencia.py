#!/usr/bin/env python3
"""Deriva os extratos de `docs/baselines/` a partir dos CSVs brutos de uma volta.

Por que existe (2026-08-31, achado do revisor): os extratos arquivados tornavam
as conclusões INSPECIONÁVEIS, mas não REPRODUZÍVEIS — eu os gerei com um script
de uma linha só, no shell, que não ficou no repo. Quem quisesse refazer a conta
teria que adivinhar o critério (o que conta como "samba", qual coluna é a folga).

Os CSVs brutos moram em `log/sim_ab/<tag>/`, que é `gitignore`d: este script não
os inventa, e **não escreve nada** se algum faltar (ver `confere_bruto`).

Uso:
    python3 tools/sim_ab/extrai_evidencia.py <destino> <tag> [<tag> ...]
    python3 tools/sim_ab/extrai_evidencia.py --resumo <arquivo.csv> <tag> ...
    python3 tools/sim_ab/extrai_evidencia.py --autoteste

Exemplo (o que gerou a evidência das 3 voltas de repetição):
    python3 tools/sim_ab/extrai_evidencia.py \\
        docs/baselines/2026-08-31-arena-latchN latchN1 latchN2 latchN3
"""
import collections
import csv
import json
import os
import sys
import tempfile

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BRUTO = os.path.join(RAIZ, 'log', 'sim_ab')
PRECISA = ('colisao.csv', 'follow_debug.csv', 'unstuck.csv', 'result.json')

# O critério da "samba", num lugar só: sair de `goal_turn` de volta pro carrot
# COM O MESMO GOAL. A mesma transição com dist_goal de metros é goal novo — o
# latch soltando, que é legítimo. Ver DIARIO_ARENA §2B.4.
SAMBA_DIST_MAX = 0.5


def _w(f):
    """csv.writer com LF. O default é \\r\\n — foi o BO 31, e o BO 50 de novo."""
    return csv.writer(f, lineterminator='\n')


def _ler(tag, nome):
    with open(os.path.join(BRUTO, tag, nome)) as f:
        return list(csv.DictReader(f))


def confere_bruto(tags):
    """TODOS os brutos, ANTES de abrir qualquer destino.

    O revisor pegou (2026-08-31): a primeira versão validava dentro do `_ler()`,
    com o destino já aberto em modo `'w'`. Num clone limpo, o comando do README
    **truncava** `colisao_3voltas.csv`, escrevia o cabeçalho e só então falhava —
    destruía a evidência boa pra descobrir que não podia gerar a nova, e um bruto
    faltando no meio deixava extratos pela metade. Isso contradizia a promessa de
    "falha sem inventar dado": não inventava, mas apagava.
    """
    faltam = [os.path.join(BRUTO, t, n) for t in tags for n in PRECISA
              if not os.path.exists(os.path.join(BRUTO, t, n))]
    if faltam:
        raise SystemExit('faltam CSVs brutos (NADA foi escrito):\n  %s\n'
                         '(sao gitignored; rode a volta antes, ver '
                         'DIARIO_ARENA §4.5)' % '\n  '.join(faltam))


# As fases da chegada e os estados que são "voltar pro carrot". Em 2026-08-31 a
# aproximação final criou o `goal_approach`, e o critério antigo só olhava
# `goal_turn` -> `turning`: uma samba pela porta nova passaria batido. As voltas
# `aprox1..3` não tiveram nenhuma (as 2 saídas de `goal_approach` foram com
# dist_goal > 6 m = goal novo), mas a métrica estava cega, o que é diferente de
# estar certa.
CHEGADA = ('goal_approach', 'goal_turn')
CARROT = ('turning', 'driving')


def conta_samba(rows):
    """Quantas vezes o seguidor SAIU da chegada de volta pro carrot, no MESMO
    goal. É o número que a §2B.4/§2B.5 chama de samba."""
    return sum(1 for a, b in zip(rows, rows[1:])
               if a['state'] in CHEGADA and b['state'] in CARROT
               and float(b['dist_goal']) < SAMBA_DIST_MAX)


def folga_por_objeto(rows):
    """Menor folga vista por objeto, na volta inteira."""
    best = {}
    for r in rows:
        o, v = r['obj'], float(r['folga_min'])
        if o not in best or v < best[o]:
            best[o] = v
    return best


# ---- os geradores: cada um escreve UM arquivo, no caminho que receber --------

def colisao(saida, tags):
    """Contato: menor folga por objeto e contagem de eventos, por volta."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'objeto', 'folga_min_m',
                    'eventos_COLISAO', 'eventos_raspao'])
        for tag in tags:
            rows = _ler(tag, 'colisao.csv')
            ev = collections.Counter((r['obj'], r['evento'])
                                     for r in rows if r['evento'])
            for o, v in sorted(folga_por_objeto(rows).items(), key=lambda kv: kv[1]):
                w.writerow([tag, o, '%.4f' % v,
                            ev.get((o, 'COLISAO'), 0), ev.get((o, 'raspao'), 0)])


def transicoes(saida, tags):
    """Samba: toda transição de/para `goal_turn`, com o dist_goal que separa
    'saiu da chegada' (defeito) de 'goal novo' (o latch soltando)."""
    with open(saida, 'w', newline='') as f:
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


def unstuck(saida, tags):
    """Unstuck: cada troca de estado, com reason/stuck_s/nav_wants."""
    cols = ['volta', 't', 'state', 'reason', 'ang', 'nav_wants',
            'stuck_s', 'x', 'y']
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(cols)
        for tag in tags:
            prev = None
            for r in _ler(tag, 'unstuck.csv'):
                if r['state'] != prev:
                    w.writerow([tag] + [r.get(c, '') for c in cols[1:]])
                    prev = r['state']


def resumo(saida, tags):
    """A tabela de uma linha por volta. Usa o `conta_samba` — antes essa coluna
    era etapa MANUAL (o revisor apontou), então o número da tabela e o número do
    extrato podiam divergir sem ninguém ver."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'total_s', 'goals_ok', 'COLISAO', 'raspao',
                    'folga_min_m', 'samba', 'unstuck_s', 'parado_por_goal'])
        for tag in tags:
            with open(os.path.join(BRUTO, tag, 'result.json')) as fh:
                r = json.load(fh)
            rows = _ler(tag, 'colisao.csv')
            ev = collections.Counter(x['evento'] for x in rows if x['evento'])
            w.writerow([
                tag, r['total_s'],
                '%d/5' % sum(1 for g in r['goals'] if g['status'] == 'OK'),
                ev.get('COLISAO', 0), ev.get('raspao', 0),
                '%.4f' % min(folga_por_objeto(rows).values()),
                conta_samba(_ler(tag, 'follow_debug.csv')),
                '%.1f' % sum(g.get('unstuck', 0) for g in r['goals']),
                ' '.join('g%d:%.1f' % (g['goal'], g['parado']) for g in r['goals'])])


def dist_final(saida, tags):
    """A que distância do goal a ação COMPLETOU — a medida direta do defeito
    2e (o robô estacionando fora do `xy_goal_tolerance` do Nav2, 0,15).

    O fim de um goal é o tick em que `dist_goal` salta de centímetros pra metros
    (o plano passa a apontar pro goal seguinte)."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'goal_n', 'dist_final_m', 'dentro_do_checker_0.15'])
        for tag in tags:
            fd = _ler(tag, 'follow_debug.csv')
            finais = [float(a['dist_goal']) for a, b in zip(fd, fd[1:])
                      if float(a['dist_goal']) < 0.5 and float(b['dist_goal']) > 2.0]
            if fd and float(fd[-1]['dist_goal']) < 0.5:
                finais.append(float(fd[-1]['dist_goal']))
            for i, d in enumerate(finais, 1):
                w.writerow([tag, i, '%.3f' % d, 'sim' if d <= 0.15 else 'NAO'])


ARQUIVOS = (('dist_final_por_goal.csv', dist_final),
            ('colisao_3voltas.csv', colisao),
            ('transicoes_goal_turn_3voltas.csv', transicoes),
            ('unstuck_disparos_3voltas.csv', unstuck))


def gerar(destino, tags, arquivos=ARQUIVOS):
    """Gera TUDO em temporários e só troca os destinos se todos derem certo.

    Sem isso, uma falha no terceiro arquivo deixa os dois primeiros já
    sobrescritos — meio extrato novo, meio velho, sem ninguém avisar.
    """
    confere_bruto(tags)
    tmps = []
    try:
        for nome, fn in arquivos:
            fd, tmp = tempfile.mkstemp(dir=destino, prefix='.%s.' % nome)
            os.close(fd)
            # entra na lista ANTES de gerar: se o `fn` explodir, o `finally`
            # ainda precisa apagar ESTE temporário. (Achado pelo próprio
            # autoteste: com o append depois, o temporário do gerador que falha
            # vazava na pasta.)
            tmps.append((tmp, os.path.join(destino, nome)))
            fn(tmp, tags)
        for tmp, alvo in tmps:
            os.replace(tmp, alvo)   # atômico no mesmo filesystem
        tmps = []
    finally:
        for tmp, _ in tmps:
            if os.path.exists(tmp):
                os.remove(tmp)


# ---- autotestes -------------------------------------------------------------

def _autoteste_samba():
    casos = [
        ([{'state': 'goal_turn', 'dist_goal': '0.16'},
          {'state': 'turning', 'dist_goal': '0.17'}], 1,
         'saiu da chegada com o MESMO goal = samba'),
        ([{'state': 'goal_approach', 'dist_goal': '0.16'},
          {'state': 'turning', 'dist_goal': '0.17'}], 1,
         'samba pela porta NOVA (goal_approach) tambem conta'),
        ([{'state': 'goal_approach', 'dist_goal': '0.12'},
          {'state': 'driving', 'dist_goal': '0.13'}], 1,
         'voltar pro carrot em driving tambem e samba'),
        ([{'state': 'goal_approach', 'dist_goal': '0.15'},
          {'state': 'turning', 'dist_goal': '6.32'}], 0,
         'goal_approach -> turning com metros = goal novo'),
        ([{'state': 'goal_approach', 'dist_goal': '0.12'},
          {'state': 'goal_turn', 'dist_goal': '0.11'}], 0,
         'approach -> goal_turn e a sequencia normal'),
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


def _autoteste_atomico():
    """O BO do revisor: bruto faltando não pode ENCOSTAR no destino."""
    import shutil
    ruim = 0
    d = tempfile.mkdtemp()
    try:
        antes = {}
        for nome, _ in ARQUIVOS:
            p = os.path.join(d, nome)
            open(p, 'w').write('EVIDENCIA BOA QUE NAO PODE SUMIR\n')
            antes[p] = open(p).read()

        try:
            gerar(d, ['_tag_que_nao_existe_'])
            print('FALHA %-52s' % 'bruto faltando devia abortar')
            ruim += 1
        except SystemExit as e:
            ok = 'NADA foi escrito' in str(e)
            ruim += not ok
            print('%s %-52s' % ('ok  ' if ok else 'FALHA',
                                'bruto faltando aborta com a mensagem certa'))

        intactos = all(open(p).read() == v for p, v in antes.items())
        ruim += not intactos
        print('%s %-52s' % ('ok  ' if intactos else 'FALHA',
                            'destino INTACTO depois do aborto'))

        # falha NO MEIO: o segundo gerador explode -> nenhum destino trocado
        def bomba(saida, tags):
            raise RuntimeError('boom')

        def ok_(saida, tags):
            open(saida, 'w').write('novo\n')

        try:
            gerar(d, [], arquivos=((ARQUIVOS[0][0], ok_),
                                   (ARQUIVOS[1][0], bomba)))
        except RuntimeError:
            pass
        intactos = all(open(p).read() == v for p, v in antes.items())
        ruim += not intactos
        print('%s %-52s' % ('ok  ' if intactos else 'FALHA',
                            'falha NO MEIO nao troca nenhum destino'))
        sobrou = [f for f in os.listdir(d) if f.startswith('.')]
        ruim += bool(sobrou)
        print('%s %-52s %s' % ('ok  ' if not sobrou else 'FALHA',
                               'sem temporario esquecido na pasta', sobrou or ''))
    finally:
        shutil.rmtree(d, ignore_errors=True)
    return ruim


def main():
    if '--autoteste' in sys.argv:
        return 1 if (_autoteste_samba() + _autoteste_atomico()) else 0
    if '--resumo' in sys.argv:
        i = sys.argv.index('--resumo')
        saida, tags = sys.argv[i + 1], sys.argv[i + 2:]
        if not tags:
            raise SystemExit('USO: --resumo <arquivo.csv> <tag> [<tag> ...]')
        confere_bruto(tags)
        gerar(os.path.dirname(os.path.abspath(saida)), tags,
              arquivos=((os.path.basename(saida), resumo),))
        print('resumo de %s -> %s' % (', '.join(tags), saida))
        return 0
    if len(sys.argv) < 3:
        raise SystemExit('USO: extrai_evidencia.py <destino> <tag> [<tag> ...]')
    destino, tags = sys.argv[1], sys.argv[2:]
    if not os.path.isdir(destino):
        raise SystemExit('destino nao existe: %s' % destino)
    gerar(destino, tags)
    print('extratos de %s -> %s' % (', '.join(tags), destino))
    return 0


if __name__ == '__main__':
    sys.exit(main())
