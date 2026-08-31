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
    python3 tools/sim_ab/extrai_evidencia.py --guard  <arquivo.csv> <tag> ...
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
PRECISA = ('colisao.csv', 'follow_debug.csv', 'unstuck.csv', 'result.json',
           'freeze_capture.csv')

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
    """Onde o robô estava no ÚLTIMO TICK antes de o plano apontar pro goal
    seguinte. É o indicador do defeito 2e (o robô parando longe do goal).

    ⚠️ **Não é** "a pose no instante da conclusão" nem "o que o checker julgou"
    (review 08-31, BO 61): a 20 Hz e 0,22 m/s há **~1,1 cm entre amostras**, e o
    `goal_checker` é `stateful: true` — satisfeito o XY uma vez, ele só reconfere
    yaw, então pode ter aceitado bem antes e em outra posição. Por isso a coluna
    se chama `ultima_amostra_le_0.15`, e não "dentro do checker"."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'goal_n', 'dist_ultima_amostra_m',
                    'ultima_amostra_le_0.15'])
        for tag in tags:
            fd = _ler(tag, 'follow_debug.csv')
            finais = [float(a['dist_goal']) for a, b in zip(fd, fd[1:])
                      if float(a['dist_goal']) < 0.5 and float(b['dist_goal']) > 2.0]
            if fd and float(fd[-1]['dist_goal']) < 0.5:
                finais.append(float(fd[-1]['dist_goal']))
            for i, d in enumerate(finais, 1):
                w.writerow([tag, i, '%.3f' % d, 'sim' if d <= 0.15 else 'NAO'])


# Dentro de `goal_approach` o estado NÃO distingue mirar de avançar — quem
# distingue é o `vx` (foi o BO 63). Uma alternância é uma troca mira<->avanço.
MIRA_VX_MIN = 0.01


def conta_churn(rows):
    """(ticks em goal_approach, alternâncias mira<->avanço)."""
    modo = [abs(float(r['vx'])) > MIRA_VX_MIN
            for r in rows if r['state'] == 'goal_approach']
    return len(modo), sum(1 for a, b in zip(modo, modo[1:]) if a != b)


def churn(saida, tags):
    """O número que a histerese foi feita pra derrubar (§2B.7). Era um
    one-liner de shell — o mesmo defeito que este script existe pra corrigir."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'ticks_goal_approach', 'alternancias_mira_avanco'])
        for tag in tags:
            n, alt = conta_churn(_ler(tag, 'follow_debug.csv'))
            w.writerow([tag, n, alt])


# Bloqueio longo pelo `motion_guard` tem duração de assinatura: `hold_still_max`
# (20s, vigília em cima de presença PARADA) + `clear_time` (5s) + settle (<=4s).
# Ver DIARIO_ARENA §2B.7.
GUARD_MIN_S = 1.0           # episódio menor que isso é transitório, não parada
CONGELADO_MIN_S = 10.0      # pose IDÊNTICA por mais que isso = robô travado


def episodios_guard(rows):
    """Trechos em que o `motion_guard` esteve `blocked`, com os ticks de giro
    que entraram nele (`auto_vel_pre`) e os que saíram (`auto_vel_raw`).

    `t` é RELATIVO à primeira linha do `freeze_capture.csv` — base de tempo
    diferente da do `colisao.csv`, que começa antes (ver o README do baseline).
    """
    t0 = None
    ini = None
    eps = []
    for r in rows:
        try:
            t = float(r['t_wall'])
        except (TypeError, ValueError):
            continue
        if t0 is None:
            t0 = t
        if r['topic'] != 'guard_state':
            continue
        if r['extra'] == 'blocked' and ini is None:
            ini = t
        elif r['extra'] != 'blocked' and ini is not None:
            eps.append((ini, t))
            ini = None
    if ini is not None:                 # bloqueado até o fim do log
        eps.append((ini, float(rows[-1]['t_wall'])))
    saida = []
    for a, b in eps:
        if b - a < GUARD_MIN_S:
            continue
        # o guard zera vx E wz (parada total), então contar só o giro
        # sub-reporta: na `aprox2` o seguidor mandava vx=0.30 reto, e a conta
        # de giro dava zero — parecia que ninguém queria andar.
        entrou = saiu = 0
        for r in rows:
            try:
                t = float(r['t_wall'])
            except (TypeError, ValueError):
                continue
            if not a <= t <= b or r['topic'] not in ('auto_vel_pre', 'auto_vel_raw'):
                continue
            vx = float(r['vx']) if r['vx'] else 0.0
            wz = float(r['wz']) if r['wz'] else 0.0
            if abs(vx) <= 0.02 and abs(wz) <= 0.05:
                continue
            if r['topic'] == 'auto_vel_pre':
                entrou += 1
            else:
                saiu += 1
        saida.append((a - t0, b - t0, 'cmd_entrou=%d cmd_saiu=%d'
                      % (entrou, saiu)))
    return saida


def episodios_congelado(rows):
    """Trechos com a pose do GROUND TRUTH idêntica (robô parado de fato), com o
    objeto mais próximo durante o trecho. `t` é o do `colisao.csv`."""
    eps = []
    i = 0
    while i < len(rows):
        j = i
        while (j + 1 < len(rows)
               and (rows[j + 1]['x'], rows[j + 1]['y']) == (rows[i]['x'], rows[i]['y'])):
            j += 1
        a, b = float(rows[i]['t']), float(rows[j]['t'])
        if b - a >= CONGELADO_MIN_S:
            perto = min(rows[i:j + 1], key=lambda r: float(r['folga_min']))
            eps.append((a, b, 'obj=%s folga_min=%s' % (perto['obj'],
                                                       perto['folga_min'])))
        i = j + 1
    return eps


def guard(saida, tags):
    """As paradas longas, pelas DUAS fontes independentes: o estado do
    `motion_guard` e a pose do ground truth. Elas não compartilham base de
    tempo — o que as liga é a DURAÇÃO (§2B.7)."""
    with open(saida, 'w', newline='') as f:
        w = _w(f)
        w.writerow(['volta', 'fonte', 't_ini', 't_fim', 'dur_s', 'detalhe'])
        for tag in tags:
            for fonte, eps in (
                    ('guard_blocked', episodios_guard(_ler(tag, 'freeze_capture.csv'))),
                    ('pose_congelada', episodios_congelado(_ler(tag, 'colisao.csv')))):
                for a, b, det in eps:
                    w.writerow([tag, fonte, '%.1f' % a, '%.1f' % b,
                                '%.1f' % (b - a), det])


ARQUIVOS = (('dist_final_por_goal.csv', dist_final),
            ('colisao_3voltas.csv', colisao),
            ('transicoes_goal_turn_3voltas.csv', transicoes),
            ('unstuck_disparos_3voltas.csv', unstuck),
            ('guard_bloqueio.csv', guard),
            ('churn_mira.csv', churn))


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


def _autoteste_churn():
    """Prova SENSÍVEL: o defeito é contar estados iguais (BO 63) em vez de
    trocas mira<->avanço, ou deixar outro estado entrar na conta."""
    def fd(seq):    # (state, vx)
        return [{'state': st, 'vx': '%.3f' % vx} for st, vx in seq]

    A = 'goal_approach'
    casos = [
        ('mira->avanco->mira = 2 alternancias',
         conta_churn(fd([(A, 0.0), (A, 0.2), (A, 0.0)])), (3, 2)),
        ('avanco continuo = 0 alternancias',
         conta_churn(fd([(A, 0.2), (A, 0.2), (A, 0.2)])), (3, 0)),
        ('so mira = 0 alternancias (o estado repetido NAO conta)',
         conta_churn(fd([(A, 0.0), (A, 0.0), (A, 0.0)])), (3, 0)),
        ('fora de goal_approach nao entra',
         conta_churn(fd([('driving', 0.0), ('driving', 0.3), (A, 0.0)])), (1, 0)),
        ('estado que sai e volta nao inventa alternancia',
         conta_churn(fd([(A, 0.0), ('turning', 2.4), (A, 0.0)])), (2, 0)),
    ]
    ruim = 0
    for nome, got, esperado in casos:
        ok = got == esperado
        ruim += not ok
        print('%s %-52s %s' % ('ok  ' if ok else 'FALHA', nome,
                               '' if ok else 'got=%r esperado=%r' % (got, esperado)))
    return ruim


def _autoteste_guard():
    """Prova SENSÍVEL (BO 63): cada caso tem um par que o defeito produziria.

    O defeito plausível aqui é somar transitório de guard como parada, ou
    chamar de "congelado" qualquer pausa curta — os dois inflariam o número
    que sustenta a §2B.7.
    """
    def fc(pares, ticks=()):
        rows = [{'t_wall': '1000.0', 'topic': 'odom', 'vx': '', 'wz': '',
                 'extra': ''}]
        for t, e in pares:
            rows.append({'t_wall': '%.3f' % (1000.0 + t), 'topic': 'guard_state',
                         'vx': '', 'wz': '', 'extra': e})
        for t, top, vx, wz in ticks:
            rows.append({'t_wall': '%.3f' % (1000.0 + t), 'topic': top,
                         'vx': '%.3f' % vx, 'wz': '%.3f' % wz, 'extra': ''})
        return sorted(rows, key=lambda r: float(r['t_wall']))

    def col(seq):
        # seq: (t, x, y, folga, obj)
        return [{'t': '%.2f' % t, 'x': x, 'y': y, 'folga_min': '%.4f' % fo,
                 'obj': o, 'yaw_deg': '0.0', 'evento': ''}
                for t, x, y, fo, o in seq]

    ruim = 0
    casos = [
        ('bloqueio de 26.9s vira 1 episodio',
         len(episodios_guard(fc([(5, 'idle'), (10, 'blocked'), (36.9, 'idle')]))), 1),
        ('duracao do episodio e a medida',
         round(episodios_guard(fc([(10, 'blocked'), (36.9, 'idle')]))[0][1]
               - episodios_guard(fc([(10, 'blocked'), (36.9, 'idle')]))[0][0], 1), 26.9),
        ('transitorio de 0.1s NAO conta',
         len(episodios_guard(fc([(10, 'blocked'), (10.1, 'idle')]))), 0),
        ('volta sem blocked da 0 episodios',
         len(episodios_guard(fc([(5, 'idle')]))), 0),
        ('conta comando que ENTRA e o que SAI',
         episodios_guard(fc([(10, 'blocked'), (36.9, 'idle')],
                            [(20, 'auto_vel_pre', 0.0, 2.4),
                             (21, 'auto_vel_pre', 0.0, 2.4),
                             (22, 'auto_vel_raw', 0.0, 0.0)]))[0][2],
         'cmd_entrou=2 cmd_saiu=0'),
        ('avanco reto zerado tambem conta (BO da aprox2)',
         episodios_guard(fc([(10, 'blocked'), (36.9, 'idle')],
                            [(20, 'auto_vel_pre', 0.3, 0.0),
                             (21, 'auto_vel_raw', 0.0, 0.0)]))[0][2],
         'cmd_entrou=1 cmd_saiu=0'),
        ('pose identica por 26.9s e episodio',
         len(episodios_congelado(col([(100, '1.0', '2.0', 0.9, 'muro_sul'),
                                      (120, '3.0', '4.0', 0.31, 'cone_3'),
                                      (146.9, '3.0', '4.0', 0.35, 'cone_3'),
                                      (150, '5.0', '6.0', 0.9, 'muro_sul')]))), 1),
        ('reporta o objeto MAIS PROXIMO do trecho',
         episodios_congelado(col([(120, '3.0', '4.0', 0.31, 'cone_3'),
                                  (146.9, '3.0', '4.0', 0.35, 'cone_3'),
                                  (150, '5.0', '6.0', 0.9, 'muro_sul')]))[0][2],
         'obj=cone_3 folga_min=0.3100'),
        ('pausa de 2s NAO e congelamento',
         len(episodios_congelado(col([(120, '3.0', '4.0', 0.31, 'cone_3'),
                                      (122, '3.0', '4.0', 0.31, 'cone_3'),
                                      (130, '5.0', '6.0', 0.9, 'muro_sul')]))), 0),
    ]
    for nome, got, esperado in casos:
        ok = got == esperado
        ruim += not ok
        print('%s %-52s %s' % ('ok  ' if ok else 'FALHA', nome,
                               '' if ok else 'got=%r esperado=%r' % (got, esperado)))
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
        return 1 if (_autoteste_samba() + _autoteste_churn()
                     + _autoteste_guard() + _autoteste_atomico()) else 0
    for flag, fn in (('--resumo', resumo), ('--guard', guard)):
        if flag not in sys.argv:
            continue
        i = sys.argv.index(flag)
        saida, tags = sys.argv[i + 1], sys.argv[i + 2:]
        if not tags:
            raise SystemExit('USO: %s <arquivo.csv> <tag> [<tag> ...]' % flag)
        confere_bruto(tags)
        gerar(os.path.dirname(os.path.abspath(saida)), tags,
              arquivos=((os.path.basename(saida), fn),))
        print('%s de %s -> %s' % (flag[2:], ', '.join(tags), saida))
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
