"""Fresta A da arena do galpão — a máquina do `door_crossing` alimentada com as
poses REAIS medidas no sim (DIARIO_ARENA §2B.10 / §2G.10).

Por que este arquivo existe (spec `2026-09-01-fresta-a-door-crossing-design.md`,
§4.9 e §4.10): o vão de 0,90 m dá orçamento lateral de ±0,20 m, e o erro do AMCL
medido nesta arena chega a 0,49 m. Nenhuma solução map-relative passa 100% — quem
tem que controlar a travessia é a máquina, que é scan/pose-relative e é LÓGICA
PURA (roda sem ROS e sem Gazebo).

⚠️ VERMELHO PRIMEIRO: enquanto `maps/arena_galpao.doors.json` não existir, a
fresta A não está marcada como porta, `/doors` chega vazio e a máquina fica
`idle` para sempre — a volta rodaria idêntica à de hoje e eu poderia achar que
"testei" (é o bloqueador do §5.1/§5.2 do spec).
"""

import dataclasses
import json
import math
import sys
import os
import unittest

from robot_nav.door_crossing import (
    DoorCrossConfig,
    DoorCrossing,
    door_geometry,
    door_progress_lateral,
    crossing_yaw,
    will_clear,
    _extrai_doors,
    doors_de_arquivo,
    ENTREGA_DO_GIRO,
    ENTREGA_DO_GIRO_MAX,
    in_approach_region,
    janela_de_alinhamento_ok,
    passo_minimo_do_giro,
    ready_to_commit,
    valida_doors,
)

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DOORS_JSON = os.path.join(RAIZ, 'maps', 'arena_galpao.doors.json')

# Geometria da fresta A. Os blocos ficam em x = 7,5 com 0,60 m de espessura
# (7,20 a 7,80) e o vão vai de y 1,80 a 2,70.
# 2026-09-02 (pedido do dono): a porta é marcada na QUINA por onde o robô chega
# (x = 7,20), não no eixo do bloco (7,50). O bloco é um TÚNEL curto; marcando no
# meio, o robô se alinha para um plano 30 cm DENTRO da parede.
A_ESPERADA = {'a': (7.20, 1.80), 'b': (7.20, 2.70)}


def carrega_portas():
    with open(DOORS_JSON) as f:
        return json.load(f)['doors']


def porta_A(portas):
    """A fresta A é a porta cujo centro cai no vão medido, com tolerância de
    meia célula do mapa (0,05 m). Procurar pelo id seria frágil."""
    for d in portas:
        cx = (d['a'][0] + d['b'][0]) / 2.0
        cy = (d['a'][1] + d['b'][1]) / 2.0
        if abs(cx - 7.20) <= 0.05 and abs(cy - 2.25) <= 0.05:
            return d
    raise AssertionError(
        f'nenhuma porta na BOCA da fresta A (7.20 ; 2.25) em {DOORS_JSON}: {portas}')


class TestFrestaAMarcadaComoPorta(unittest.TestCase):
    """§5.1 — a fresta A tem que estar marcada, e com as bordas EXATAS dos
    blocos (não clicadas a olho): o eixo torto é erro que a máquina não vê."""

    def test_arquivo_de_portas_da_arena_existe(self):
        self.assertTrue(
            os.path.exists(DOORS_JSON),
            f'{DOORS_JSON} não existe — a fresta A não está marcada como porta, '
            'então o door_crossing sobe e fica idle para sempre. '
            'Gere com: python3 tools/gera_arena_galpao.py --mapa maps/')

    def test_batentes_batem_com_as_bordas_dos_blocos(self):
        d = porta_A(carrega_portas())
        pontos = sorted([tuple(d['a']), tuple(d['b'])], key=lambda p: p[1])
        self.assertAlmostEqual(pontos[0][0], A_ESPERADA['a'][0], places=3)
        self.assertAlmostEqual(pontos[0][1], A_ESPERADA['a'][1], places=3)
        self.assertAlmostEqual(pontos[1][0], A_ESPERADA['b'][0], places=3)
        self.assertAlmostEqual(pontos[1][1], A_ESPERADA['b'][1], places=3)

    def test_vao_marcado_tem_0_90_m(self):
        d = porta_A(carrega_portas())
        vao = math.hypot(d['b'][0] - d['a'][0], d['b'][1] - d['a'][1])
        self.assertAlmostEqual(vao, 0.90, places=3)

    def test_as_QUATRO_frestas_estao_marcadas(self):
        """2026-09-02, decisão do dono: marcar todas. Com a porta na QUINA a
        conta do §4.4-(a) fecha em todas (giro +31,7 a +39,6 cm) — antes, com a
        marcação no meio do bloco, a de 0,60 dava só +7,1 cm.

        ⚠️ A fresta C (0,60) segue sendo tratada como PAREDE pelo Nav2
        (`robot_radius 0.32`), então a porta dela é zona armada que o planejador
        não usa. Marcada por pedido do dono; não é erro, é escopo."""
        vaos = sorted(round(math.hypot(d['b'][0] - d['a'][0],
                                       d['b'][1] - d['a'][1]), 2)
                      for d in carrega_portas())
        self.assertEqual(vaos, [0.60, 0.70, 0.80, 0.90])

    def test_toda_porta_marcada_tem_margem_de_giro_POSITIVA(self):
        """O par que protege a decisão acima: marcar fresta em que o robô bate
        girando é pior que não marcar."""
        sys.path.insert(0, os.path.join(RAIZ, 'tools'))
        import gera_arena_galpao as ga
        for nome in ga.MARCADAS_COMO_PORTA:
            with self.subTest(nome):
                self.assertGreater(ga.margem_point_turn(nome), 0.15)


class TestDoorsFile(unittest.TestCase):
    """§5.2 — o nó tem que conseguir carregar porta do DISCO.

    `/doors` só é publicado pelo `controle_web`, e o harness A/B do sim não sobe
    o stack web. Sem isto o nó sobe, não recebe porta, fica idle, e a volta roda
    idêntica à de hoje — com a diferença de que eu poderia achar que testei."""

    def test_le_a_porta_da_arena_do_disco(self):
        d = doors_de_arquivo(DOORS_JSON)
        self.assertEqual(len(d), 4)
        self.assertEqual(d[0]['id'], 1)

    def test_caminho_vazio_e_sem_portas_nao_e_erro(self):
        """Fora da arena ninguém passa doors_file — não pode explodir."""
        self.assertEqual(doors_de_arquivo(''), [])
        self.assertEqual(
            doors_de_arquivo(os.path.join(RAIZ, 'maps',
                                          'arena_galpao_semA.doors.json')), [])

    def test_arquivo_QUE_NAO_EXISTE_erra_alto(self):
        """O par sensível: descartar em silêncio vira 'nó idle', que é
        indistinguível de 'não tem porta' — e aí o robô atravessa a fresta sem
        ninguém dirigindo. Deploy quebrado tem que aparecer."""
        with self.assertRaises(ValueError):
            doors_de_arquivo(os.path.join(RAIZ, 'maps', 'nao_existe.doors.json'))

    def test_porta_malformada_erra_alto(self):
        ruins = (
            ('não é lista', {'x': 1}),
            ('porta não é objeto', [42]),
            ('sem batente b', [{'id': 1, 'a': [1.0, 2.0]}]),
            ('batente com 1 número', [{'id': 1, 'a': [1.0], 'b': [2.0, 2.0]}]),
            ('batente não numérico', [{'id': 1, 'a': ['x', 2.0], 'b': [2.0, 2.0]}]),
            ('batentes no mesmo ponto', [{'id': 1, 'a': [1.0, 2.0], 'b': [1.0, 2.0]}]),
            ('sem id (o gate do arme usa o id)',
             [{'a': [1.0, 2.0], 'b': [1.0, 3.0]}]),
        )
        for rotulo, doors in ruins:
            with self.subTest(rotulo):
                with self.assertRaises(ValueError):
                    valida_doors(doors)

    def test_chave_doors_AUSENTE_erra_alto(self):
        """Achado do review 2026-09-02: `dados.get('doors', [])` tratava chave
        errada como ZERO PORTAS em silêncio — e zero portas é indistinguível de
        nó idle, que é a fresta sem ninguém dirigindo."""
        for ruim in ({'portas': []}, {}, [], 'x'):
            with self.subTest(repr(ruim)):
                with self.assertRaises(ValueError):
                    _extrai_doors(ruim, 'teste')

    def test_lista_VAZIA_explicita_continua_legitima(self):
        """O par: é o que o gerador escreve pro mapa tampado (--fecha-fresta),
        onde a fresta é parede e armar seria errado."""
        self.assertEqual(_extrai_doors({'doors': []}, 'teste'), [])

    def test_porta_boa_passa(self):
        """O par: se tudo erra, o validador não vale nada."""
        boa = [{'id': 1, 'a': [7.5, 1.8], 'b': [7.5, 2.7]}]
        self.assertEqual(valida_doors(boa), boa)


class TestPoseDaNoguard3(unittest.TestCase):
    """§4.9 — a amostra REAL do 1º raspão da `noguard3`."""

    # pose medida: 0,259 m fora do eixo e 7,7° torta na chegada da fresta.
    POSE_TORTA = (6.90, 2.509, math.radians(-7.7))
    POSE_BOA = (6.90, 2.25, 0.0)     # o par: já no eixo, já alinhada

    def _maquina(self):
        return DoorCrossing(DoorCrossConfig()), DoorCrossConfig()

    def _tick(self, m, t, pose, portas, goal_succeeded=False):
        return m.update(t, pose, portas, goal_active=True, nav_forward=True,
                        gap=3.0, scan_fresh=True, front_gap=3.0, rear_gap=3.0,
                        goal_succeeded=goal_succeeded)

    def _pre_porta_cumprido(self, m, pose, portas):
        """Entrega o pulso que a pendência C exige: um goal do Nav2 que TERMINA
        (SUCCEEDED) com o robô dentro da zona da porta. Na rota da prova isso é
        o waypoint pré-fresta (§2H.7) — hoje OPT-IN (`--pre-fresta`)."""
        self._tick(m, 0.0, pose, portas, goal_succeeded=True)

    def test_SEM_o_pre_porta_a_maquina_NUNCA_arma(self):
        """A pendência C, reproduzida: com a rota atual (sem waypoint
        pré-fresta) nenhum goal termina na zona, então `_cleared` fica vazio e o
        nó fica `idle` PARA SEMPRE — a volta roda idêntica à de hoje.
        É o segundo bloqueador de integração (§2H.5)."""
        portas = carrega_portas()
        m, _ = self._maquina()
        estados = {self._tick(m, i * 0.05, self.POSE_BOA, portas).state
                   for i in range(40)}
        self.assertEqual(estados, {'idle'},
                         f'armou sem o pré-porta cumprido: {estados}')

    def test_pose_torta_NAO_atravessa_direto(self):
        portas = carrega_portas()
        m, cfg = self._maquina()
        self._pre_porta_cumprido(m, self.POSE_TORTA, portas)
        cmd = self._tick(m, 0.05, self.POSE_TORTA, portas)
        self.assertNotEqual(cmd.state, 'crossing',
                            'entrou torta (lat 0,259 m) — é o raspão da noguard3')
        self.assertNotEqual(cmd.state, 'idle',
                            'a máquina não pegou a porta: com ela idle a volta '
                            'roda igual à de hoje')
        self.assertLessEqual(abs(cmd.vx), cfg.stage_speed + 1e-9,
                             'avançou em velocidade de rota com a porta armada')
        self.assertEqual(cmd.state, 'staging',
                         'entrada ruim agora tem que ser corrigida na AREA da '
                         'porta, nao pular direto para rotating')
        self.assertNotAlmostEqual(cmd.wz, 0.0, msg='tinha que estar ajeitando '
                                  'o heading na aproximacao')

    def test_pose_boa_ATRAVESSA_depois_de_estabilizar(self):
        """O par que prova o teste sensível: sem ele, a máquina desligada
        (sempre idle) passaria no teste de cima."""
        portas = carrega_portas()
        m, cfg = self._maquina()
        self._pre_porta_cumprido(m, self.POSE_BOA, portas)
        estados = []
        for i in range(1, cfg.align_stable + 4):
            estados.append(self._tick(m, i * 0.05, self.POSE_BOA, portas).state)
        self.assertIn('crossing', estados,
                      f'no eixo e alinhada, nunca commitou: {estados}')

    def _malha_fechada(self, pose0, portas, cfg=None, dt=0.05, tmax=60.0):
        """Roda a máquina realimentando o (vx, wz) que ela comanda na pose.

        ⚠️ **MODELO CALIBRADO, NÃO PREVISÃO.** O `wz` é multiplicado por
        `ENTREGA_DO_GIRO` (0,135, medido em 6099 ticks) porque sem isso o modelo
        dá ao robô 7× a autoridade de giro real — foi assim que eu "descobri" um
        ciclo-limite inexistente (erro 88). Mesmo calibrado ele acerta a
        ESTRUTURA e erra a MAGNITUDE: na entrada ruim dá +5,9 cm onde o sim
        mediu +13,4. Portanto: usar para afirmar **sequência de estados** e
        **decisão** (commitou? re-estagiou?), NUNCA para afirmar folga/desvio.
        Quem afirma número é a volta no sim (§2H.16/§2H.17).
        """
        cfg = cfg or DoorCrossConfig()
        m = DoorCrossing(cfg)
        x, y, yaw = pose0
        m.update(0.0, (x, y, yaw), portas, goal_active=True, nav_forward=True,
                 gap=3.0, scan_fresh=True, front_gap=3.0, rear_gap=3.0,
                 goal_succeeded=True)
        t, tr, cruz = dt, [], None
        for i in range(int(tmax / dt)):
            c = m.update(t, (x, y, yaw), portas, goal_active=True,
                         nav_forward=True, gap=3.0, scan_fresh=True,
                         front_gap=3.0, rear_gap=3.0)
            if not tr or tr[-1] != c.state:
                tr.append(c.state)
            nx = x + c.vx * math.cos(yaw) * dt
            if x < 7.50 <= nx:
                f = (7.50 - x) / (nx - x)
                cruz = (y + f * c.vx * math.sin(yaw) * dt, math.degrees(yaw))
            x, y = nx, y + c.vx * math.sin(yaw) * dt
            yaw += c.wz * ENTREGA_DO_GIRO * dt
            t += dt
            if c.state == 'idle' and i > 3:
                break
        return dict(cruzou=cruz, transicoes=tr)

    def test_GEOMETRIA_12_cm_ainda_PASSA_no_will_clear_mas_NAO_no_preparo(self):
        """O `will_clear` continua sendo trava de batente, não de preparação.

        A pose +12 cm fora do eixo ainda PASSA na projeção geométrica pura
        (`fit = 0,15 m`), então o ganho desta mudança não vem de apertar a
        conta — vem de exigir preparo na ÁREA da porta antes do commit.
        """
        g = door_geometry((7.50, 1.80), (7.50, 2.70))
        self.assertAlmostEqual(g.half_width - 0.25 - 0.05, 0.15, places=6)
        s, d, yaw_err = -0.60, 0.12, 0.0
        self.assertTrue(will_clear(g, -0.60, 0.12, 0.0, -1, 0.25, 0.05),
                        'o fit geometrico continua permitindo +12 cm')
        self.assertFalse(ready_to_commit(d, yaw_err, DoorCrossConfig()),
                         'o preparo da AREA da porta tem que segurar +12 cm')
        self.assertFalse(will_clear(g, -0.60, 0.26, 0.0, -1, 0.25, 0.05))

    def test_ESTRUTURA_a_pose_ruim_arma_em_staging(self):
        """A área da porta agora tem que comandar a correção antes da boca."""
        portas = carrega_portas()
        m, _ = self._maquina()
        self._pre_porta_cumprido(m, (6.50, 2.37, math.radians(-10.7)), portas)
        cmd = self._tick(m, 0.05, (6.50, 2.37, math.radians(-10.7)), portas)
        self.assertEqual(cmd.state, 'staging',
                         'entrada ruim tem que entrar em staging para matar lateral')

    def test_ESTRUTURA_a_pose_boa_ainda_pode_ir_direto_ao_rotating(self):
        """O par sensível: pose já pronta não deve ganhar um estágio à toa."""
        portas = carrega_portas()
        m, _ = self._maquina()
        self._pre_porta_cumprido(m, (6.50, 2.25, 0.0), portas)
        cmd = self._tick(m, 0.05, (6.50, 2.25, 0.0), portas)
        self.assertEqual(cmd.state, 'rotating')

    def test_AREA_de_aproximacao_filtra_pose_lateral_demais(self):
        """Círculo sozinho era frouxo; fora do corredor a manobra não assume."""
        portas = carrega_portas()
        m, cfg = self._maquina()
        pose = (6.50, 2.70, 0.0)
        self._pre_porta_cumprido(m, pose, portas)
        cmd = self._tick(m, 0.05, pose, portas)
        self.assertEqual(cmd.state, 'idle')
        g = door_geometry(tuple(porta_A(portas)['a']), tuple(porta_A(portas)['b']))
        raw_s = ((pose[0] - g.cx) * g.nx + (pose[1] - g.cy) * g.ny)
        side = -1 if raw_s > 0 else +1
        s, d = door_progress_lateral(g, pose[0], pose[1], side)
        self.assertFalse(in_approach_region(s, d, cfg))

    def test_da_pose_QUE_RASPOU_a_trava_geometrica_REPROVA_e_ela_re_estagia(self):
        """O contraste que fecha o quadro: a 25,9 cm fora do eixo (a amostra do
        1º raspão da `noguard3`, já DENTRO da zona) o `will_clear` reprova e a
        máquina re-estagia. A 12 cm ela commita. O defeito não é ausência de
        trava — é o **limiar** dela ser maior que o que o vão tolera na prática.
        """
        r = self._malha_fechada(self.POSE_TORTA, carrega_portas())
        self.assertIn('staging', r['transicoes'],
                      f'nao re-estagiou: {r["transicoes"]}')
        self.assertIn('crossing', r['transicoes'],
                      f're-estagiou e nunca atravessou: {r["transicoes"]}')

    def test_MODELO_reproduz_a_ESTRUTURA_das_duas_entradas(self):
        """Boa entra quase pronta; ruim usa a área para se ajeitar antes."""
        casos = (
            ((6.50, 2.25, 0.0), 'porta1', ['rotating', 'crossing', 'idle']),
            ((6.50, 2.37, math.radians(-10.7)), 'torta1',
             ['staging', 'rotating', 'crossing', 'idle']),
        )
        for pose, nome, esperado in casos:
            with self.subTest(nome):
                r = self._malha_fechada(pose, carrega_portas())
                self.assertEqual(r['transicoes'], esperado,
                                 f'{nome}: transicoes inesperadas')
                self.assertIsNotNone(r['cruzou'])


class TestGuardaDeConfiguracaoDoAlinhamento(unittest.TestCase):
    """A janela do ROTATING cabe mais de um passo do giro? (guarda de CONFIG)

    ⚠️ Esta classe já foi outra coisa, e o histórico importa (erro 88 da §5 do
    DIARIO_ARENA): ela testava um "ciclo-limite" de 7,16°/tick, calculado como
    `rot_min/rate_hz` — ou seja, **assumindo que o robô atinge o comando**. Isso
    foi **refutado por medição no mesmo dia** (6099 ticks reais: entrega mediana
    0,135 do comandado, |Δyaw| ~1,0°/tick) e depois **em execução**: as voltas
    `porta1` e `torta1` atravessaram a fresta com o WARN aceso e ZERO abort.

    O que sobrou é uma guarda honesta: pergunta se a config é *geometricamente
    impossível*, usando o **pior tick medido** (0,796). Não prevê comportamento.
    """

    def test_a_config_EM_VIGOR_passa(self):
        """O que as 2 voltas mostraram: 3° a 20 Hz funciona. Se este teste
        voltar a reprovar, é sinal de que alguém restaurou a conta antiga."""
        cfg = DoorCrossConfig()
        self.assertTrue(janela_de_alinhamento_ok(cfg.align_yaw, cfg.rot_min, 20.0))
        pior = math.degrees(passo_minimo_do_giro(2.5, 20.0, ENTREGA_DO_GIRO_MAX))
        self.assertAlmostEqual(pior, 5.70, places=2)
        self.assertLess(pior, 2 * math.degrees(cfg.align_yaw))

    def test_a_entrega_do_giro_e_MEDIDA_e_nao_1(self):
        """O erro 88 em forma de teste: se alguém puser a entrega de volta em
        1.0 (o comando integralmente entregue), a conta antiga volta e a guarda
        volta a gritar sem motivo."""
        self.assertLess(ENTREGA_DO_GIRO, 0.3)
        self.assertLess(ENTREGA_DO_GIRO_MAX, 1.0)
        self.assertAlmostEqual(
            math.degrees(passo_minimo_do_giro(2.5, 20.0)), 0.97, places=2)
        # com entrega=1.0 (a conta refutada) a config em vigor REPROVARIA:
        self.assertFalse(janela_de_alinhamento_ok(
            math.radians(3.0), 2.5, 20.0, entrega=1.0))

    def test_config_impossivel_ainda_e_pega(self):
        """O par: se nada reprova, a guarda não vale nada."""
        self.assertFalse(janela_de_alinhamento_ok(math.radians(0.5), 2.5, 20.0))
        self.assertFalse(janela_de_alinhamento_ok(math.radians(3.0), 2.5, 5.0))

    def test_o_criterio_REAGE_aos_quatro_termos(self):
        base = dict(align_yaw=math.radians(3.0), rot_min=2.5, rate_hz=20.0)
        self.assertTrue(janela_de_alinhamento_ok(**base))
        self.assertFalse(janela_de_alinhamento_ok(**{**base, 'align_yaw': math.radians(0.5)}))
        self.assertFalse(janela_de_alinhamento_ok(**{**base, 'rot_min': 20.0}))
        self.assertFalse(janela_de_alinhamento_ok(**{**base, 'rate_hz': 4.0}))
        self.assertFalse(janela_de_alinhamento_ok(**base, entrega=1.0))


if __name__ == '__main__':
    unittest.main()
