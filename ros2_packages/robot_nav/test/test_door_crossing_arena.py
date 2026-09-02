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
import os
import unittest

from robot_nav.door_crossing import (
    DoorCrossConfig,
    DoorCrossing,
    _extrai_doors,
    doors_de_arquivo,
    janela_de_alinhamento_ok,
    passo_minimo_do_giro,
    valida_doors,
)

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DOORS_JSON = os.path.join(RAIZ, 'maps', 'arena_galpao.doors.json')

# Geometria da fresta A, como o gerador a produz (tools/gera_arena_galpao.py:43):
# blocos em x = 7,5 cobrindo y 0,30-1,80 e 2,70-4,20 -> vão de 0,90 m.
A_ESPERADA = {'a': (7.50, 1.80), 'b': (7.50, 2.70)}


def carrega_portas():
    with open(DOORS_JSON) as f:
        return json.load(f)['doors']


def porta_A(portas):
    """A fresta A é a porta cujo centro cai no vão medido, com tolerância de
    meia célula do mapa (0,05 m). Procurar pelo id seria frágil."""
    for d in portas:
        cx = (d['a'][0] + d['b'][0]) / 2.0
        cy = (d['a'][1] + d['b'][1]) / 2.0
        if abs(cx - 7.50) <= 0.05 and abs(cy - 2.25) <= 0.05:
            return d
    raise AssertionError(
        f'nenhuma porta no vão da fresta A (7.50 ; 2.25) em {DOORS_JSON}: {portas}')


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

    def test_a_fresta_C_de_0_60_NAO_pode_estar_marcada(self):
        """§5.1/§6.2: com robot_radius 0.32 o Nav2 já trata a de 0,60 como
        parede, e a conta do §4.4-(a) provavelmente não fecha lá. Marcar porta
        que o planejador não usa é zona armada à toa."""
        for d in carrega_portas():
            vao = math.hypot(d['b'][0] - d['a'][0], d['b'][1] - d['a'][1])
            self.assertGreater(
                vao, 0.65,
                f'porta de {vao:.2f} m marcada — a de 0,60 não pode ser marcada '
                'sem refazer a conta do §4.4-(a): id={}'.format(d.get('id')))


class TestDoorsFile(unittest.TestCase):
    """§5.2 — o nó tem que conseguir carregar porta do DISCO.

    `/doors` só é publicado pelo `controle_web`, e o harness A/B do sim não sobe
    o stack web. Sem isto o nó sobe, não recebe porta, fica idle, e a volta roda
    idêntica à de hoje — com a diferença de que eu poderia achar que testei."""

    def test_le_a_porta_da_arena_do_disco(self):
        d = doors_de_arquivo(DOORS_JSON)
        self.assertEqual(len(d), 1)
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
        self.assertNotAlmostEqual(cmd.wz, 0.0, msg='tinha que estar girando '
                                  'pra tirar os 7,7° antes de entrar')

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
        """Roda a máquina em MALHA FECHADA: integra o (vx, wz) que ela comanda
        de volta na pose. Sem isso o teste segura o robô parado e mede a
        máquina brigando com uma pose que ela mandou mudar (foi o defeito da
        1ª versão deste teste).

        Cinemática ideal, de propósito: sem inércia, sem collision_monitor, sem
        derrapagem. Serve para achar defeito de LÓGICA — um resultado limpo aqui
        NÃO substitui a volta no sim.
        """
        cfg = cfg or DoorCrossConfig()
        m = DoorCrossing(cfg)
        x, y, yaw = pose0
        m.update(0.0, (x, y, yaw), portas, goal_active=True, nav_forward=True,
                 gap=3.0, scan_fresh=True, front_gap=3.0, rear_gap=3.0,
                 goal_succeeded=True)
        t, tr, cruz, reest = dt, [], None, 0
        for i in range(int(tmax / dt)):
            c = m.update(t, (x, y, yaw), portas, goal_active=True,
                         nav_forward=True, gap=3.0, scan_fresh=True,
                         front_gap=3.0, rear_gap=3.0)
            if not tr or tr[-1] != c.state:
                tr.append(c.state)
                if c.state == 'staging':
                    reest += 1
            nx = x + c.vx * math.cos(yaw) * dt
            if x < 7.50 <= nx:                      # interpola o plano dos batentes
                f = (7.50 - x) / (nx - x)
                cruz = (y + f * c.vx * math.sin(yaw) * dt, math.degrees(yaw))
            x, y, yaw = nx, y + c.vx * math.sin(yaw) * dt, yaw + c.wz * dt
            t += dt
            if c.state == 'idle' and i > 3:
                break
        return dict(cruzou=cruz, transicoes=tr, reestagios=reest, t=t)

    def test_da_pose_QUE_RASPOU_ela_re_estagia_e_atravessa_centrada(self):
        """O que a máquina compra, na pose exata do 1º raspão da `noguard3`.

        Ela NÃO entra com os 25,9 cm de desvio — o `will_clear` reprova
        (fit = 0,45 − 0,25 − 0,05 = 0,15 m), ela dá ré, re-estagia no eixo e
        só então atravessa. É o único re-estágio permitido pelo §4.10-item-3."""
        r = self._malha_fechada(self.POSE_TORTA, carrega_portas())
        self.assertIsNotNone(r['cruzou'],
                             f'nunca atravessou: {r["transicoes"]}')
        y, yawg = r['cruzou']
        desvio, folga = abs(y - 2.25), 0.45 - abs(y - 2.25) - (
            0.25 * math.cos(math.radians(abs(yawg)))
            + 0.25 * math.sin(math.radians(abs(yawg))))
        self.assertLessEqual(desvio, 0.08,
                             f'entrou {desvio*100:.1f} cm fora do eixo')
        self.assertLessEqual(abs(yawg), 3.0,
                             f'entrou {yawg:.1f}° torta')
        # hoje, sem a máquina, esta volta entrou com 12,1 cm / -10,7° e folga 3,7 cm
        self.assertGreater(folga, 0.10,
                           f'folga {folga*100:.1f} cm — tinha que ser MUITO '
                           'melhor que os 3,7 cm que rasparam')
        self.assertLessEqual(r['reestagios'], 1,
                             f'thrash de re-estágio: {r["transicoes"]}')

    def test_com_a_janela_em_vigor_ela_ABORTA_de_algumas_poses(self):
        """O defeito do §2H.11, medido em malha fechada a partir de uma pose
        REAL de entrada na zona (`aprox2`, yaw -25,8°): com align_yaw 3° a 20 Hz
        o giro pula por cima da janela e a travessia morre no align_timeout."""
        r = self._malha_fechada((6.452, 2.549, math.radians(-25.8)),
                                carrega_portas())
        self.assertIsNone(r['cruzou'],
                          'se isto passou a atravessar, o ciclo-limite foi '
                          'consertado — atualize o §2H.11 e este teste')
        self.assertIn('idle', r['transicoes'])

    def test_com_a_janela_CORRIGIDA_a_mesma_pose_atravessa(self):
        """O par que prova que a causa é a janela, e não a pose: só mudando
        align_yaw 3° -> 5° (nada mais), a mesma entrada atravessa."""
        cfg = dataclasses.replace(DoorCrossConfig(),
                                  align_yaw=math.radians(5.0))
        r = self._malha_fechada((6.452, 2.549, math.radians(-25.8)),
                                carrega_portas(), cfg=cfg)
        self.assertIsNotNone(r['cruzou'],
                             f'não atravessou nem com 5°: {r["transicoes"]}')
        y, yawg = r['cruzou']
        self.assertLessEqual(abs(yawg), 3.0,
                             f'entrou {yawg:.1f}° — alargar a janela do ROTATING '
                             'não pode piorar o yaw DE ENTRADA (o cross_k_yaw '
                             'fecha a malha durante a travessia)')


class TestCicloLimiteDoAlinhamento(unittest.TestCase):
    """A janela do ROTATING tem que caber mais de um passo do giro (§2H.11).

    `rot_min` é PISO (abaixo dele o skid-steer não vira), então o giro no lugar
    é quantizado em `rot_min/rate_hz`. Com os valores em vigor a janela mede
    6,00° e o passo 7,16°: o giro pula por cima da janela inteira toda vez.
    """

    def test_a_config_EM_VIGOR_reprova_a_20_Hz(self):
        """Este é o defeito, escrito como teste. Medido nas 13 poses reais de
        entrada na fresta A: 3 abortaram por align_timeout, e os 10 que
        passaram passaram por FASE, não por mecanismo."""
        cfg = DoorCrossConfig()
        self.assertFalse(janela_de_alinhamento_ok(cfg.align_yaw, cfg.rot_min, 20.0))
        self.assertAlmostEqual(
            math.degrees(passo_minimo_do_giro(cfg.rot_min, 20.0)), 7.16, places=2)
        self.assertAlmostEqual(2 * math.degrees(cfg.align_yaw), 6.00, places=2)

    def test_as_duas_saidas_medidas_aprovam(self):
        """As duas que dão 13/13 nas 13 poses reais: alargar a janela ou subir
        a taxa. Alargar não custa precisão — o yaw NA ENTRADA fica em 1,51° nos
        dois casos, porque o cross_k_yaw fecha a malha durante a travessia."""
        cfg = DoorCrossConfig()
        self.assertTrue(janela_de_alinhamento_ok(math.radians(5.0), cfg.rot_min, 20.0))
        self.assertTrue(janela_de_alinhamento_ok(cfg.align_yaw, cfg.rot_min, 50.0))

    def test_o_criterio_REAGE_aos_tres_termos(self):
        """O par sensível: um critério que não reage a algum dos três termos
        passaria com a conta errada (foi assim que o defeito ficou 2 meses)."""
        base = dict(align_yaw=math.radians(5.0), rot_min=2.5, rate_hz=20.0)
        self.assertTrue(janela_de_alinhamento_ok(**base))
        self.assertFalse(janela_de_alinhamento_ok(
            **{**base, 'align_yaw': math.radians(1.0)}))   # janela menor
        self.assertFalse(janela_de_alinhamento_ok(**{**base, 'rot_min': 8.0}))
        self.assertFalse(janela_de_alinhamento_ok(**{**base, 'rate_hz': 5.0}))


if __name__ == '__main__':
    unittest.main()
