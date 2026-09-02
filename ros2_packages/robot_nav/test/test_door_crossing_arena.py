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

import json
import math
import os
import unittest

from robot_nav.door_crossing import (
    DoorCrossConfig,
    DoorCrossing,
    _extrai_doors,
    doors_de_arquivo,
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

    def _tick(self, m, t, pose, portas):
        return m.update(t, pose, portas, goal_active=True, nav_forward=True,
                        gap=3.0, scan_fresh=True, front_gap=3.0, rear_gap=3.0)

    def test_pose_torta_NAO_atravessa_direto(self):
        portas = carrega_portas()
        m, cfg = self._maquina()
        cmd = self._tick(m, 0.0, self.POSE_TORTA, portas)
        self.assertNotEqual(cmd.state, 'crossing',
                            'entrou torta (lat 0,259 m) — é o raspão da noguard3')
        self.assertNotEqual(cmd.state, 'idle',
                            'a máquina não pegou a porta: com ela idle a volta '
                            'roda igual à de hoje')
        self.assertLessEqual(abs(cmd.vx), cfg.stage_speed + 1e-9,
                             'avançou em velocidade de rota com a porta armada')

    def test_pose_boa_ATRAVESSA_depois_de_estabilizar(self):
        """O par que prova o teste sensível: sem ele, a máquina desligada
        (sempre idle) passaria no teste de cima."""
        portas = carrega_portas()
        m, cfg = self._maquina()
        estados = []
        for i in range(cfg.align_stable + 3):
            estados.append(self._tick(m, i * 0.05, self.POSE_BOA, portas).state)
        self.assertIn('crossing', estados,
                      f'no eixo e alinhada, nunca commitou: {estados}')


if __name__ == '__main__':
    unittest.main()
