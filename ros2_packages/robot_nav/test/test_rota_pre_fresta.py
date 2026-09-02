"""Waypoint pré-fresta A — o goal que ARMA o `door_crossing` (§2H.7).

Sem ele o `_pick_door` nunca libera a porta (pendência C) e o nó fica `idle`
para sempre. Com ele, o robô chega ao ponto **parado e centrado** pelo
`xy_goal_tolerance`, que é o contexto para o qual `zone_radius = 1.1` foi
calibrado — hoje ele entraria na zona a ~0,9 m/s (risco §8.1 do spec).

⚠️ OPT-IN: enquanto o dono não decidir, a rota da prova sai IDÊNTICA. O teste
`test_o_default_NAO_muda_a_rota_da_prova` é o que garante isso.
"""
import json
import math
import os
import sys
import unittest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(RAIZ, 'tools'))
import gera_arena_galpao as ga                                    # noqa: E402

ROTA = os.path.join(RAIZ, 'maps', 'routes', 'arena_galpao.json')


class TestRotaPreFresta(unittest.TestCase):

    def test_o_default_NAO_muda_a_rota_da_prova(self):
        """A 3 dias da prova, mexer na rota é decisão do dono. Enquanto ela não
        vem, o gerador tem que produzir EXATAMENTE a rota commitada."""
        with open(ROTA) as f:
            commitada = json.load(f)['waypoints']
        self.assertEqual(ga.rota_waypoints(), commitada,
                         'a rota da prova mudou sem ninguém pedir')

    def test_o_default_nao_tem_o_waypoint(self):
        self.assertNotIn('pre_fresta_A',
                         [w['alvo'] for w in ga.rota_waypoints()])

    def test_com_pre_fresta_o_waypoint_entra_ANTES_do_cone_2(self):
        """A perna cone_1 -> cone_2 é a que atravessa a fresta A; depois do
        cone_2 o goal já passou do vão e não arma nada."""
        alvos = [w['alvo'] for w in ga.rota_waypoints(pre_fresta=True)]
        self.assertIn('pre_fresta_A', alvos)
        self.assertLess(alvos.index('pre_fresta_A'), alvos.index('cone_2'))
        self.assertGreater(alvos.index('pre_fresta_A'), alvos.index('cone_1'))

    def test_so_acrescenta_um_ponto_e_nao_mexe_nos_outros(self):
        """O par: inserir não pode reescrever os goals que já estavam medidos."""
        base = ga.rota_waypoints()
        com = ga.rota_waypoints(pre_fresta=True)
        self.assertEqual(len(com), len(base) + 1)
        self.assertEqual([w for w in com if w['alvo'] != 'pre_fresta_A'], base)

    def test_o_waypoint_esta_DENTRO_da_zona_do_door_crossing(self):
        """`zone_radius = 1.1` (door_crossing.py:174). Fora dela não arma."""
        x, y, _ = ga.ponto_pre_fresta()
        (ax, ay), (bx, by) = ga.batentes('A_fresta90')
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        self.assertLess(math.hypot(x - cx, y - cy), 1.1)

    def test_o_yaw_APONTA_pro_vao(self):
        """Achado do review: `approach_bearing = 70°` é medido do yaw do ROBÔ —
        waypoint com yaw errado reprova o gate mesmo dentro da zona."""
        x, y, yaw = ga.ponto_pre_fresta()
        (ax, ay), (bx, by) = ga.batentes('A_fresta90')
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        rumo = math.atan2(cy - y, cx - x)
        erro = abs(math.atan2(math.sin(yaw - rumo), math.cos(yaw - rumo)))
        self.assertLess(math.degrees(erro), 70.0)
        self.assertLess(math.degrees(erro), 1.0, 'devia apontar direto pro vão')

    def test_a_margem_do_point_turn_usa_o_ENVELOPE_e_nao_o_ponto_ideal(self):
        """O erro que este teste existe pra impedir é o do §4.4-(a) do spec:
        conta feita no ponto exato, decisão tomada como se o robô parasse lá.
        A 0,6 m (o valor do spec) o pior caso é NEGATIVO."""
        self.assertLess(ga.margem_pre_fresta(dist=0.6), 0.0)
        self.assertGreater(ga.margem_pre_fresta(dist=1.0), 0.20)
        # e a margem tem que CRESCER com a distância (senão a conta não reage)
        self.assertGreater(ga.margem_pre_fresta(dist=1.0),
                           ga.margem_pre_fresta(dist=0.8))

    def test_distancia_curta_demais_ABORTA_a_geracao(self):
        """Falha fechada: rota que põe o robô girando em cima do batente não
        pode ser gerada em silêncio."""
        orig = ga.PRE_FRESTA_DIST
        try:
            ga.PRE_FRESTA_DIST = 0.5
            with self.assertRaises(SystemExit):
                ga.rota_waypoints(pre_fresta=True)
        finally:
            ga.PRE_FRESTA_DIST = orig

    def test_o_ponto_sai_da_tabela_OBST_e_nao_de_numero_chapado(self):
        """Se alguém mover a fresta na OBST, o waypoint tem que se mover junto —
        senão vira goal no meio do muro."""
        x, y, _ = ga.ponto_pre_fresta()
        self.assertAlmostEqual(x, 6.50, places=3)
        self.assertAlmostEqual(y, 2.25, places=3)
        orig = ga.OBST[0]
        try:
            ga.OBST[0] = ('A_fresta90', 'x', 9.0, [(0.30, 1.80), (2.70, 4.20)],
                          0.90, 'x')
            x2, _, _ = ga.ponto_pre_fresta()
            self.assertAlmostEqual(x2, 8.00, places=3)
        finally:
            ga.OBST[0] = orig


if __name__ == '__main__':
    unittest.main()
