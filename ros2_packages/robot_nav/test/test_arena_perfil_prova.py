#!/usr/bin/env python3
"""O perfil `--arena` é a PROVA de 05/09 inteira, e o mapa dela é o TAMPADO.

Por que existe (2026-09-01, decisão do dono — DIARIO_ARENA §2G): a rede de
segurança da prova é **não usar a fresta A**. Isso depende de três coisas que
não moram no mesmo arquivo e podem se desencontrar em silêncio:

  1. o `maps/arena_galpao_semA.*` estar **no git** (a Pi deploya por
     `git reset --hard` e NÃO roda o gerador);
  2. o `.pgm` commitado ser mesmo o que o gerador produz **hoje** (mapa velho no
     git = robô mandado por um vão que o mapa novo fecharia);
  3. o `launch.sh --arena` carregar esse mapa, e não o aberto.

Um desencontro em qualquer um dos três manda o robô pela fresta de 0,90 m —
onde 1 volta em 3 bateu (9 COLISÃO + 48 raspões, §2B.9). Os testes são
SENSÍVEIS por par: cada afirmação sobre o mapa tampado tem a afirmação oposta
sobre o mapa aberto ao lado, então uma implementação que confunda os dois
reprova.
"""
import importlib.util
import os
import subprocess
import unittest

import yaml

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
MAPAS = os.path.join(RAIZ, 'maps')
CFG = os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'config')
MARCA_INI = '# >>> PERFIL_ARENA_DEFAULTS'
MARCA_FIM = '# <<< PERFIL_ARENA_DEFAULTS'

_spec = importlib.util.spec_from_file_location(
    'gera_arena_prova', os.path.join(RAIZ, 'tools', 'gera_arena_galpao.py'))
ga = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ga)


def _rastreados():
    out = subprocess.run(['git', 'ls-files'], cwd=RAIZ,
                         capture_output=True, text=True).stdout
    return set(out.split('\n'))


def _pgm(caminho):
    with open(caminho, 'rb') as f:
        return f.read()


def _gera(tmp, fecha=()):
    """Roda o gerador de verdade num diretório temporário."""
    pgm = os.path.join(tmp, 'x.pgm')
    ga.gera_mapa(pgm, os.path.join(tmp, 'x.yaml'), fecha=fecha)
    return _pgm(pgm)


class TestMapaDaProvaVersionado(unittest.TestCase):

    def test_o_mapa_tampado_esta_no_git(self):
        """Sem isto o mapa some num clone limpo — e a Pi É um clone limpo."""
        trk = _rastreados()
        for nome in ('maps/arena_galpao_semA.pgm', 'maps/arena_galpao_semA.yaml'):
            self.assertIn(nome, trk,
                          '%s tem que estar no git: a Pi deploya por '
                          '`git reset --hard` e nao roda o gerador' % nome)

    def test_o_pgm_commitado_e_o_que_o_gerador_produz_HOJE(self):
        """Mapa velho no git é a falha silenciosa: tudo existe, tudo carrega, e
        o vão que o gerador fecharia continua aberto pro planejador."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            esperado = _gera(tmp, fecha=('A_fresta90',))
        self.assertEqual(
            _pgm(os.path.join(MAPAS, 'arena_galpao_semA.pgm')), esperado,
            'maps/arena_galpao_semA.pgm != saida do gerador. Regere com: '
            'python3 tools/gera_arena_galpao.py --mapa maps/ --fecha-fresta A')

    def test_o_mapa_OFICIAL_continua_com_a_fresta_ABERTA(self):
        """O par do teste acima. Se o tampão vazasse para o `arena_galpao.pgm`,
        os dois arquivos ficariam iguais e ninguém notaria — mas aí o mundo e o
        mapa deixariam de ser comparáveis, e o experimento inteiro muda."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            aberto = _gera(tmp)
        self.assertEqual(_pgm(os.path.join(MAPAS, 'arena_galpao.pgm')), aberto)
        self.assertNotEqual(_pgm(os.path.join(MAPAS, 'arena_galpao.pgm')),
                            _pgm(os.path.join(MAPAS, 'arena_galpao_semA.pgm')),
                            'os dois mapas nao podem ser iguais: um fecha a '
                            'fresta A e o outro nao')

    def test_o_yaml_tampado_aponta_pro_pgm_tampado(self):
        with open(os.path.join(MAPAS, 'arena_galpao_semA.yaml')) as f:
            y = yaml.safe_load(f)
        self.assertEqual(y['image'], 'arena_galpao_semA.pgm')


class TestLaunchArena(unittest.TestCase):
    """Executa o BLOCO REAL do launch.sh (entre os marcadores), não uma cópia:
    reconstruir a lógica aqui seria tautologia (BO 63)."""

    def _roda(self, arena, explicito=False):
        with open(os.path.join(RAIZ, 'launch.sh')) as f:
            texto = f.read()
        bloco = texto.split(MARCA_INI)[1].split(MARCA_FIM)[0]
        script = (
            'SCRIPT_DIR=%s\nARENA=%s\n'
            'MAP_EXPLICITO=%s\nWORLD_EXPLICITO=%s\nSPAWN_EXPLICITO=%s\n'
            'MAP_FILE=$SCRIPT_DIR/maps/hotmilk_portas.yaml\n'
            'WORLD_FILE=$SCRIPT_DIR/worlds/sala.sdf\n'
            'SPAWN_X=2.0\nSPAWN_Y=2.5\n'
            % (RAIZ, 'true' if arena else 'false',
               *(['true'] * 3 if explicito else ['false'] * 3))
        ) + bloco + '\necho "$MAP_FILE|$WORLD_FILE|$SPAWN_X|$SPAWN_Y"\n'
        r = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return r.stdout.strip().split('\n')[-1].split('|')

    def test_arena_carrega_o_mapa_TAMPADO(self):
        mapa, mundo, sx, sy = self._roda(arena=True)
        self.assertTrue(mapa.endswith('maps/arena_galpao_semA.yaml'), mapa)
        self.assertTrue(mundo.endswith('worlds/arena_galpao.sdf'), mundo)
        self.assertEqual((sx, sy), ('1.0', '1.0'))

    def test_sem_arena_nada_muda(self):
        """O par: fora da arena os defaults antigos continuam intactos."""
        mapa, mundo, sx, sy = self._roda(arena=False)
        self.assertTrue(mapa.endswith('maps/hotmilk_portas.yaml'), mapa)
        self.assertTrue(mundo.endswith('worlds/sala.sdf'), mundo)
        self.assertEqual((sx, sy), ('2.0', '2.5'))

    def test_o_que_o_operador_passa_na_mao_VENCE_o_arena(self):
        """Rodar PELA fresta (ou noutro mundo) tem que continuar possível."""
        mapa, mundo, sx, sy = self._roda(arena=True, explicito=True)
        self.assertTrue(mapa.endswith('maps/hotmilk_portas.yaml'), mapa)
        self.assertTrue(mundo.endswith('worlds/sala.sdf'), mundo)
        self.assertEqual((sx, sy), ('2.0', '2.5'))


class TestAckDoPlanner(unittest.TestCase):
    """Item 2l: o 1o goal da `contornoA3` morreu no ack do compute_path_to_pose."""

    def _p(self, nome):
        with open(os.path.join(CFG, nome)) as f:
            return yaml.safe_load(f)['bt_navigator']['ros__parameters']

    def test_arena_espera_pelo_menos_1s_pelo_ack(self):
        self.assertGreaterEqual(self._p('nav2_params_arena.yaml')
                                ['default_server_timeout'], 1000)

    def test_o_perfil_PI_nao_foi_junto(self):
        """O par: a mudança é do perfil da PROVA, não global. Se alguém subir o
        `pi` junto, que seja por decisão, com este teste falhando primeiro."""
        self.assertEqual(self._p('nav2_params_pi.yaml')
                         ['default_server_timeout'], 200)


if __name__ == '__main__':
    unittest.main()
