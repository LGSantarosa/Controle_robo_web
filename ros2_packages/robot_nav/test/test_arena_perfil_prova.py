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
import json
import math
import os
import subprocess
import sys
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

    def _roda(self, arena, explicito=False, fecha_fresta=False):
        with open(os.path.join(RAIZ, 'launch.sh')) as f:
            texto = f.read()
        bloco = texto.split(MARCA_INI)[1].split(MARCA_FIM)[0]
        script = (
            'SCRIPT_DIR=%s\nARENA=%s\nFECHA_FRESTA=%s\n'
            'MAP_EXPLICITO=%s\nWORLD_EXPLICITO=%s\nSPAWN_EXPLICITO=%s\n'
            'MAP_FILE=$SCRIPT_DIR/maps/hotmilk_portas.yaml\n'
            'WORLD_FILE=$SCRIPT_DIR/worlds/sala.sdf\n'
            'SPAWN_X=2.0\nSPAWN_Y=2.5\n'
            % (RAIZ, 'true' if arena else 'false',
               'true' if fecha_fresta else 'false',
               *(['true'] * 3 if explicito else ['false'] * 3))
        ) + bloco + '\necho "$MAP_FILE|$WORLD_FILE|$SPAWN_X|$SPAWN_Y"\n'
        r = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return r.stdout.strip().split('\n')[-1].split('|')

    def test_arena_carrega_o_mapa_ABERTO(self):
        """2026-09-02 (§2G.10): o default do --arena voltou a ser o mapa ABERTO
        — a fresta A existe pro planejador e o robô PASSA. Quem controla a
        travessia é o door_crossing, não o mapa."""
        mapa, mundo, sx, sy = self._roda(arena=True)
        self.assertTrue(mapa.endswith('maps/arena_galpao.yaml'), mapa)
        self.assertFalse(mapa.endswith('semA.yaml'), mapa)
        self.assertTrue(mundo.endswith('worlds/arena_galpao.sdf'), mundo)
        self.assertEqual((sx, sy), ('1.0', '1.0'))

    def test_fecha_fresta_e_o_BOTAO_DE_PANICO(self):
        """O par do teste acima: --fecha-fresta troca SÓ o mapa pelo tampado
        (mundo e spawn seguem os da prova). É a rede de segurança da véspera."""
        mapa, mundo, sx, sy = self._roda(arena=True, fecha_fresta=True)
        self.assertTrue(mapa.endswith('maps/arena_galpao_semA.yaml'), mapa)
        self.assertTrue(mundo.endswith('worlds/arena_galpao.sdf'), mundo)
        self.assertEqual((sx, sy), ('1.0', '1.0'))

    def test_fecha_fresta_SEM_arena_nao_faz_nada(self):
        """Fora do perfil da arena a flag é inerte — não pode sequestrar o mapa
        de quem roda a sala normal."""
        mapa, _, _, _ = self._roda(arena=False, fecha_fresta=True)
        self.assertTrue(mapa.endswith('maps/hotmilk_portas.yaml'), mapa)

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


class TestMargemDoPointTurn(unittest.TestCase):
    """O standoff tem que caber o GIRO NO LUGAR, não só o goal.

    Por que existe (2026-09-01, §2G.8): com STANDOFF 1,0 m a `nominal1` raspou o
    `cone_2` 18 vezes. O seguidor conclui o goal e gira no lugar pra encarar o
    próximo; nesse giro o canto do robô varre `hypot(0,25; 0,25) = 0,354 m` e o
    cone ocupa 0,17. Sobrava 0,477 m de margem — e o erro de pose do AMCL nesta
    arena chega a 0,45 m (item 2c). Margem e erro do mesmo tamanho = contato por
    sorteio.

    Isto NÃO testa o defeito (item 1: o giro segue cego ao anel). Testa a
    MITIGAÇÃO, que é o único ponto onde ela mora: a tabela da rota.
    """

    VARRE = math.hypot(0.25, 0.25)   # canto do footprint 0,5 x 0,5
    R_CONE = 0.17
    PIOR_ERRO_AMCL = 0.45            # máx medido na arena (§2G.3, §2G.8)

    def _rota(self):
        with open(os.path.join(RAIZ, 'maps', 'routes',
                               'arena_galpao.json')) as f:
            return json.load(f)['waypoints']

    def _margem(self, w):
        cx, cy, _tem = ga.PONTOS[w['alvo']]
        d = math.hypot(w['x'] - cx, w['y'] - cy)
        return d - self.VARRE - self.R_CONE

    def test_todo_standoff_cabe_o_point_turn_com_folga(self):
        wps = [w for w in self._rota() if ga.PONTOS[w['alvo']][2]]
        self.assertEqual(len(wps), 4, 'a rota tem que ter os 4 cones')
        for w in wps:
            self.assertGreater(
                self._margem(w), self.PIOR_ERRO_AMCL + 0.30,
                '%s: margem de point-turn %.3f m nao cobre o pior erro de pose '
                'medido (%.2f m) com sobra' % (w['alvo'], self._margem(w),
                                               self.PIOR_ERRO_AMCL))

    def test_o_standoff_ANTIGO_reprovaria(self):
        """O par: prova que o teste acima é sensível ao número, e não passa por
        acidente. Com 1,0 m a margem era 0,4764 — menor que o pior erro + 0,30."""
        margem_antiga = 1.0 - self.VARRE - self.R_CONE
        self.assertLess(margem_antiga, self.PIOR_ERRO_AMCL + 0.30)
        self.assertAlmostEqual(margem_antiga, 0.4764, places=4)

    def test_a_rota_commitada_e_a_que_o_gerador_produz(self):
        """Rota editada na mão é a falha silenciosa: o `--conferir` aprova
        qualquer goal navegável, inclusive um colado no cone."""
        self.assertEqual(self._rota(), ga.rota_waypoints())


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


MARCA_DOOR_INI, MARCA_DOOR_FIM = '>>> PERFIL_ARENA_DOOR', '<<< PERFIL_ARENA_DOOR'


class TestLaunchArenaDoorCrossing(unittest.TestCase):
    """O `--arena` tem que LIGAR o door_crossing e apontar o doors.json do mapa
    QUE ELE CARREGOU (2026-09-02, spec §5.3).

    Por que o pareamento importa: se o operador aperta `--fecha-fresta`, o mapa
    trata a fresta A como parede — e armar uma travessia ali seria mandar o robô
    atravessar o que o planejador acha que é muro. Mapa e portas saem do mesmo
    comando do gerador justamente para não divergirem.

    Executa o BLOCO REAL do launch.sh entre os marcadores, não uma cópia (BO 63).
    """

    def _roda(self, arena, mapa):
        with open(os.path.join(RAIZ, 'launch.sh')) as f:
            texto = f.read()
        bloco = texto.split(MARCA_DOOR_INI)[1].split(MARCA_DOOR_FIM)[0]
        script = (f'SCRIPT_DIR={RAIZ}\n'
                  f'ARENA={"true" if arena else "false"}\n'
                  f'MAP_FILE={mapa}\n' + bloco
                  + '\necho "ARG=$ARENA_DOOR_ARG"\n')
        r = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        return r.returncode, r.stdout + r.stderr

    def test_arena_com_mapa_ABERTO_liga_e_aponta_a_porta(self):
        rc, out = self._roda(True, os.path.join(MAPAS, 'arena_galpao.yaml'))
        self.assertEqual(rc, 0, out)
        self.assertIn('door_crossing:=true', out)
        self.assertIn(
            'doors_file:=' + os.path.join(MAPAS, 'arena_galpao.doors.json'),
            out)

    def test_arena_com_o_mapa_TAMPADO_aponta_o_doors_do_TAMPADO(self):
        """O par: com --fecha-fresta o doors.json do tampado é o de lista VAZIA
        (o gerador o escreve assim), então o nó sobe e não arma nada — que é o
        certo, porque ali a fresta é parede pro planejador."""
        rc, out = self._roda(True, os.path.join(MAPAS, 'arena_galpao_semA.yaml'))
        self.assertEqual(rc, 0, out)
        self.assertIn('arena_galpao_semA.doors.json', out)
        self.assertNotIn('arena_galpao.doors.json:', out)

    def test_sem_arena_nao_liga_nada(self):
        rc, out = self._roda(False, os.path.join(MAPAS, 'hotmilk_portas.yaml'))
        self.assertEqual(rc, 0, out)
        self.assertIn('ARG=', out)
        self.assertNotIn('door_crossing:=true', out)

    def test_FALHA_FECHADA_sem_o_doors_json(self):
        """Sem o arquivo o nó subiria e ficaria `idle` para sempre — o robô
        atravessaria a fresta sem ninguém dirigindo, que é o caso que bateu
        (noguard3). Tem que ABORTAR, não seguir em silêncio."""
        rc, out = self._roda(True, os.path.join(MAPAS, 'nao_existe.yaml'))
        self.assertNotEqual(rc, 0, 'tinha que abortar: ' + out)
        self.assertIn('portas do mapa nao existem', out)

    def test_o_doors_json_da_arena_esta_NO_GIT(self):
        """Igual ao mapa: a Pi deploya por `git reset --hard` e não roda o
        gerador — sem o artefato versionado o launch aborta lá (falha fechada
        que eu acabei de escrever) e ninguém sabe por quê.

        ⚠️ `maps/*` está no .gitignore (`.gitignore:15`), então estes arquivos
        só entram com `git add -f`. A 1ª versão deste teste checava
        `os.path.exists` e passava com o arquivo FORA do git — que é exatamente
        o estado que quebra a Pi."""
        trk = _rastreados()
        for nome in ('maps/arena_galpao.doors.json',
                     'maps/arena_galpao_semA.doors.json'):
            self.assertIn(nome, trk,
                          '%s tem que estar no git (maps/* e ignorado: use '
                          '`git add -f`)' % nome)

    def test_o_doors_json_commitado_e_o_que_o_gerador_produz_HOJE(self):
        """O par do teste do .pgm: portas velhas no git = o robô arma travessia
        num eixo que o mundo não tem mais."""
        sys.path.insert(0, os.path.join(RAIZ, 'tools'))
        import gera_arena_galpao as ga
        with open(os.path.join(MAPAS, 'arena_galpao.doors.json')) as f:
            self.assertEqual(json.load(f)['doors'], ga.portas(),
                             'maps/arena_galpao.doors.json != saida do gerador. '
                             'Regere com: python3 tools/gera_arena_galpao.py '
                             '--mapa maps/')


class TestLaunchArenaDoorsInvalido(unittest.TestCase):
    """O `--arena` valida o CONTEÚDO do doors.json, não só a existência.

    Decisão de 2026-09-02 (achado do review): validar dentro do nó não protege —
    nó `idle` e nó MORTO dão no mesmo, ninguém dirige a travessia. Quem protege é
    **não subir a stack**, e isso só dá para fazer aqui, antes do launch, com o
    erro na tela do operador em vez de enterrado no nav2.log.

    A validação chama a MESMA função que o nó usa (`doors_de_arquivo`); nada de
    reimplementar o schema no shell e ter duas fontes de verdade divergindo.
    """

    def _com_doors(self, conteudo):
        """Monta um par <mapa>.yaml / <mapa>.doors.json temporário e roda o
        bloco real do launch.sh em cima dele."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            mapa = os.path.join(tmp, 'x.yaml')
            with open(mapa, 'w') as f:
                f.write('image: x.pgm\n')
            if conteudo is not None:
                with open(os.path.join(tmp, 'x.doors.json'), 'w') as f:
                    f.write(conteudo)
            return TestLaunchArenaDoorCrossing._roda(self, True, mapa)

    def test_json_corrompido_ABORTA(self):
        rc, out = self._com_doors('{ nao e json')
        self.assertEqual(rc, 1, out)
        self.assertIn('nao prestam', out)

    def test_chave_doors_AUSENTE_aborta(self):
        """O caso silencioso: JSON válido, chave errada. Antes disso virava
        'zero portas' sem um pio — e zero portas = fresta sem ninguém dirigindo."""
        rc, out = self._com_doors('{"portas": []}')
        self.assertEqual(rc, 1, out)
        self.assertIn('sem a chave', out)

    def test_porta_malformada_aborta(self):
        rc, out = self._com_doors('{"doors": [{"id": 1, "a": [1.0, 2.0]}]}')
        self.assertEqual(rc, 1, out)

    def test_arquivo_ausente_aborta(self):
        rc, out = self._com_doors(None)
        self.assertEqual(rc, 1, out)
        self.assertIn('nao existem', out)

    def test_lista_VAZIA_explicita_e_legitima(self):
        """O par sensível: é o que o gerador escreve para o mapa tampado
        (`--fecha-fresta`), onde a fresta é parede e armar seria errado. Se este
        teste falhar, a falha fechada virou paranoia e quebrou o botão de pânico."""
        rc, out = self._com_doors('{"doors": []}')
        self.assertEqual(rc, 0, out)
        self.assertIn('door_crossing:=true', out)


MARCA_FOLLOW_INI = '# >>> PERFIL_ARENA_FOLLOW'
MARCA_FOLLOW_FIM = '# <<< PERFIL_ARENA_FOLLOW'


class TestFollowForwardSpeed(unittest.TestCase):
    """O teto EFETIVO da autonomia é o `forward_speed` do path_follower, não o
    `max_vel_x` do nav2_params: o follower publica `follow_vel` com prio 15 no
    `twist_mux_auto` e o `nav_vel` (DWB/RotationShim/smoother) tem 10 — a cadeia
    do controller_server PERDE o mux enquanto o follower publicar.

    Em 2026-09-05 uma análise minha propôs subir os três tetos do YAML (a cadeia
    perdedora) e o dono derrubou na revisão. Estes testes existem para que o
    degrau de velocidade não volte a ser aplicado no lugar errado, e para
    prender o 0.35 ao perfil ARENA — o `--nav2` normal não herda velocidade que
    não foi medida no cenário dele.

    Executa o BLOCO REAL do launch.sh entre os marcadores, não uma cópia (BO 63).
    """

    def _roda(self, arena):
        with open(os.path.join(RAIZ, 'launch.sh')) as f:
            texto = f.read()
        bloco = texto.split(MARCA_FOLLOW_INI)[1].split(MARCA_FOLLOW_FIM)[0]
        script = ('ARENA=%s\n' % ('true' if arena else 'false')) + bloco + \
                 '\necho "ARG=$ARENA_FOLLOW_ARG"\n'
        r = subprocess.run(['bash', '-c', script], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return r.stdout.strip().split('\n')[-1]

    def test_arena_passa_o_degrau_035(self):
        out = self._roda(True)
        self.assertIn('follow_forward_speed:=0.35', out)
        # o degrau não pode ter atropelado a velocidade-por-folga que já morava aqui
        self.assertIn('follow_clear_full:=1.2', out)
        self.assertIn('follow_clear_min:=0.35', out)

    def test_sem_arena_NAO_passa_velocidade_nenhuma(self):
        """Sem --arena o arg sai vazio e o nó cai no default do launch (0.30).
        Se um dia isto falhar, o `--nav2` normal começou a herdar um degrau
        medido só na arena."""
        self.assertEqual(self._roda(False), 'ARG=')

    def test_o_default_do_launch_e_o_valor_validado(self):
        """O default do `follow_forward_speed` no nav2.launch.py tem que ser o
        mesmo `forward_speed` do FollowConfig — senão subir o launch sem --arena
        muda a velocidade de quem nunca pediu."""
        with open(os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'launch',
                               'nav2.launch.py')) as f:
            launch = f.read()
        i = launch.index("'follow_forward_speed'")
        trecho = launch[i:i + 200]
        self.assertIn("default_value='0.30'", trecho)

        with open(os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'robot_nav',
                               'path_follower.py')) as f:
            follower = f.read()
        self.assertIn('forward_speed: float = 0.30', follower)

    def test_o_arg_chega_no_no(self):
        """Declarar o arg sem ligá-lo ao Node seria um botão morto: o launch
        aceitaria `follow_forward_speed:=0.35` e o robô andaria a 0.30."""
        with open(os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'launch',
                               'nav2.launch.py')) as f:
            launch = f.read()
        i = launch.index("executable='path_follower'")
        bloco = launch[i:i + 900]
        self.assertIn("'forward_speed'", bloco)
        self.assertIn("LaunchConfiguration('follow_forward_speed')", bloco)

    def test_o_no_NAO_aceita_param_a_quente(self):
        """Trava de documentação (review 2026-09-05): enquanto não existir
        `add_on_set_parameters_callback`, `ros2 param set /path_follower ...`
        é um NO-OP silencioso e não serve de rollback. Se alguém adicionar o
        callback, este teste falha — e aí o rollback a quente passa a valer e a
        nota do launch.sh precisa ser reescrita."""
        with open(os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'robot_nav',
                               'path_follower.py')) as f:
            follower = f.read()
        # a CHAMADA, não a palavra: o comentário do próprio nó cita o nome do
        # callback pra explicar que ele não existe.
        self.assertNotIn('.add_on_set_parameters_callback(', follower)
        with open(os.path.join(RAIZ, 'launch.sh')) as f:
            self.assertIn('ROLLBACK E\' RESTART', f.read())
