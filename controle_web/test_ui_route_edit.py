"""Apagar UM ponto da rota (2026-09-06).

Antes, tirar um ponto errado do meio custava "Limpar" + redesenhar a rota
inteira. O teste roda a FUNÇÃO REAL extraída do `map.js` no node, em vez de
reimplementar a lógica em Python (que viraria tautologia). Sem node — é o caso
da Pi — sobra o teste de fiação, que roda em qualquer lugar.
"""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

RAIZ = Path(__file__).resolve().parents[1]
MAPJS = RAIZ / 'controle_web' / 'static' / 'js' / 'map.js'
HTML = RAIZ / 'controle_web' / 'templates' / 'index.html'
NODE = shutil.which('node') or shutil.which('nodejs')


def _funcao(nome):
    """Recorta `function <nome>(...) { ... }` do map.js contando chaves."""
    src = MAPJS.read_text(encoding='utf-8')
    i = src.index(f'function {nome}(')
    j = src.index('{', i)
    nivel = 0
    for k in range(j, len(src)):
        if src[k] == '{':
            nivel += 1
        elif src[k] == '}':
            nivel -= 1
            if nivel == 0:
                return src[i:k + 1]
    raise AssertionError(f'{nome}: chaves não fecham')


def _roda(waypoints, selecionado, ativo=False):
    harness = """
let waypoints = %s;
let wpSelectedIdx = %d;
let wpActive = %s;
let lastGoal = {x: 1, y: 2};
const wpStatusEl = { textContent: '' };
function waypointType(wp) { return (wp && wp.light === false) ? 'passagem' : 'goal'; }
function updateWpButtons() { if (wpSelectedIdx >= waypoints.length) wpSelectedIdx = -1; }
function render() {}
%s
deleteSelectedWaypoint();
console.log(JSON.stringify({waypoints, wpSelectedIdx,
                            status: wpStatusEl.textContent, lastGoal}));
""" % (json.dumps(waypoints), selecionado,
       'true' if ativo else 'false', _funcao('deleteSelectedWaypoint'))
    r = subprocess.run([NODE, '-e', harness], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def _rota(n):
    return [{'x': float(i), 'y': 0.0, 'yaw': 0.0, 'light': True} for i in range(n)]


# NÃO usar `pytestmark`: ele valeria pro módulo inteiro e pularia junto os testes
# de fiação lá embaixo, que não precisam de node e têm que rodar na Pi também.
precisa_node = pytest.mark.skipif(NODE is None, reason='node ausente (ex.: a Pi)')


@precisa_node
def test_apaga_so_o_ponto_selecionado():
    fora = _roda(_rota(4), 1)
    assert [w['x'] for w in fora['waypoints']] == [0.0, 2.0, 3.0]


@precisa_node
def test_selecao_fica_no_mesmo_indice_pra_apagar_varios_seguidos():
    fora = _roda(_rota(4), 1)
    assert fora['wpSelectedIdx'] == 1            # agora aponta pro que era o 2
    assert fora['waypoints'][1]['x'] == 2.0


@precisa_node
def test_apagar_o_ultimo_sobe_a_selecao():
    fora = _roda(_rota(3), 2)
    assert fora['wpSelectedIdx'] == 1
    assert len(fora['waypoints']) == 2


@precisa_node
def test_apagar_o_unico_esvazia_e_solta_o_goal():
    fora = _roda(_rota(1), 0)
    assert fora['waypoints'] == []
    assert fora['wpSelectedIdx'] == -1
    assert fora['lastGoal'] is None              # senão fica marcador órfão no mapa
    assert fora['status'] == 'rota vazia'


@precisa_node
def test_com_a_rota_RODANDO_nao_apaga_nada():
    """Apagar ponto no meio da navegação dessincronizaria a lista da UI da que o
    robô está executando."""
    fora = _roda(_rota(3), 1, ativo=True)
    assert len(fora['waypoints']) == 3


@precisa_node
def test_sem_selecao_nao_apaga_nada():
    fora = _roda(_rota(3), -1)
    assert len(fora['waypoints']) == 3


# ---- fiação (roda sem node, inclusive na Pi) --------------------------------

def test_o_botao_existe_e_e_separado_do_Limpar():
    html = HTML.read_text(encoding='utf-8')
    assert 'id="btn-wp-del"' in html
    assert 'id="btn-wp-clear"' in html            # o Limpar continua existindo
    js = MAPJS.read_text(encoding='utf-8')
    assert "getElementById('btn-wp-del')" in js
    assert "btnWpDel.addEventListener('click', deleteSelectedWaypoint)" in js


def test_o_botao_fica_desligado_sem_selecao_e_durante_a_rota():
    js = MAPJS.read_text(encoding='utf-8')
    assert 'btnWpDel.disabled = wpActive || !selected' in js
