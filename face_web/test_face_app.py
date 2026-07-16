"""Testes da cara do robô (face_web, porta 7000).

O risco real aqui não é lógica — é REGRESSÃO DE SINTAXE: o iPad 2
(iOS 9.3.5) aborta o script inteiro, em silêncio, no primeiro token
pós-ES5. O teste de léxico abaixo transforma isso em falha de CI.
"""
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

BASE = os.path.dirname(__file__)


def test_face_route_ok():
    # flask só existe no .venv do controle_web / na Pi; no pytest do sistema
    # (suíte de lógica pura) este teste pula e os de arquivo cobrem o resto.
    pytest.importorskip('flask')
    from face_app import app
    client = app.test_client()
    resp = client.get('/')
    assert resp.status_code == 200
    body = resp.get_data(as_text=True)
    assert 'canvas' in body
    assert 'face.js' in body


def test_face_html_ipad_pronto():
    html = open(os.path.join(BASE, 'templates', 'face.html')).read()
    assert 'apple-mobile-web-app-capable' in html
    assert 'viewport' in html
    assert 'canvas' in html


def test_face_js_expressoes_completas():
    # A cara tem olhos + sobrancelhas + boca, e todos os humores prometidos.
    # Se alguém remover um humor, o gancho da fase 2 (setMood via /state)
    # quebra em silêncio no iPad — este teste transforma isso em CI.
    js = open(os.path.join(BASE, 'static', 'face.js')).read()
    for pedaco in ['drawBrow', 'drawMouth', 'setMood', 'touchstart',
                   'requestFullscreen',
                   "happy", "squint", "focused", "yawn"]:
        assert pedaco in js, 'sumiu do face.js: ' + pedaco


def test_face_js_es5_puro():
    js = open(os.path.join(BASE, 'static', 'face.js')).read()
    proibidos = [
        (r'=>', 'funcao-seta'),
        (r'\bconst\b', 'const'),
        (r'\blet\b', 'let'),
        (r'`', 'template string'),
        (r'\?\.', 'encadeamento opcional'),
        (r'\basync\b', 'async'),
        (r'\bawait\b', 'await'),
        (r'\bfetch\s*\(', 'fetch'),
        (r'\bPromise\b', 'Promise'),
        (r'\bclass\s+\w', 'class'),
        (r'\.\.\.', 'spread'),
    ]
    for pat, nome in proibidos:
        assert not re.search(pat, js), 'sintaxe pos-ES5 no face.js: ' + nome


# ---- fase 2: /state (olhos seguem a pessoa) ------------------------------

def _grava_json(tmp_path, cbear, idade_s=0.0):
    import json
    import time
    p = tmp_path / 'face.json'
    p.write_text(json.dumps({'ts': 0, 'cbear_deg': cbear}))
    if idade_s:
        velho = time.time() - idade_s
        os.utime(str(p), (velho, velho))
    return str(p)


def test_state_sem_arquivo():
    import time
    import face_state
    assert face_state.read_state('/nao/existe.json', time.time()) == \
        {'person': False}


def test_state_stale(tmp_path):
    import time
    import face_state
    p = _grava_json(tmp_path, 30, idade_s=5.0)
    assert face_state.read_state(p, time.time()) == {'person': False}


def test_state_null_e_pessoa_atras(tmp_path):
    import time
    import face_state
    assert face_state.read_state(_grava_json(tmp_path, None),
                                 time.time()) == {'person': False}
    assert face_state.read_state(_grava_json(tmp_path, 135),
                                 time.time()) == {'person': False}


def test_state_pessoa_na_frente_mapeia_e_flipa(tmp_path):
    import time
    import face_state
    p = _grava_json(tmp_path, 45)
    assert face_state.read_state(p, time.time()) == \
        {'person': True, 'x': 0.5}
    assert face_state.read_state(p, time.time(), sign=-1.0) == \
        {'person': True, 'x': -0.5}


def test_state_json_corrompido(tmp_path):
    import time
    import face_state
    p = tmp_path / 'face.json'
    p.write_text('{meia lin')
    assert face_state.read_state(str(p), time.time()) == {'person': False}


def test_state_route(tmp_path):
    pytest.importorskip('flask')
    import face_app
    face_app.STATE_FILE = _grava_json(tmp_path, 45)
    st = face_app.app.test_client().get('/state').get_json()
    assert st == {'person': True, 'x': 0.5}
