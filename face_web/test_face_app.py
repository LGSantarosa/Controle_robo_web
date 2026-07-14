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
