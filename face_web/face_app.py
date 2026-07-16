#!/usr/bin/env python3
"""Cara do robô — servidor SEPARADO do console (porta 7000).

Feito pro iPad 2 (iOS 9.3.5, WebKit de 2015) pendurado no tripé em cima do
robô: o console principal (porta 5000) usa JS moderno que o iPad não parseia
(a página carrega e morre em silêncio). Aqui TUDO que vai pro navegador é
ES5 puro — leia o aviso no topo de static/face.js antes de mexer; o
test_face_app.py barra sintaxe moderna.

Rodar: python3 face_web/face_app.py   (escuta em 0.0.0.0:7000)

Fase 2 (olhos seguem a pessoa): o motion_guard grava o rumo do cluster
móvel mais próximo em /tmp/motion_guard_face.json; a rota /state lê e
devolve pro face.js (poll XHR). Spec:
docs/superpowers/specs/2026-07-16-face-follow-design.md
"""
import os
import time

from flask import Flask, jsonify, render_template

import face_state

app = Flask(__name__)

# Sinal do espelhamento olho×mundo: depende de pra onde o iPad aponta no
# tripé — flipar pra -1.0 na demo se o olho seguir pro lado errado.
FACE_GAZE_SIGN = 1.0
STATE_FILE = os.environ.get('FACE_STATE_FILE',
                            '/tmp/motion_guard_face.json')


@app.route('/')
def face():
    return render_template('face.html')


@app.route('/state')
def state():
    """Fase 2 (cara reativa): rumo da pessoa vindo do motion_guard."""
    return jsonify(face_state.read_state(STATE_FILE, time.time(),
                                         sign=FACE_GAZE_SIGN))


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=7000)
