#!/usr/bin/env python3
"""Cara do robô — servidor SEPARADO do console (porta 7000).

Feito pro iPad 2 (iOS 9.3.5, WebKit de 2015) pendurado no tripé em cima do
robô: o console principal (porta 5000) usa JS moderno que o iPad não parseia
(a página carrega e morre em silêncio). Aqui TUDO que vai pro navegador é
ES5 puro — leia o aviso no topo de static/face.js antes de mexer; o
test_face_app.py barra sintaxe moderna.

Rodar: python3 face_web/face_app.py   (escuta em 0.0.0.0:7000)
"""
from flask import Flask, render_template

app = Flask(__name__)


@app.route('/')
def face():
    return render_template('face.html')


# Gancho da fase 2 (cara REATIVA): uma rota /state vai devolver o humor
# derivado do estado real do robô (guard blocked, goal ativo, idle...) e o
# face.js passa a chamá-la por XHR. Hoje é decorativo puro — nada a fazer.


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=7000)
