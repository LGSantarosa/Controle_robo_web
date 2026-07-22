#!/usr/bin/env python3
"""Gera os mp3 das falas da cara (gTTS pt-BR). Rodar num ambiente com gtts +
internet (ex.: o .venv do controle_web): controle_web/.venv/bin/python3
face_web/tools/gen_tts.py. Não precisa rodar no deploy — os mp3 são versionados."""
from gtts import gTTS

FALAS = {
    'ola': 'Olá!',
    'licenca': 'Com licença!',
    'seguir_inicio': 'Irei te seguir, tente ficar próximo e ir devagar',
    'nao_te_vejo': 'Não estou mais te vendo, poderia se aproximar?',
}
for nome, texto in FALAS.items():
    gTTS(texto, lang='pt', tld='com.br').save('face_web/static/%s.mp3' % nome)
    print('gerado face_web/static/%s.mp3' % nome)
