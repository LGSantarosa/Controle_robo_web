#!/usr/bin/env python3
"""Tampão de fresta: fecha uma fresta SÓ no mapa do planejador (.pgm), com o
mundo (SDF) intacto.

Por que existe: a rede de segurança da prova de 05/09 é NÃO usar a fresta A e ir
pelo contorno. Fechá-la na tabela `OBST` fecharia no MUNDO também (mapa e SDF
saem da mesma fonte, por invariante deliberada) — o que muda o experimento em vez
de mudar a rota. O tampão quebra a invariante "mapa = mundo" DE PROPÓSITO e só
nesta direção; estes testes são o que impede a quebra de vazar pro SDF.

Os testes são SENSÍVEIS por construção: cada um afirma o par (com tampão / sem
tampão), então uma implementação que ignore a flag reprova.
"""
import importlib.util
import os
import sys

import numpy as np
import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
_spec = importlib.util.spec_from_file_location(
    'gera_arena_galpao', os.path.join(RAIZ, 'tools', 'gera_arena_galpao.py'))
ga = importlib.util.module_from_spec(_spec)
sys.modules['gera_arena_galpao'] = ga
_spec.loader.exec_module(ga)


def _le_pgm(caminho):
    """Devolve (array HxW, W, H). Lê o P5 de cabeçalho conhecido do gerador."""
    with open(caminho, 'rb') as f:
        dados = f.read()
    # P5\n#comentario\nW H\n255\n<binario> -> 3 linhas úteis (o # não conta)
    pos, linhas = 0, []
    while len(linhas) < 3:
        fim = dados.index(b'\n', pos)
        linha = dados[pos:fim]
        pos = fim + 1
        if not linha.startswith(b'#'):
            linhas.append(linha)
    W, H = (int(v) for v in linhas[1].split())
    return np.frombuffer(dados[pos:], dtype=np.uint8).reshape(H, W), W, H


def _celula(o, x, y, H):
    """(x,y) do mundo -> (linha, coluna) do pgm, mesma conta do gerador."""
    x0, y0 = o
    return int(round((H - 1) - (y - y0) / ga.RES)), int(round((x - x0) / ga.RES))


def _gera(tmp_path, fecha=()):
    pgm = str(tmp_path / 'arena_galpao.pgm')
    yml = str(tmp_path / 'arena_galpao.yaml')
    W, H, o = ga.gera_mapa(pgm, yml, fecha=fecha)
    a, _W, _H = _le_pgm(pgm)
    return a, o, H


# ---- o centro da fresta A: livre sem tampão, ocupado com tampão -------------

def test_fresta_A_livre_sem_tampao(tmp_path):
    a, o, H = _gera(tmp_path)
    l, c = _celula(o, 7.5, 2.25, H)
    assert a[l, c] == 254, 'sem tampão o centro da fresta A tem que estar LIVRE'


def test_fresta_A_ocupada_com_tampao(tmp_path):
    a, o, H = _gera(tmp_path, fecha=('A_fresta90',))
    l, c = _celula(o, 7.5, 2.25, H)
    assert a[l, c] == 0, 'com tampão o centro da fresta A tem que estar OCUPADO'


def test_tampao_cobre_o_vao_INTEIRO(tmp_path):
    """Não basta pintar o centro: um vão parcialmente aberto ainda seria
    planejável (0.90 - folga). Cobre de batente a batente."""
    a, o, H = _gera(tmp_path, fecha=('A_fresta90',))
    for y in np.arange(1.81, 2.70, 0.02):       # dentro do vão, sem tocar o bloco
        l, c = _celula(o, 7.5, float(y), H)
        assert a[l, c] == 0, f'vão aberto em y={y:.2f}'


def test_tampao_nao_transborda_pra_fora_do_vao(tmp_path):
    """O tampão tem a espessura do bloco (0.60 em x): não pode engordar o
    obstáculo nem estreitar o CONTORNO (y > 4.20), que é a saída da missão."""
    a, o, H = _gera(tmp_path, fecha=('A_fresta90',))
    for y in (4.30, 5.0, 6.0, 8.0):             # o contorno tem que seguir livre
        l, c = _celula(o, 7.5, y, H)
        assert a[l, c] == 254, f'tampão invadiu o contorno em y={y:.2f}'
    for x in (7.10, 7.85):                      # fora da espessura do bloco
        l, c = _celula(o, x, 2.25, H)
        assert a[l, c] == 254, f'tampão transbordou em x={x:.2f}'


# ---- o mundo NÃO muda -------------------------------------------------------

def _cobre(caixa, x, y):
    _n, cx, cy, sx, sy = caixa
    return abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2


def test_o_MUNDO_nao_tem_tampao_no_vao_da_fresta_A():
    """A garantia central: o tampão é do planejador, NUNCA do mundo. Afirma a
    estrutura (nenhum sólido cobre o centro do vão), não a igualdade de uma
    string consigo mesma — que é cega a um tampão que vaze pra blocos(), porque
    aí ele estaria dos DOIS lados da comparação (foi o defeito injetado)."""
    solidos = ga.muros() + ga.blocos()
    culpados = [c[0] for c in solidos if _cobre(c, 7.5, 2.25)]
    assert not culpados, f'o mundo fechou a fresta A: {culpados}'
    txt = ga.corpo_sdf()
    assert 'A_fresta90_1' in txt and 'A_fresta90_2' in txt
    assert ga.OBST[0][3] == [(0.30, 1.80), (2.70, 4.20)]


def test_gerar_mapa_com_tampao_nao_tem_efeito_colateral_no_mundo(tmp_path):
    """Complementar ao de cima: nem por estado global o tampão pode vazar."""
    antes = ga.corpo_sdf()
    _gera(tmp_path, fecha=('A_fresta90',))
    assert ga.corpo_sdf() == antes
    assert not [c[0] for c in ga.muros() + ga.blocos() if _cobre(c, 7.5, 2.25)]


def test_outras_frestas_intactas_com_tampao_em_A(tmp_path):
    """Fechar A não pode fechar B/C/D — senão a volta não tem como terminar."""
    a, o, H = _gera(tmp_path, fecha=('A_fresta90',))
    for nome, eixo, coord, faixas, _f, _c in ga.OBST[1:]:
        meio = (faixas[0][1] + faixas[1][0]) / 2
        x, y = (coord, meio) if eixo == 'x' else (meio, coord)
        l, c = _celula(o, x, y, H)
        assert a[l, c] == 254, f'{nome} foi fechada junto'


def test_nome_de_fresta_desconhecido_explode(tmp_path):
    with pytest.raises(ValueError):
        _gera(tmp_path, fecha=('Z_naoexiste',))


# ---- a guarda da CLI (o flag não pode chegar perto do mundo) ----------------

def _cli(tmp_path, *args):
    import subprocess
    return subprocess.run([sys.executable,
                           os.path.join(RAIZ, 'tools', 'gera_arena_galpao.py'),
                           *args], capture_output=True, text=True, cwd=str(tmp_path))


def test_cli_recusa_fecha_fresta_com_sdf(tmp_path):
    """--sdf retornava ANTES da guarda e escrevia o mundo mesmo assim."""
    dest = tmp_path / 'w.sdf'
    r = _cli(tmp_path, '--sdf', str(dest), '--fecha-fresta', 'A')
    assert r.returncode != 0, r.stdout + r.stderr
    assert not dest.exists(), 'escreveu o SDF apesar de recusar'
    assert 'NÃO altera o mundo' in (r.stdout + r.stderr)


def test_cli_mapa_tampado_sai_com_outro_nome(tmp_path):
    """O mapa tampado não pode sobrescrever o oficial: nome diferente."""
    r = _cli(tmp_path, '--mapa', str(tmp_path), '--fecha-fresta', 'A')
    assert r.returncode == 0, r.stdout + r.stderr
    assert (tmp_path / 'arena_galpao_semA.pgm').exists()
    assert (tmp_path / 'arena_galpao_semA.yaml').exists()
    assert not (tmp_path / 'arena_galpao.pgm').exists(), 'sobrescreveu o oficial'
