"""Isolamento: o que separa um CONE de um pedaço de parede/caixa.

BO do sim 2026-08-24 (mundo worlds/trekking.sdf, robô parado no spawn): o
detector publicava 10 cones por scan, sendo 2 reais e 8 fantasmas, e 100% dos
fantasmas eram pedaços da MESMA caixa de 0,90 m. A caixa nunca vira UM cluster:
numa face oblíqua o espaçamento entre feixes vizinhos (455 amostras/360°)
estoura o gap_threshold de 8 cm e a face se parte em mini-clusters de 6 a 16 cm
— todos dentro da janela de largura 4–45 cm.

Conclusão: largura não distingue. O que distingue é ISOLAMENTO — um cone solto
tem vazio em volta (o feixe logo depois salta pro fundo); um pedaço de
superfície tem vizinho na MESMA distância logo ali.
"""
import pytest

from robot_nav.cone_detector import cluster_isolated


ISO = 0.25


def test_cone_solto_no_meio_do_scan():
    """Fundo a 5 m, cone a 2 m: degrau grande dos dois lados."""
    r = [5.0, 5.0, 2.0, 2.0, 2.0, 5.0, 5.0]
    assert cluster_isolated(r, 2, 5, ISO) is True


def test_pedaco_de_parede_obliqua_nao_passa():
    """Face contínua: o vizinho está praticamente na mesma distância."""
    r = [2.00, 2.05, 2.10, 2.15, 2.20, 2.25, 2.30]
    assert cluster_isolated(r, 2, 4, ISO) is False


def test_so_um_lado_isolado_nao_basta():
    """Primeiro fragmento de uma face: vazio à esquerda, face à direita."""
    r = [6.0, 2.00, 2.05, 2.10, 2.15]
    assert cluster_isolated(r, 1, 3, ISO) is False


def test_cluster_na_borda_do_array_conta_como_isolado():
    """Sem vizinho daquele lado (fim da janela válida) = nada encostado."""
    r = [2.0, 2.0, 2.0, 5.0, 5.0]
    assert cluster_isolated(r, 0, 3, ISO) is True
    r2 = [5.0, 5.0, 2.0, 2.0, 2.0]
    assert cluster_isolated(r2, 2, 5, ISO) is True


def test_degrau_para_MAIS_PERTO_tambem_isola():
    """Descontinuidade é descontinuidade — objeto na frente do cone também."""
    r = [1.0, 3.0, 3.0, 3.0, 1.0]
    assert cluster_isolated(r, 1, 4, ISO) is True


def test_degrau_no_limiar():
    r = [2.0, 2.0 + ISO, 2.0 + ISO, 2.0]          # exatamente no limiar
    assert cluster_isolated(r, 1, 3, ISO) is True
    r2 = [2.0, 2.0 + ISO * 0.9, 2.0 + ISO * 0.9, 2.0]
    assert cluster_isolated(r2, 1, 3, ISO) is False


def test_iso_step_zero_desliga_o_criterio():
    """Knob em 0 = comportamento antigo (aceita tudo), pra A/B em campo."""
    r = [2.00, 2.05, 2.10, 2.15, 2.20]
    assert cluster_isolated(r, 2, 4, 0.0) is True
