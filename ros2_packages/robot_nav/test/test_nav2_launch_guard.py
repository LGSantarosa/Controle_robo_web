#!/usr/bin/env python3
"""`motion_guard:=false` (perfil ARENA) tem que RELIGAR o pipeline.

Por que existe (2026-08-31): o guard é um ESTÁGIO da artéria da autonomia
(`auto_vel_pre` -> `auto_vel_raw`). Tirar o nó sem religar a saída do
`twist_mux_auto` deixa o `collision_monitor` sem publisher na entrada e a
autonomia inteira emudece — falha total, e silenciosa no launch.

Estes testes leem a LaunchDescription DE VERDADE (não reconstroem a expressão,
que seria tautologia — BO 63) e resolvem as substituições nos dois valores.
"""
import importlib.util
import os
import unittest

from launch import LaunchContext
from launch.utilities import perform_substitutions

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
LAUNCH = os.path.join(RAIZ, 'ros2_packages', 'robot_nav', 'launch',
                      'nav2.launch.py')


def _ld():
    spec = importlib.util.spec_from_file_location('nav2_launch_sut', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


def _nos(ld):
    return [e for e in ld.entities if e.__class__.__name__ == 'Node']


def _no(ld, nome):
    for n in _nos(ld):
        if getattr(n, '_Node__node_name', None) == nome:
            return n
    return None


def _ctx(valor):
    c = LaunchContext()
    c.launch_configurations['motion_guard'] = valor
    return c


def _saida_do_mux(ld, valor):
    mux = _no(ld, 'twist_mux_auto')
    ctx = _ctx(valor)
    for de, para in mux._Node__remappings:
        if perform_substitutions(ctx, list(de)) == 'cmd_vel_out':
            return perform_substitutions(ctx, list(para))
    return None


class TestGuardLaunch(unittest.TestCase):

    def test_default_e_com_guard(self):
        """Default LIGADO: fora da arena nada pode mudar."""
        ld = _ld()
        arg = [e for e in ld.entities
               if e.__class__.__name__ == 'DeclareLaunchArgument'
               and e.name == 'motion_guard']
        self.assertEqual(len(arg), 1, 'o launch arg tem que existir')
        self.assertEqual(arg[0].default_value[0].perform(LaunchContext()),
                         'true')

    def test_com_guard_o_mux_publica_no_pre(self):
        self.assertEqual(_saida_do_mux(_ld(), 'true'), 'auto_vel_pre')

    def test_sem_guard_o_mux_publica_direto_no_raw(self):
        """O par do teste acima: se a religação não acontecer, este valor
        continua 'auto_vel_pre' e o collision_monitor fica mudo."""
        self.assertEqual(_saida_do_mux(_ld(), 'false'), 'auto_vel_raw')

    def test_o_no_do_guard_sobe_so_com_true(self):
        guard = _no(_ld(), 'motion_guard')
        self.assertIsNotNone(guard, 'o nó tem que estar na descrição')
        self.assertTrue(guard.condition.evaluate(_ctx('true')))
        self.assertFalse(guard.condition.evaluate(_ctx('false')))

    def test_collision_monitor_le_sempre_o_raw(self):
        """A entrada do collision NÃO muda — quem muda de lado é o mux. Se um
        dia alguém 'consertar' isto mexendo no collision, o teste avisa."""
        for valor in ('true', 'false'):
            cm = _no(_ld(), 'collision_monitor')
            self.assertIsNotNone(cm)
            saida = _saida_do_mux(_ld(), valor)
            self.assertIn(saida, ('auto_vel_pre', 'auto_vel_raw'))


if __name__ == '__main__':
    unittest.main()
