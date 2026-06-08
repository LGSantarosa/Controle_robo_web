#!/usr/bin/env python3
"""Núcleo PURO da odometria fundida (rodas + IMU + flow) com degradação graciosa.

Sem dependência de rclpy — testável isoladamente (estilo cone_pose_fix.py). O nó
`pose_estimator` alimenta este núcleo com velocidades de roda, taxa/freshness da IMU,
velocidade do flow + peso α, e dt; e publica o resultado (/odom + TF + /trekking/*).

Seleção da TAXA de yaw (degradação graciosa):
  - IMU = MPU6050 (6 eixos): NÃO há yaw absoluto, só taxa do giro. O yaw é
    sempre INTEGRADO (ponto-médio); a IMU só troca a FONTE da taxa:
      - IMU fresca  → taxa do giro (vence a derrapagem do skid-steer no giro).
      - IMU ausente → taxa do diferencial de roda, igual ao odom_publisher
                      antigo. É o caso degenerado.
Translação:
  - vx_body = α·vx_flow + (1-α)·vx_roda ; vy_body = α·vy_flow (roda cega à lateral).
"""
import math
from dataclasses import dataclass

from .utils import wrap_pi


def wheel_twist(v_fl, v_fr, v_rl, v_rr, wheel_base):
    """4 velocidades de roda (m/s) → (vx_body m/s, angular rad/s) diff-drive.

    Média por lado (robusto a derrapagem de uma roda). `wheel_base` é a bitola
    EFETIVA (calibrada), não a geométrica.
    """
    v_left = (v_fl + v_rl) / 2.0
    v_right = (v_fr + v_rr) / 2.0
    vx = (v_left + v_right) / 2.0
    angular = (v_right - v_left) / wheel_base
    return vx, angular


def flow_alpha(quality, q_mid, q_slope, flow_age, flow_timeout):
    """Peso do flow ∈ [0,1]. Zero se o flow está velho (age > timeout).

    Sigmoid sobre (quality - q_mid)/q_slope, estável pra evitar overflow.
    """
    if flow_age > flow_timeout:
        return 0.0
    z = (quality - q_mid) / max(q_slope, 1e-3)
    if z >= 0:
        return 1.0 / (1.0 + math.exp(-z))
    e = math.exp(z)
    return e / (1.0 + e)


def flow_yaw_gate(yaw_rate, gate_lo, gate_hi):
    """Fator ∈ [0,1] que ZERA o flow em rotação rápida.

    O sensor está no CENTRO do robô (não é erro de ω×r). Em giro o PMW3901, que
    é um sensor de TRANSLAÇÃO, vê a textura do chão GIRANDO sob ele — não uma
    translação limpa — e o casamento de imagem cospe dx/dy espúrio. Soma-se a
    derrapagem do skid-steer (o robô não pivota perfeito, translada um pouco DE
    VERDADE), indistinguível do artefato. Num spin chega a ~0,5 m de deriva
    lateral no flow (medido 2026-06-08). A IMU dá o ω limpo (~99%), então usamos
    |yaw_rate| pra cortar: passa inteiro abaixo de gate_lo, ignora acima de
    gate_hi, rampa linear no meio (sem degrau → sem flicker no α).
    """
    w = abs(yaw_rate)
    if w <= gate_lo:
        return 1.0
    if w >= gate_hi:
        return 0.0
    return (gate_hi - w) / (gate_hi - gate_lo)


def fuse_translation(vx_wheel, flow_vx, flow_vy, alpha):
    """vx/vy no body frame: funde flow (peso α) e roda (vx); roda contribui 0 em vy."""
    vx_body = alpha * flow_vx + (1.0 - alpha) * vx_wheel
    vy_body = alpha * flow_vy
    return vx_body, vy_body


@dataclass
class StepResult:
    x: float
    y: float
    yaw: float
    yaw_rate: float
    vx_body: float
    vy_body: float
    yaw_source: str   # 'imu' | 'wheel'


class FusedOdom:
    """Mantém (x, y, yaw) no frame odom e integra um passo de odometria fundida."""

    def __init__(self, wheel_base):
        self.wheel_base = float(wheel_base)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, dt, v_fl, v_fr, v_rl, v_rr,
             imu_fresh, imu_yaw_rate,
             flow_vx, flow_vy, alpha):
        vx_wheel, wheel_angular = wheel_twist(v_fl, v_fr, v_rl, v_rr, self.wheel_base)

        # --- seleção da TAXA de yaw com degradação graciosa ---
        # MPU6050 não dá yaw absoluto: integramos sempre a partir do yaw atual
        # (ponto-médio, igual ao odom_publisher). A IMU só troca a fonte da taxa
        # (giro × derrapagem da roda); como é sempre integração relativa, uma
        # correção manual de direção (yaw_fix) agora persiste mesmo com IMU.
        if imu_fresh:
            yaw_rate = imu_yaw_rate
            yaw_source = 'imu'
        else:
            yaw_rate = wheel_angular
            yaw_source = 'wheel'

        integ_yaw = wrap_pi(self.yaw + 0.5 * yaw_rate * dt)
        self.yaw = wrap_pi(self.yaw + yaw_rate * dt)

        # --- translação fundida ---
        vx_body, vy_body = fuse_translation(vx_wheel, flow_vx, flow_vy, alpha)

        # --- integra no mundo usando integ_yaw ---
        cy = math.cos(integ_yaw)
        sy = math.sin(integ_yaw)
        self.x += (vx_body * cy - vy_body * sy) * dt
        self.y += (vx_body * sy + vy_body * cy) * dt

        return StepResult(self.x, self.y, self.yaw, yaw_rate,
                          vx_body, vy_body, yaw_source)
