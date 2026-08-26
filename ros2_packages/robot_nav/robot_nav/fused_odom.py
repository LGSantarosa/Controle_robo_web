#!/usr/bin/env python3
"""Núcleo PURO da odometria fundida (rodas + IMU + flow) com degradação graciosa.

Sem dependência de rclpy — testável isoladamente (estilo cone_pose_fix.py). O nó
`pose_estimator` alimenta este núcleo com velocidades de roda, taxa/freshness da IMU,
velocidade do flow + peso α, e dt; e publica o resultado (/odom + TF + /trekking/*).

Seleção da TAXA de yaw (degradação graciosa), com DUAS IMUs:
  - IMU #1 = MPU6050 (6 eixos): só taxa do giro, sem yaw absoluto.
  - IMU #2 = BNO055 (9 eixos): taxa do giro + orientação ABSOLUTA (mag).
  O yaw é sempre INTEGRADO (ponto-médio); as IMUs trocam a FONTE da taxa:
      - as duas frescas → média ponderada das duas taxas (ver blend_yaw_rate),
                          com gate de discordância pra pegar montagem invertida.
      - só uma fresca   → a taxa dela.
      - nenhuma         → taxa do diferencial de roda (caso degenerado, igual ao
                          odom_publisher antigo).
E, POR CIMA da integração, a BNO055 ainda ancora o yaw no norte magnético:
uma correção LENTA e LIMITADA (heading_correction) tira a deriva acumulada sem
dar solavanco no heading.

Translação:
  - vx_body = α·vx_flow + (1-α)·vx_roda ; vy_body = α·vy_flow (roda cega à lateral).
  A BNO055 NÃO entra na translação: integrar accel duas vezes diverge em metros
  por minuto (o accel dela mede gravidade + vibração do chassi, não deslocamento
  útil). O ganho dela na POSIÇÃO é indireto e vale muito mais — a direção pra
  onde a velocidade das rodas/flow é projetada passa a ser um yaw sem deriva.
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


def flow_plausible(flow_vx, flow_vy, v_max):
    """True se a velocidade do flow é fisicamente plausível (|v| ≤ v_max em cada
    eixo).

    O PMW3901 cospe lixo por EMI do motor na MANOBRA (giro no lugar, acel/freia):
    medido na bancada flow=-10,6 m/s e +2,27 m/s com as rodas PARADAS, e com
    quality ALTA (130-160) — o gate de qualidade (flow_alpha) NÃO pega. Como o
    chassi não passa de ~0,35 m/s, qualquer leitura muito acima disso é EMI.
    Descartar o flow nesse tick (cai pra roda+IMU) evita que o pico teleporte a
    pose e perca a localização. Band-aid até o HW do shifter ser trocado
    (ver project_pmw3901_emi_motor).
    """
    return abs(flow_vx) <= v_max and abs(flow_vy) <= v_max


def flow_tick_velocity(accum_dx, accum_dy, dt):
    """Velocidade do flow no tick a partir do deslocamento ACUMULADO desde o
    último tick (m), dividido pelo dt do TICK.

    Crítico: usar o dt da janela de integração (o tick), NÃO o intervalo de
    chegada das mensagens. O PMW3901 chega em rajada (várias msgs coladas, depois
    um gap); calcular flow_vx = d/dt_chegada gera uma velocidade instantânea
    alta que, SEGURADA e re-integrada a 50 Hz, dobra a distância (bancada
    2026-06-08: odom_net 4,88 m num percurso de 2,0 m, fator ~2,1×). Acumular o
    deslocamento e dividir pelo dt do tick garante Σ(v·dt) = Σ(deslocamento) — a
    pose não infla com o jitter de chegada.
    """
    if dt <= 0.0:
        return 0.0, 0.0
    return accum_dx / dt, accum_dy / dt


def slip_estimate(vx_wheel, vx_ref, ref_alpha, alpha_min=0.1):
    """Divergência roda ↔ referência de translação (m/s), ou NaN sem referência.

    NaN e NÃO zero. Zero é uma AFIRMAÇÃO — "medi, e as rodas conferem" — e era
    mentira desde 2026-07-01, quando o PMW3901 foi arrancado: sem flow o α fica
    0, o ramo `else` devolvia 0.0 e /trekking/slip publicava "sem derrapagem"
    pra sempre. Um detector que reporta tudo em ordem porque perdeu o sensor é
    pior que detector nenhum, porque parece que tem.

    NaN diz "não sei", que é a verdade, e se propaga: comparação com NaN é
    sempre False (o warn não dispara), e qualquer consumidor que faça conta com
    ele vira NaN em vez de absorver um zero como dado bom.
    """
    if ref_alpha <= alpha_min:
        return float('nan')
    return vx_wheel - vx_ref


def blend_yaw_rate(imu_fresh, imu_rate, imu2_fresh, imu2_rate,
                   imu2_weight, disagree_min=0.15):
    """Combina as taxas de yaw das DUAS IMUs. Devolve (rate, source, disagree).

    `rate` é None quando nenhuma IMU está fresca (o chamador cai pra roda).
    `imu2_weight` ∈ [0,1] é o peso da BNO055 quando as duas estão frescas.

    GATE DE DISCORDÂNCIA (o motivo de esta função existir em vez de uma média
    solta): se a BNO055 for montada com outra orientação — girada, de lado, de
    ponta-cabeça — a taxa dela chega com o SINAL trocado, e uma média ingênua
    de +ω com -ω dá ZERO: o robô giraria no chão sem girar no mapa, o pior modo
    de falha possível (a pose mente em silêncio e o SLAM só desmonta minutos
    depois). Então: quando as duas leem giro REAL (|ω| > disagree_min, acima do
    ruído/bias parado) e os sinais DIVERGEM, a #2 é descartada no tick e a #1
    manda sozinha — comportamento idêntico ao de antes da BNO055 existir. O
    chamador recebe disagree=True pra gritar no log; o conserto é o parâmetro
    imu2_yaw_sign, não uma reflashada da MEGA.
    """
    if imu_fresh and imu2_fresh:
        disagree = (abs(imu_rate) > disagree_min and abs(imu2_rate) > disagree_min
                    and (imu_rate > 0.0) != (imu2_rate > 0.0))
        if disagree:
            return imu_rate, 'imu', True
        w = min(1.0, max(0.0, imu2_weight))
        return (1.0 - w) * imu_rate + w * imu2_rate, 'imu+imu2', False
    if imu_fresh:
        return imu_rate, 'imu', False
    if imu2_fresh:
        return imu2_rate, 'imu2', False
    return None, 'wheel', False


def heading_correction(yaw, yaw_ref, gain, dt, max_rate):
    """Quanto girar o yaw NESTE tick pra puxá-lo até o heading absoluto (rad).

    Complementar de 1ª ordem: corr = gain·erro·dt, saturado em max_rate·dt.
    Nunca substitui o yaw pelo valor absoluto de uma vez, por dois motivos:

    1. O magnetômetro tem ruído e sofre EMI dos motores (o robô carrega duas
       placas de hoverboard chaveando corrente alta). Um "yaw = yaw_mag" a 50 Hz
       transformaria cada distúrbio magnético num salto de heading — e no Nav2
       um salto de heading vira uma manobra de correção real, com o robô
       jogando o corpo pro lado por causa de um ímã no chão.
    2. Com gate lento (gain ~0.2 → τ≈5 s) e teto (max_rate), o giro fica
       DEVAGAR e monotônico: o SLAM/AMCL reconvergem sem perder o casamento do
       scan, e uma correção manual (yaw_fix) não é apagada no tick seguinte.

    gain em 1/s (0 desliga), max_rate em rad/s.
    """
    if gain <= 0.0 or dt <= 0.0:
        return 0.0
    err = wrap_pi(yaw_ref - yaw)
    corr = gain * err * dt
    cap = abs(max_rate) * dt
    if cap > 0.0:
        corr = max(-cap, min(cap, corr))
    return corr


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
    yaw_source: str          # 'imu' | 'imu2' | 'imu+imu2' | 'wheel'
    # Correção de heading absoluto (BNO055) aplicada NESTE tick, em rad. 0.0 =
    # nenhuma (sem IMU #2, mag descalibrado ou correção desligada).
    heading_corr: float = 0.0
    # True quando as duas IMUs leram giro real com SINAIS opostos — quase sempre
    # imu2_yaw_sign errado. A #2 foi ignorada neste tick; ver blend_yaw_rate.
    rate_disagree: bool = False


class FusedOdom:
    """Mantém (x, y, yaw) no frame odom e integra um passo de odometria fundida."""

    def __init__(self, wheel_base):
        self.wheel_base = float(wheel_base)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, dt, v_fl, v_fr, v_rl, v_rr,
             imu_fresh, imu_yaw_rate,
             flow_vx, flow_vy, alpha,
             wheel_fresh=True,
             imu2_fresh=False, imu2_yaw_rate=0.0, imu2_rate_weight=0.0,
             abs_yaw=None, heading_gain=0.0, heading_max_rate=0.0):
        """Um passo de fusão. Os argumentos da IMU #2 (BNO055) são OPCIONAIS:
        omitidos, o resultado é bit a bit o de antes dela existir.

        abs_yaw = heading ABSOLUTO já trazido pro frame odom (rad), ou None
        quando não há um confiável neste tick (BNO055 ausente/velha, mag
        descalibrado, correção desligada). Quem decide isso é o pose_estimator.
        """
        vx_wheel, wheel_angular = wheel_twist(v_fl, v_fr, v_rl, v_rr, self.wheel_base)

        # --- guarda de freshness das rodas (anti-giro-fantasma) ---
        # Quando o stream da MEGA para (I2C lockup do firmware — ver
        # project_mega_i2c_hang), `_on_wheels` deixa de disparar e v_fl..v_rr
        # ficam CONGELADAS no último valor. Se o robô estava girando, o
        # diferencial congelado vira yaw_rate constante → o tick (50 Hz)
        # integra um giro INFINITO no mapa com o robô parado. Sem dado novo, a
        # contribuição das rodas é DESCONHECIDA, não "o último valor": zera.
        # (Mesma proteção que o flow tem via flow_timeout → vx=0.)
        if not wheel_fresh:
            vx_wheel = 0.0
            wheel_angular = 0.0

        # --- seleção da TAXA de yaw com degradação graciosa ---
        # Nenhuma das duas IMUs impõe yaw absoluto AQUI: a integração é sempre
        # relativa (ponto-médio, igual ao odom_publisher), as IMUs só trocam a
        # fonte da TAXA (giro × derrapagem da roda). Por isso uma correção manual
        # de direção (yaw_fix) persiste. O heading absoluto da BNO055 entra
        # depois, como empurrão suave — nunca como atribuição.
        yaw_rate, yaw_source, rate_disagree = blend_yaw_rate(
            imu_fresh, imu_yaw_rate, imu2_fresh, imu2_yaw_rate, imu2_rate_weight)
        if yaw_rate is None:
            yaw_rate = wheel_angular
            yaw_source = 'wheel'

        integ_yaw = wrap_pi(self.yaw + 0.5 * yaw_rate * dt)
        self.yaw = wrap_pi(self.yaw + yaw_rate * dt)

        # --- âncora de heading absoluto (BNO055 + magnetômetro) ---
        # Aplicada DEPOIS de integrar e FORA do yaw_rate publicado: o twist do
        # /odom tem que continuar sendo a velocidade angular MEDIDA, não a
        # medida + a correção de deriva (senão o controlador enxerga um giro que
        # não existe). Sobre a pose ela age como a lima que tira o acúmulo.
        heading_corr = 0.0
        if abs_yaw is not None:
            heading_corr = heading_correction(
                self.yaw, abs_yaw, heading_gain, dt, heading_max_rate)
            if heading_corr:
                self.yaw = wrap_pi(self.yaw + heading_corr)

        # --- translação fundida ---
        vx_body, vy_body = fuse_translation(vx_wheel, flow_vx, flow_vy, alpha)

        # --- integra no mundo usando integ_yaw ---
        cy = math.cos(integ_yaw)
        sy = math.sin(integ_yaw)
        self.x += (vx_body * cy - vy_body * sy) * dt
        self.y += (vx_body * sy + vy_body * cy) * dt

        return StepResult(self.x, self.y, self.yaw, yaw_rate,
                          vx_body, vy_body, yaw_source,
                          heading_corr, rate_disagree)
