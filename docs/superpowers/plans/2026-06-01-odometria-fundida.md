# Odometria Fundida Unificada — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fazer o `pose_estimator` virar o nó único de odometria que publica `/odom` + TF `odom→base_link` fundindo rodas + IMU + flow com degradação graciosa, e afinar o slam_toolbox pra confiar no scan matching — consertando a parede fantasma na curva.

**Architecture:** Extrair o núcleo de fusão/integração num módulo puro (`fused_odom.py`, testável sem rclpy, no estilo de `cone_pose_fix.py`). O `pose_estimator` vira um wrapper fino que alimenta esse núcleo e publica `/odom` + TF + os `/trekking/*` de sempre. O `odom_publisher` sai dos launches (seu comportamento "só rodas" é o caso degenerado do núcleo). O `slam.launch.py` ganha gates de travel menores.

**Tech Stack:** ROS2 (rclpy), Python, pytest, colcon. Pacote `ros2_packages/robot_nav`.

**Spec:** `docs/superpowers/specs/2026-06-01-odometria-fundida-design.md`

---

## File Structure

- **Create** `ros2_packages/robot_nav/robot_nav/fused_odom.py` — núcleo puro: cinemática de roda, peso do flow, fusão de translação, seleção de yaw com degradação, integração da pose. Sem rclpy.
- **Create** `ros2_packages/robot_nav/test/test_fused_odom.py` — testes unitários do núcleo.
- **Modify** `ros2_packages/robot_nav/robot_nav/pose_estimator.py` — usar `FusedOdom`; freshness da IMU; remover gate `if not have_yaw`; publicar `/odom` + TF.
- **Modify** `ros2_packages/robot_nav/launch/robot.launch.py` — trocar `odom_publisher` por `pose_estimator` (com calibração de flow + `imu_timeout`).
- **Modify** `ros2_packages/robot_nav/launch/trekking.launch.py` — remover o `pose_estimator` (já está na base); atualizar docstring.
- **Modify** `ros2_packages/robot_nav/launch/slam.launch.py` — gates de travel menores.
- **Não tocar:** `sim.launch.py`, `odom_publisher.py` (fica como referência), `setup.py` (entry point `pose_estimator` já existe).

---

## Task 1: Núcleo puro de fusão (`fused_odom.py`)

**Files:**
- Create: `ros2_packages/robot_nav/robot_nav/fused_odom.py`
- Test: `ros2_packages/robot_nav/test/test_fused_odom.py`

- [ ] **Step 1: Escrever os testes que falham**

Create `ros2_packages/robot_nav/test/test_fused_odom.py`:

```python
import math

import pytest

from robot_nav.fused_odom import (
    FusedOdom,
    flow_alpha,
    fuse_translation,
    wheel_twist,
)


def test_wheel_twist_straight():
    vx, w = wheel_twist(1.0, 1.0, 1.0, 1.0, wheel_base=0.5)
    assert vx == pytest.approx(1.0)
    assert w == pytest.approx(0.0)


def test_wheel_twist_spin_in_place():
    # lado esquerdo recua, direito avança → gira (omega > 0)
    vx, w = wheel_twist(-0.5, 0.5, -0.5, 0.5, wheel_base=0.5)
    assert vx == pytest.approx(0.0)
    assert w == pytest.approx((0.5 - (-0.5)) / 0.5)


def test_flow_alpha_zero_when_stale():
    assert flow_alpha(245.0, q_mid=80.0, q_slope=20.0,
                      flow_age=1.0, flow_timeout=0.5) == 0.0


def test_flow_alpha_high_when_quality_good():
    a = flow_alpha(200.0, q_mid=80.0, q_slope=20.0,
                   flow_age=0.05, flow_timeout=0.5)
    assert a > 0.99


def test_flow_alpha_half_at_qmid():
    a = flow_alpha(80.0, q_mid=80.0, q_slope=20.0,
                   flow_age=0.05, flow_timeout=0.5)
    assert a == pytest.approx(0.5)


def test_fuse_translation_alpha_zero_is_wheel_only():
    vx, vy = fuse_translation(vx_wheel=0.8, flow_vx=0.2, flow_vy=0.1, alpha=0.0)
    assert vx == pytest.approx(0.8)
    assert vy == pytest.approx(0.0)


def test_fuse_translation_alpha_one_is_flow_only():
    vx, vy = fuse_translation(vx_wheel=0.8, flow_vx=0.2, flow_vy=0.1, alpha=1.0)
    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(0.1)


def test_no_imu_uses_wheel_yaw():
    # Sem IMU, girando: yaw integra do diferencial de roda
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'wheel'
    assert r.yaw == pytest.approx(2.0 * 0.1)  # omega=2 rad/s * dt
    assert r.yaw_rate == pytest.approx(2.0)


def test_imu_fresh_uses_imu_yaw_ignoring_wheels():
    # Com IMU fresca, o yaw é o absoluto da IMU mesmo com rodas girando
    fo = FusedOdom(wheel_base=0.5)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=True, imu_yaw=0.7, imu_yaw_rate=0.3,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'imu'
    assert r.yaw == pytest.approx(0.7)
    assert r.yaw_rate == pytest.approx(0.3)


def test_imu_dropout_snaps_to_wheel_from_last_yaw():
    # IMU presente (yaw=1.0), depois cai → integra do último yaw, sem voltar a 0
    fo = FusedOdom(wheel_base=0.5)
    fo.step(dt=0.1, v_fl=0.0, v_fr=0.0, v_rl=0.0, v_rr=0.0,
            imu_fresh=True, imu_yaw=1.0, imu_yaw_rate=0.0,
            flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    r = fo.step(dt=0.1, v_fl=-0.5, v_fr=0.5, v_rl=-0.5, v_rr=0.5,
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    assert r.yaw_source == 'wheel'
    assert r.yaw == pytest.approx(1.0 + 2.0 * 0.1)  # continua de 1.0


def test_degenerate_matches_wheel_only_odom():
    # Sem IMU, sem flow: deve bater com a integração ponto-médio do odom_publisher
    fo = FusedOdom(wheel_base=0.5)
    # avanço com leve giro
    v_fl = v_rl = 0.8
    v_fr = v_rr = 1.0
    dt = 0.1
    r = fo.step(dt=dt, v_fl=v_fl, v_fr=v_fr, v_rl=v_rl, v_rr=v_rr,
                imu_fresh=False, imu_yaw=0.0, imu_yaw_rate=0.0,
                flow_vx=0.0, flow_vy=0.0, alpha=0.0)
    # Espelha odom_publisher: linear=(vr+vl)/2, angular=(vr-vl)/wb, ponto-médio
    v_left = (v_fl + v_rl) / 2.0
    v_right = (v_fr + v_rr) / 2.0
    linear = (v_left + v_right) / 2.0
    angular = (v_right - v_left) / 0.5
    theta_mid = 0.0 + 0.5 * angular * dt
    exp_x = linear * math.cos(theta_mid) * dt
    exp_y = linear * math.sin(theta_mid) * dt
    assert r.x == pytest.approx(exp_x)
    assert r.y == pytest.approx(exp_y)
    assert r.yaw == pytest.approx(angular * dt)
```

- [ ] **Step 2: Rodar os testes e confirmar que falham**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_fused_odom.py -v`
Expected: FAIL com `ModuleNotFoundError: No module named 'robot_nav.fused_odom'` (ou ImportError).

- [ ] **Step 3: Implementar o módulo**

Create `ros2_packages/robot_nav/robot_nav/fused_odom.py`:

```python
#!/usr/bin/env python3
"""Núcleo PURO da odometria fundida (rodas + IMU + flow) com degradação graciosa.

Sem dependência de rclpy — testável isoladamente (estilo cone_pose_fix.py). O nó
`pose_estimator` alimenta este núcleo com velocidades de roda, yaw/freshness da IMU,
velocidade do flow + peso α, e dt; e publica o resultado (/odom + TF + /trekking/*).

Seleção de yaw (degradação graciosa):
  - IMU fresca  → yaw absoluto da IMU.
  - IMU ausente → integra yaw do diferencial de roda (ponto-médio), igual ao
                  odom_publisher antigo. É o caso degenerado.
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
             imu_fresh, imu_yaw, imu_yaw_rate,
             flow_vx, flow_vy, alpha):
        vx_wheel, wheel_angular = wheel_twist(v_fl, v_fr, v_rl, v_rr, self.wheel_base)

        # --- seleção de yaw com degradação graciosa ---
        if imu_fresh:
            # IMU fresca: yaw absoluto. integ_yaw = o próprio yaw da IMU.
            self.yaw = imu_yaw
            integ_yaw = imu_yaw
            yaw_rate = imu_yaw_rate
            yaw_source = 'imu'
        else:
            # Fallback de roda: integra no ponto-médio (igual odom_publisher), depois
            # avança o yaw. Snap parte do último yaw conhecido (decisão B do spec).
            integ_yaw = wrap_pi(self.yaw + 0.5 * wheel_angular * dt)
            self.yaw = wrap_pi(self.yaw + wheel_angular * dt)
            yaw_rate = wheel_angular
            yaw_source = 'wheel'

        # --- translação fundida ---
        vx_body, vy_body = fuse_translation(vx_wheel, flow_vx, flow_vy, alpha)

        # --- integra no mundo usando integ_yaw ---
        cy = math.cos(integ_yaw)
        sy = math.sin(integ_yaw)
        self.x += (vx_body * cy - vy_body * sy) * dt
        self.y += (vx_body * sy + vy_body * cy) * dt

        return StepResult(self.x, self.y, self.yaw, yaw_rate,
                          vx_body, vy_body, yaw_source)
```

- [ ] **Step 4: Rodar os testes e confirmar que passam**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_fused_odom.py -v`
Expected: PASS (11 testes).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/fused_odom.py ros2_packages/robot_nav/test/test_fused_odom.py
git commit -m "fused_odom: nucleo puro de fusao rodas+IMU+flow com degradacao graciosa"
```

---

## Task 2: Integrar `FusedOdom` no `pose_estimator` (+/odom +TF)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/pose_estimator.py`

- [ ] **Step 1: Imports e estado novo**

Em `pose_estimator.py`, trocar a linha de import do geometry_msgs e adicionar tf2/núcleo.

Trocar:
```python
from geometry_msgs.msg import PoseStamped, Vector3Stamped
```
por:
```python
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster

from .fused_odom import FusedOdom, flow_alpha
```

- [ ] **Step 2: Param `imu_timeout`, núcleo, publishers de /odom e TF**

No `__init__`, depois de `self.declare_parameter('base_frame', 'base_link')` (bloco de saída), adicionar a declaração do timeout da IMU:
```python
        self.declare_parameter('imu_timeout', 0.3)   # s — IMU a 50 Hz; >0.3 = ausente
```
E na leitura dos params (junto de `self.base_frame = ...`):
```python
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
```

Substituir o bloco de estado de pose. Trocar:
```python
        # --- Estado ---
        self._lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0          # do BNO055 (rad/s)
        self.have_yaw = False
```
por:
```python
        # --- Estado ---
        self._lock = threading.Lock()
        # A pose (x, y, yaw) vive no núcleo puro FusedOdom.
        self._fused = FusedOdom(self.wheel_base)
        # Última leitura da IMU (None = nunca chegou).
        self._imu_yaw = 0.0
        self._imu_yaw_rate = 0.0
        self._last_imu_wall = None    # rclpy.time.Time
```

Nos publishers, depois de `self.pub_health = ...`, adicionar o /odom padrão e o TF:
```python
        # /odom + TF odom->base_link: o que SLAM/AMCL/Nav2 consomem. Este nó é o
        # ÚNICO dono desse TF agora (odom_publisher saiu dos launches).
        self.pub_odom_std = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
```

- [ ] **Step 3: Reescrever `_on_imu` (timestamp) e `_on_pose_fix` (estado no núcleo)**

Trocar `_on_imu` inteiro por:
```python
    def _on_imu(self, msg: Imu):
        with self._lock:
            self._imu_yaw = _quat_to_yaw(
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            )
            self._imu_yaw_rate = msg.angular_velocity.z
            self._last_imu_wall = self.get_clock().now()
```

Em `_on_pose_fix`, trocar as referências `self.x`/`self.y` pelo estado do núcleo. Trocar:
```python
        with self._lock:
            nx, ny, ok = apply_pose_fix(
                self.x, self.y, dx, dy, self.pose_fix_gain, self.pose_fix_max,
            )
            if ok:
                self.x = nx
                self.y = ny
```
por:
```python
        with self._lock:
            nx, ny, ok = apply_pose_fix(
                self._fused.x, self._fused.y, dx, dy,
                self.pose_fix_gain, self.pose_fix_max,
            )
            if ok:
                self._fused.x = nx
                self._fused.y = ny
```

- [ ] **Step 4: Reescrever `_tick`**

Substituir o método `_tick` inteiro (do `def _tick(self):` até o fim do método, antes de `def main`) por:
```python
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_pub_time).nanoseconds / 1e9
        self.last_pub_time = now
        if dt <= 0.0 or dt > 0.5:
            # Salto de tempo (drift do clock ou pausa). Não integra.
            return

        with self._lock:
            # Freshness da IMU
            if self._last_imu_wall is None:
                imu_age = float('inf')
            else:
                imu_age = (now - self._last_imu_wall).nanoseconds / 1e9
            imu_fresh = imu_age <= self.imu_timeout

            # Idade + peso do flow
            flow_age = float('inf')
            if self._last_flow_wall is not None:
                flow_age = (now - self._last_flow_wall).nanoseconds / 1e9
            alpha = flow_alpha(self.flow_quality, self.q_mid, self.q_slope,
                               flow_age, self.flow_timeout)
            flow_stale = flow_age > self.flow_timeout
            flow_vx = 0.0 if flow_stale else self.flow_vx
            flow_vy = 0.0 if flow_stale else self.flow_vy

            self._last_alpha = alpha
            self._last_flow_age = flow_age

            # Passo de fusão (núcleo puro)
            res = self._fused.step(
                dt,
                self.v_fl, self.v_fr, self.v_rl, self.v_rr,
                imu_fresh, self._imu_yaw, self._imu_yaw_rate,
                flow_vx, flow_vy, alpha,
            )

            # Cache pra slip / twist
            vx_wheel = (self.v_fl + self.v_rl + self.v_fr + self.v_rr) / 4.0
            self.v_wheel_body = vx_wheel
            self.vx_body = res.vx_body
            self.vy_body = res.vy_body

            # Detecta slip (só log/publish)
            slip = vx_wheel - flow_vx if alpha > 0.1 else 0.0
            if alpha > 0.3 and abs(slip) > self.slip_threshold:
                self.get_logger().warn(
                    f'slip detectado: roda={vx_wheel:+.2f} m/s vs flow={flow_vx:+.2f} m/s '
                    f'(α={alpha:.2f}, q={self.flow_quality:.0f})',
                    throttle_duration_sec=1.0,
                )

            x = res.x
            y = res.y
            yaw = res.yaw
            yaw_rate = res.yaw_rate
            yaw_source = res.yaw_source
            slip_out = slip
            quality_out = self.flow_quality

        # ----- diagnóstico do flow -----
        if flow_stale and not self._flow_was_stale:
            self.get_logger().warn(
                f'flow stale (age={flow_age:.2f} s > {self.flow_timeout:.2f} s) — '
                f'pose_estimator usando só rodas',
                throttle_duration_sec=60.0,
            )
        elif not flow_stale and self._flow_was_stale:
            self.get_logger().info('flow voltou')
        self._flow_was_stale = flow_stale

        if alpha < 0.05:
            if self._alpha_low_since is None:
                self._alpha_low_since = now
            else:
                low_dt = (now - self._alpha_low_since).nanoseconds / 1e9
                if low_dt > 2.0:
                    self.get_logger().warn(
                        f'alpha={alpha:.3f} (quality={quality_out:.0f}) há {low_dt:.1f} s — '
                        f'flow contribuindo ~0 na fusão',
                        throttle_duration_sec=60.0,
                    )
        else:
            self._alpha_low_since = None

        # ----- publica -----
        stamp = now.to_msg()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # /trekking/pose (frame odom)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.odom_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_pose.publish(ps)

        # /trekking/odom (twist no body frame)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.vx_body
        od.twist.twist.linear.y = self.vy_body
        od.twist.twist.angular.z = yaw_rate
        self.pub_odom.publish(od)

        # /odom padrão (consumido por SLAM/AMCL/Nav2) + covariâncias
        od_std = Odometry()
        od_std.header.stamp = stamp
        od_std.header.frame_id = self.odom_frame
        od_std.child_frame_id = self.base_frame
        od_std.pose.pose.position.x = x
        od_std.pose.pose.position.y = y
        od_std.pose.pose.orientation.z = qz
        od_std.pose.pose.orientation.w = qw
        od_std.twist.twist.linear.x = self.vx_body
        od_std.twist.twist.linear.y = self.vy_body
        od_std.twist.twist.angular.z = yaw_rate
        od_std.pose.covariance[0] = 0.05
        od_std.pose.covariance[7] = 0.05
        # yaw menos confiável no fallback de roda → AMCL/Nav confiam menos
        od_std.pose.covariance[35] = 0.10 if yaw_source == 'imu' else 0.5
        od_std.twist.covariance[0] = 0.01
        od_std.twist.covariance[35] = 0.05
        self.pub_odom_std.publish(od_std)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        self.pub_slip.publish(Float32(data=float(slip_out)))

        # /trekking/health
        health = {
            'flow_stale': bool(flow_stale),
            'flow_age':   round(flow_age, 3) if flow_age != float('inf') else None,
            'alpha':      round(alpha, 3),
            'quality':    int(quality_out),
            'yaw_source': yaw_source,
        }
        self.pub_health.publish(String(data=json.dumps(health, sort_keys=True)))
```

- [ ] **Step 5: Atualizar a docstring do módulo**

No topo do arquivo, trocar a linha da docstring:
```
NÃO publica TF (`odom→base_link` continua sendo do odom_publisher). O
```
por:
```
Publica /odom + TF (`odom→base_link`) — é o nó único de odometria agora. O
```

- [ ] **Step 6: Build + testes de regressão**

Run:
```bash
cd /home/rbe-luis/Workspace/Controle_robo_web
colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav wheel_msgs
source install/setup.bash
python -m pytest ros2_packages/robot_nav/test/ -v
```
Expected: build OK; PASS em `test_fused_odom.py` (11) + `test_cone_pose_fix.py` (9 já existentes). Nenhum import quebrado.

- [ ] **Step 7: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/pose_estimator.py
git commit -m "pose_estimator: vira no unico de odometria (FusedOdom + /odom + TF, freshness IMU)"
```

---

## Task 3: Wiring dos launches (aposentar odom_publisher)

**Files:**
- Modify: `ros2_packages/robot_nav/launch/robot.launch.py`
- Modify: `ros2_packages/robot_nav/launch/trekking.launch.py`

- [ ] **Step 1: `robot.launch.py` — trocar odom_publisher por pose_estimator**

Substituir o nó `odom_publisher` (bloco `odom_publisher = Node(...)`, ~linhas 98-109) por:
```python
    pose_estimator = Node(
        package='robot_nav',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            # wheel_base aqui é a bitola EFETIVA (calibrada no skid-steer) usada
            # pra estimar o yaw de roda quando não há IMU. Default geométrico até
            # calibrar (ver plano, Task 5).
            'wheel_base': LaunchConfiguration('wheel_base'),
            'left_wheel_sign': LaunchConfiguration('left_wheel_sign'),
            'right_wheel_sign': LaunchConfiguration('right_wheel_sign'),
            # Janela de freshness da IMU: sem /imu/data nesse tempo → cai pro
            # yaw de roda (degradação graciosa).
            'imu_timeout': 0.3,
            # Calibração do PMW3901 → body frame (movida do trekking.launch.py:
            # frente entra por dy negativo do sensor). Vale pra TODOS os modos
            # agora que a fusão é a odometria base.
            'flow_swap_xy': True,
            'flow_x_sign': -1.0,
        }],
    )
```

E na lista de retorno do `LaunchDescription`, trocar `odom_publisher,` por `pose_estimator,`.

- [ ] **Step 2: `robot.launch.py` — atualizar o comentário-sumário do topo**

Na docstring do topo, trocar a linha:
```
  3. odom_publisher       (4 RPMs → /odom + TF odom→base_link)
```
por:
```
  3. pose_estimator       (funde 4 RPMs + IMU + flow → /odom + TF odom→base_link,
                           com degradação graciosa; tambem publica /trekking/*)
```

- [ ] **Step 3: `trekking.launch.py` — remover o pose_estimator (já está na base)**

Remover o bloco `pose_estimator = Node(...)` inteiro (linhas ~43-59), remover o `flow_height_arg` (linhas ~30-33, só era usado por ele) e tirar `flow_height_arg,` e `pose_estimator,` da lista de retorno do `LaunchDescription`.

Atualizar a docstring do topo: trocar
```
Sobe os 3 nós que compõem o controle ponto-a-ponto da competição:
  1. pose_estimator    — funde IMU + flow + rodas em /trekking/pose
  2. cone_detector     — clusteriza /scan + /trekking/pose → /trekking/cones
  3. trekking_runner   — máquina de estado IDLE/RECORD/PLAY com PID

Pré-requisito: robot.launch.py já está rodando (mega_bridge + URDF +
cmd_vel_to_wheels), e o LiDAR está publicando /scan.

Observação importante: o `odom_publisher` do robot.launch.py continua
rodando e publicando o TF `odom→base_link` baseado só nas rodas. Isso
serve pro restante do sistema (rviz, etc.). O modo trekking ignora esse
TF e usa /trekking/pose direto — mais preciso pelo flow.
```
por:
```
Sobe os 2 nós específicos do controle ponto-a-ponto da competição:
  1. cone_detector     — clusteriza /scan + /trekking/pose → /trekking/cones
  2. trekking_runner   — máquina de estado IDLE/RECORD/PLAY com PID

Pré-requisito: robot.launch.py já está rodando — ele sobe o `pose_estimator`
(que publica /trekking/pose + /odom + TF) além de mega_bridge + URDF +
cmd_vel_to_wheels, e o LiDAR está publicando /scan. O trekking consome
/trekking/pose direto (mais preciso pelo flow), sem depender do TF.
```

- [ ] **Step 4: Validar que os launches importam/descrevem sem erro**

Run:
```bash
cd /home/rbe-luis/Workspace/Controle_robo_web && source install/setup.bash
python -c "from launch import LaunchDescription; import importlib.util, sys
for f in ['robot','trekking','slam']:
    spec = importlib.util.spec_from_file_location(f, f'ros2_packages/robot_nav/launch/{f}.launch.py')
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    assert isinstance(m.generate_launch_description(), LaunchDescription), f
    print(f, 'OK')"
```
Expected: `robot OK`, `trekking OK`, `slam OK` (sem exceção).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/launch/robot.launch.py ros2_packages/robot_nav/launch/trekking.launch.py
git commit -m "launch: pose_estimator vira odom base (robot.launch); remove do trekking.launch"
```

---

## Task 4: Afinar o slam_toolbox (yaw por scan matching)

**Files:**
- Modify: `ros2_packages/robot_nav/launch/slam.launch.py`

- [ ] **Step 1: Baixar os gates de travel + scan matching explícito**

No dict `parameters` do nó `slam` (linhas ~36-50), adicionar as chaves abaixo (depois de `'transform_timeout': 0.5,`):
```python
            # Afinação contra "parede fantasma" na curva: processa scan em
            # incrementos pequenos pra o matcher (Ceres) convergir mesmo com a
            # semente de yaw de roda ruim — em vez de esperar 0.5 rad (~28°) e
            # confiar no odom no meio do giro. Ver spec 2026-06-01-odometria-fundida.
            'use_scan_matching': True,
            'minimum_travel_distance': 0.15,
            'minimum_travel_heading': 0.12,
            'scan_buffer_size': 20,
```

- [ ] **Step 2: Validar a launch description**

Run:
```bash
cd /home/rbe-luis/Workspace/Controle_robo_web && source install/setup.bash
python -c "import importlib.util
spec = importlib.util.spec_from_file_location('slam','ros2_packages/robot_nav/launch/slam.launch.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
m.generate_launch_description(); print('slam OK')"
```
Expected: `slam OK`.

- [ ] **Step 3: Commit**

```bash
git add ros2_packages/robot_nav/launch/slam.launch.py
git commit -m "slam.launch: baixa gates de travel + scan matching explicito (corrige parede fantasma)"
```

---

## Task 5: Validação no robô + calibração do wheel_base (HANDS-ON — gated)

> **IMPORTANTE:** Esta task move o robô. Anunciar e **aguardar o "pode" do usuário**
> antes de cada janela de teste (ver memória `feedback_announce_before_test`). Deploy
> na Pi via git (memória `project_pi_git_desync`): `git fetch && git reset --hard origin/main`.

- [ ] **Step 1: Deploy na Pi**

No PC dev: `git push` (do branch, após merge na main conforme fluxo). Na Pi:
```bash
cd ~/workspace/Controle_robo_web && git fetch && git reset --hard origin/main
```

- [ ] **Step 2: Smoke do /odom (sem mover — rodas no ar ou parado)**

Subir `./launch.sh --slam` na Pi e verificar (read-only):
```bash
ros2 topic hz /odom            # deve publicar ~50 Hz
ros2 topic echo --once /odom | grep -A4 orientation
ros2 run tf2_ros tf2_echo odom base_link   # TF presente, um publicador só
```
Expected: `/odom` publicando, um único publicador de `odom→base_link`. Sem IMU agora, `/trekking/health` deve mostrar `yaw_source: wheel`.

- [ ] **Step 3: Calibrar wheel_base efetivo (rotação pura — AGUARDAR "pode")**

Com o fallback de roda (sem IMU), zerar a pose, comandar **uma volta completa** marcada no chão (360° reais) e ler o yaw integrado:
```bash
ros2 topic echo /odom --field pose.pose.orientation
```
Calcular `wheel_base_eff = wheel_base_atual · (yaw_integrado_em_360 / (2π))` e repetir
até o yaw integrado bater com 360°. (Skid-steer: a bitola efetiva > 0.50 geométrica.)

- [ ] **Step 4: Gravar o valor calibrado**

Atualizar o default em `robot.launch.py`:
```python
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base', default_value='<VALOR_CALIBRADO>',
        description='Bitola EFETIVA (calibrada no skid-steer) entre centros L-R, m'
    )
```
Commit:
```bash
git add ros2_packages/robot_nav/launch/robot.launch.py
git commit -m "robot.launch: wheel_base efetivo calibrado=<VALOR> (yaw de roda sem IMU)"
```

- [ ] **Step 5: Teste de mapeamento (curva — AGUARDAR "pode")**

`./launch.sh --slam`, dirigir incluindo curvas, observar o mapa na UI web. Critério:
a curva não deve mais gerar parede fantasma (slam_toolbox afinado + scan matching).
Se ainda houver deriva no Nav, avaliar a Fase 2 (nó de odometria por LiDAR) do spec.

---

## Self-Review

**Cobertura do spec:**
- Nó único de odometria + /odom + TF → Task 2 ✓
- Degradação graciosa (yaw IMU→roda; flow watchdog) → Task 1 (núcleo) + Task 2 (freshness) ✓
- Decisão A (aposentar odom_publisher do launch) → Task 3 ✓
- Decisão B (snap duro na queda de IMU) → Task 1 (`test_imu_dropout_snaps...`) ✓
- Calibração wheel_base → Task 5 ✓
- Afinação slam_toolbox → Task 4 ✓
- Calibração de flow movida pro base → Task 3 Step 1 ✓
- Fase 2 (LiDAR-odom) → fora de escopo, registrado no spec ✓
- Testes (cenários do spec) → Task 1 cobre sem-IMU/com-IMU/flow-stale/degenerado/dropout ✓

**Consistência de tipos:** `FusedOdom.step(...)` e `flow_alpha(...)`/`wheel_twist(...)`/
`fuse_translation(...)` têm assinaturas idênticas entre Task 1 (def), os testes e o uso
no `_tick` (Task 2). `StepResult` expõe `.x .y .yaw .yaw_rate .vx_body .vy_body
.yaw_source` — todos consumidos no `_tick`.

**Sem placeholders:** exceto `<VALOR_CALIBRADO>` na Task 5, que é um resultado de
medição hands-on (não um detalhe omitido) — explicitamente marcado.
