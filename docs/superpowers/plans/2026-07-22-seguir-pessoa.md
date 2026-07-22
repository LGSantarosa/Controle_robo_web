# Seguir pessoa (tap-to-track lidar) — Plano de Implementação

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Modo "seguir pessoa": toque na cara → o robô anda atrás da pessoa mantendo ~1,5 m (só lidar), com todas as seguranças de hoje intactas.

**Architecture:** Nó novo `person_follower` (lógica pura testável + cola ROS no molde do `motion_guard`) calcula a velocidade DESEJADA rumo à pessoa e publica em `follow_person_vel`, que entra no `twist_mux_auto` (mesma porta do `path_follower`). Toda a cadeia a jusante (motion_guard → collision_monitor → unstuck → E-stop) processa igual à navegação por pontos. O `motion_guard` publica os clusters-pessoa que já calcula; o `app.py` orquestra pausa/retomada da rota e o comando start/stop; a cara (iPad :7000) tem os botões e as falas.

**Tech Stack:** Python 3, ROS 2 Jazzy (rclpy), pytest, Flask (controle_web :5000 e face_web :7000), JS ES5 (face.js), gTTS pt-BR (mp3).

## Global Constraints

- **Segurança idêntica à de hoje:** `follow_person_vel` NÃO fura nada — passa por `twist_mux_auto → motion_guard → collision_monitor → mux final`. Nunca publicar direto em `cmd_vel`/`auto_vel`/`unstuck_vel`.
- **Distância mantida:** `stop_dist = 1.5 m` (piso rígido). Nunca andar com o alvo fora do cone frontal. Não recuar ativamente.
- **`wz_cap = 2.4`** perto de gente (reuso do cap do slowing do guard). **`vx_max = 0.25`**.
- **Lógica pura sem ROS** numa classe (padrão `MotionGuard`/`GuardConfig`); o `main()` é só cola (`# pragma: no cover`), validado no sim.
- **Testes com relógio travado na fonte** (não misturar ROS clock com `time.monotonic()` — bug já cometido no unstuck).
- **Frame de trabalho = odom** (associação estável enquanto o robô anda): clusters e alvo em `(cx, cy)` no odom; controle converte pra bearing/dist relativo usando a pose.
- **Sim antes do real** (regra do projeto): `bin/teleop-pernas` no `sala_grande`. Real só valida, dono presente, mão no E-stop.
- **Sem rodapé Co-Authored-By nos commits** (convenção do repo).
- Feature nova → **branch própria** (não misturar na `motion-guard-release-corredor`).

---

## Estrutura de arquivos

- **Novo:** `ros2_packages/robot_nav/robot_nav/person_follower.py` — `FollowConfig`, `PersonFollower` (lógica pura) + `FollowFaceFile` + `main()` (cola ROS).
- **Novo:** `ros2_packages/robot_nav/test/test_person_follower.py` — testes da lógica pura.
- **Novo:** `face_web/static/seguir_inicio.mp3`, `face_web/static/nao_te_vejo.mp3` + `face_web/tools/gen_tts.py` (gerador gTTS, se ainda não houver).
- **Edita:** `ros2_packages/robot_nav/setup.py` (entry point).
- **Edita:** `ros2_packages/robot_nav/robot_nav/motion_guard.py` (publisher `follow_person_targets`, atrás de param).
- **Edita:** `ros2_packages/robot_nav/config/twist_mux_auto.yaml` (entrada `follow_person_vel`).
- **Edita:** `ros2_packages/robot_nav/launch/robot.launch.py` + `launch/sim.launch.py` (sobe o nó).
- **Edita:** `controle_web/app.py` + `controle_web/map_bridge.py` (endpoint `/follow`, pause/resume, pub `follow_cmd`, sub `follow_person_state`, botão GUI).
- **Edita:** `face_web/face_app.py` + `face_web/static/face.js` (botões Seguir/Parar, falas, leitura do estado).

---

## Task 1: Núcleo — `FollowConfig` + aquisição do alvo

**Files:**
- Create: `ros2_packages/robot_nav/robot_nav/person_follower.py`
- Test: `ros2_packages/robot_nav/test/test_person_follower.py`

**Interfaces:**
- Consumes: nada (primeira task).
- Produces:
  - `Target = namedtuple('Target', 'cx cy')` — centróide da pessoa no frame odom.
  - `FollowConfig` (dataclass) com os defaults do §7 do spec.
  - `PersonFollower(cfg)` com atributos `.state: str` (`'idle'|'following'|'lost'|'ending'`), `.target: Target|None`, e método `.acquire(clusters, pose) -> Target|None` (clusters = `list[(cx,cy)]` odom; pose = `(rx,ry,ryaw)`). Retorna o cluster mais PRÓXIMO dentro de `acquire_range` e cone `±acquire_cone_deg/2`, ou `None`.
  - Helper `_rel(cx, cy, pose) -> (dist, bearing_deg)` (bearing 0 = frente, + = esquerda, em graus, wrap ±180).

- [ ] **Step 1: Escrever o teste que falha**

```python
"""Testes da lógica pura do person_follower (sem ROS)."""
import math
from robot_nav.person_follower import FollowConfig, PersonFollower, Target, _rel

POSE = (0.0, 0.0, 0.0)   # robô na origem olhando +x (frame odom)

def _pf(**kw):
    return PersonFollower(FollowConfig(**kw))

def test_rel_bearing_frente_esquerda_direita():
    d, b = _rel(2.0, 0.0, POSE); assert abs(d - 2.0) < 1e-6 and abs(b) < 1e-6
    _, b = _rel(2.0, 2.0, POSE); assert abs(b - 45.0) < 1e-6
    _, b = _rel(2.0, -2.0, POSE); assert abs(b + 45.0) < 1e-6

def test_acquire_pega_o_mais_proximo_no_cone():
    pf = _pf(acquire_range=3.0, acquire_cone_deg=60.0)
    clusters = [(2.5, 0.0), (1.2, 0.2), (2.0, 5.0)]  # 2º é o mais perto; 3º fora do cone
    t = pf.acquire(clusters, POSE)
    assert t == Target(1.2, 0.2)

def test_acquire_none_se_fora_do_alcance_ou_cone():
    pf = _pf(acquire_range=3.0, acquire_cone_deg=60.0)
    assert pf.acquire([(4.0, 0.0)], POSE) is None          # longe
    assert pf.acquire([(1.0, 3.0)], POSE) is None          # fora do cone (~72°)
    assert pf.acquire([], POSE) is None                    # vazio
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -v`
Expected: FAIL (ImportError: cannot import name 'FollowConfig').

- [ ] **Step 3: Implementação mínima**

```python
"""person_follower — modo 'seguir pessoa' (tap-to-track por lidar).

Lógica PURA (classe PersonFollower/FollowConfig) testável sem ROS, no molde
do motion_guard. O main() é só cola ROS (# pragma: no cover), validado no sim.
Frame de trabalho = odom: clusters e alvo em (cx,cy); o controle converte pra
bearing/dist relativo usando a pose. A velocidade de saída é DESEJO — a
segurança (guard/collision/unstuck/E-stop) é aplicada a JUSANTE no pipeline.
"""
import math
from collections import namedtuple
from dataclasses import dataclass

Target = namedtuple('Target', 'cx cy')


@dataclass
class FollowConfig:
    stop_dist: float = 1.5
    stop_hyst: float = 0.2
    vx_max: float = 0.25
    wz_cap: float = 2.4
    wz_kp: float = 2.0            # ganho do giro (rad/s por rad de erro), antes do cap
    face_deadband_deg: float = 8.0
    drive_align_deg: float = 20.0
    acquire_cone_deg: float = 60.0
    acquire_range: float = 3.0
    assoc_gate: float = 0.6
    lost_grace: float = 1.0
    lost_timeout: float = 12.0


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _rel(cx: float, cy: float, pose):
    """(dist, bearing_deg) do ponto odom (cx,cy) relativo ao robô.
    bearing 0 = frente, + = esquerda."""
    rx, ry, ryaw = pose
    dx, dy = cx - rx, cy - ry
    dist = math.hypot(dx, dy)
    bearing = math.degrees(_wrap_rad(math.atan2(dy, dx) - ryaw))
    return dist, bearing


def _wrap_rad(rad: float) -> float:
    return (rad + math.pi) % (2 * math.pi) - math.pi


class PersonFollower:
    def __init__(self, cfg: FollowConfig):
        self.cfg = cfg
        self.state = 'idle'
        self.target = None

    def acquire(self, clusters, pose):
        cfg = self.cfg
        best, best_d = None, math.inf
        for cx, cy in clusters:
            d, b = _rel(cx, cy, pose)
            if d <= cfg.acquire_range and abs(b) <= cfg.acquire_cone_deg / 2 and d < best_d:
                best, best_d = Target(cx, cy), d
        return best
```

- [ ] **Step 4: Rodar e ver passar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -v`
Expected: PASS (3 testes).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/test/test_person_follower.py
git commit -m "feat(person_follower): nucleo + aquisicao do alvo (cone/alcance)"
```

---

## Task 2: Associação do alvo quadro-a-quadro

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/person_follower.py`
- Test: `ros2_packages/robot_nav/test/test_person_follower.py`

**Interfaces:**
- Consumes: `Target`, `FollowConfig`, `PersonFollower` da Task 1.
- Produces: `PersonFollower.associate(clusters) -> Target|None` — dado `self.target`, casa com o cluster mais próximo dentro de `assoc_gate` (metros no odom); atualiza e retorna o novo `Target`, ou `None` se nenhum cluster dentro do gate.

- [ ] **Step 1: Escrever o teste que falha**

```python
def test_associate_segue_salto_pequeno():
    pf = _pf(assoc_gate=0.6)
    pf.target = Target(2.0, 0.0)
    t = pf.associate([(2.3, 0.1), (5.0, 5.0)])   # 0.32m de salto < gate
    assert t == Target(2.3, 0.1) and pf.target == Target(2.3, 0.1)

def test_associate_none_se_salto_grande():
    pf = _pf(assoc_gate=0.6)
    pf.target = Target(2.0, 0.0)
    assert pf.associate([(3.0, 0.0)]) is None    # 1.0m > gate
    assert pf.associate([]) is None
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k associate -v`
Expected: FAIL (AttributeError: 'PersonFollower' object has no attribute 'associate').

- [ ] **Step 3: Implementação mínima** (adicionar método na classe)

```python
    def associate(self, clusters):
        if self.target is None:
            return None
        tx, ty = self.target
        best, best_d = None, self.cfg.assoc_gate
        for cx, cy in clusters:
            d = math.hypot(cx - tx, cy - ty)
            if d <= best_d:
                best, best_d = Target(cx, cy), d
        if best is not None:
            self.target = best
        return best
```

- [ ] **Step 4: Rodar e ver passar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k associate -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/test/test_person_follower.py
git commit -m "feat(person_follower): associacao quadro-a-quadro por proximidade (gate)"
```

---

## Task 3: Lei de controle (girar encarando + andar mantendo 1,5 m)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/person_follower.py`
- Test: `ros2_packages/robot_nav/test/test_person_follower.py`

**Interfaces:**
- Consumes: `FollowConfig`, `_rel` da Task 1.
- Produces: `PersonFollower.control(dist, bearing_deg) -> (vx, wz)`. Regras:
  - `wz`: 0 se `|bearing| < face_deadband_deg`; senão `clamp(wz_kp * bearing_rad, ±wz_cap)`.
  - `vx`: só > 0 se `|bearing| < drive_align_deg`. Histerese via `self._driving`: começa a andar quando `dist > stop_dist + stop_hyst`, para quando `dist <= stop_dist`. Quando anda, `vx = clamp(vx_max * (dist - stop_dist), 0, vx_max)`... na prática satura rápido; usar `vx = min(vx_max, max(0, dist - stop_dist))` (ganho 1). Nunca recua (vx ≥ 0).
  - Estado interno `self._driving: bool` (init False).

- [ ] **Step 1: Escrever o teste que falha**

```python
def test_control_encara_sem_andar_se_desalinhado():
    pf = _pf()
    vx, wz = pf.control(dist=3.0, bearing_deg=40.0)   # fora do drive_align 20°
    assert vx == 0.0 and wz > 0.0                     # gira p/ esquerda, não anda

def test_control_wz_zero_na_zona_morta_e_cap():
    pf = _pf(face_deadband_deg=8.0, wz_cap=2.4)
    assert pf.control(3.0, 5.0)[1] == 0.0             # dentro da zona morta
    assert pf.control(3.0, 179.0)[1] == 2.4           # satura no cap (esq)
    assert pf.control(3.0, -179.0)[1] == -2.4         # satura no cap (dir)

def test_control_anda_alinhado_e_para_em_1_5m():
    pf = _pf(stop_dist=1.5, stop_hyst=0.2, vx_max=0.25)
    vx, wz = pf.control(dist=3.0, bearing_deg=0.0)    # longe e alinhado
    assert 0.0 < vx <= 0.25 and wz == 0.0
    # aproxima até 1.4m -> para (histerese: parou abaixo de stop_dist)
    pf._driving = True
    vx, _ = pf.control(dist=1.4, bearing_deg=0.0)
    assert vx == 0.0

def test_control_histerese_nao_pulsa_em_1_5m():
    pf = _pf(stop_dist=1.5, stop_hyst=0.2)
    pf._driving = False
    assert pf.control(1.6, 0.0)[0] == 0.0             # dentro de stop+hyst, parado segue parado
    assert pf.control(1.8, 0.0)[0] > 0.0              # acima de stop+hyst -> anda
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k control -v`
Expected: FAIL (AttributeError: no attribute 'control').

- [ ] **Step 3: Implementação mínima** (adicionar no `__init__`: `self._driving = False`; e o método)

```python
    def control(self, dist, bearing_deg):
        cfg = self.cfg
        # --- giro: encara o alvo ---
        if abs(bearing_deg) < cfg.face_deadband_deg:
            wz = 0.0
        else:
            wz = math.radians(bearing_deg) * cfg.wz_kp
            wz = max(-cfg.wz_cap, min(cfg.wz_cap, wz))
        # --- avanço: mantém stop_dist, com histerese ---
        if self._driving:
            if dist <= cfg.stop_dist:
                self._driving = False
        else:
            if dist > cfg.stop_dist + cfg.stop_hyst:
                self._driving = True
        aligned = abs(bearing_deg) < cfg.drive_align_deg
        if self._driving and aligned:
            vx = min(cfg.vx_max, max(0.0, dist - cfg.stop_dist))
        else:
            vx = 0.0
        return vx, wz
```

- [ ] **Step 4: Rodar e ver passar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k control -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/test/test_person_follower.py
git commit -m "feat(person_follower): lei de controle (encara + anda mantendo 1,5m, histerese)"
```

---

## Task 4: Máquina de estados / `tick()` (start, following, lost, timeout, stop)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/person_follower.py`
- Test: `ros2_packages/robot_nav/test/test_person_follower.py`

**Interfaces:**
- Consumes: `acquire`, `associate`, `control` das Tasks 1-3.
- Produces:
  - `PersonFollower.start()` — arma o pedido (`self._start_req = True`).
  - `PersonFollower.stop()` — de `following|lost` → `ending`.
  - `PersonFollower.tick(t, clusters, pose) -> (vx, wz)` — avança a máquina UMA vez com o relógio `t` (segundos, travado na fonte). Efeitos:
    - `idle` + start pedido: `acquire`; achou → `following`, seta `self.just_spoke='start'`, `_last_seen=t`; não achou → segue `idle`, `_start_req=False`, `self.no_target=True`.
    - `following`: `associate`; casou → `control` (retorna vx,wz), `_last_seen=t`; não casou por `>lost_grace` → `lost`, `_lost_since=t`, `self.just_spoke='lost'`.
    - `lost`: `associate`; casou → `following`; senão vx=wz=0; `t-_lost_since>lost_timeout` → `ending`.
    - `ending`: vx=wz=0. O nó lê `ending`, retoma rota, e chama `.reset()` → `idle`.
  - `PersonFollower.reset()` — volta a `idle`, limpa target/flags.
  - Flags de saída pro nó: `self.just_spoke: str|None` (`'start'|'lost'|None`, consumida e zerada pelo nó), `self.state`, `self.target`.

- [ ] **Step 1: Escrever o teste que falha**

```python
def _clusters_at(dist, bearing_deg=0.0, pose=POSE):
    # devolve um cluster odom a (dist, bearing) do robô em `pose`
    rx, ry, ryaw = pose
    a = ryaw + math.radians(bearing_deg)
    return [(rx + dist * math.cos(a), ry + dist * math.sin(a))]

def test_tick_start_trava_e_fala():
    pf = _pf()
    pf.start()
    vx, wz = pf.tick(0.0, _clusters_at(2.5), POSE)
    assert pf.state == 'following' and pf.just_spoke == 'start' and pf.target is not None

def test_tick_start_sem_ninguem_fica_idle():
    pf = _pf()
    pf.start()
    pf.tick(0.0, [], POSE)
    assert pf.state == 'idle' and pf.no_target is True

def test_tick_perde_alvo_vira_lost_e_fala():
    pf = _pf(lost_grace=1.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE)
    pf.tick(0.5, [], POSE)                      # sumiu, mas dentro do grace
    assert pf.state == 'following'
    pf.tick(1.6, [], POSE)                      # >lost_grace sem match
    assert pf.state == 'lost' and pf.just_spoke == 'lost'

def test_tick_lost_reaparece_volta_following():
    pf = _pf(lost_grace=1.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE); pf.tick(1.6, [], POSE)
    assert pf.state == 'lost'
    pf.tick(2.0, _clusters_at(2.4), POSE)       # reaparece perto do último
    assert pf.state == 'following'

def test_tick_lost_timeout_vira_ending():
    pf = _pf(lost_grace=1.0, lost_timeout=12.0)
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE); pf.tick(1.6, [], POSE)
    pf.tick(1.6 + 12.1, [], POSE)
    assert pf.state == 'ending'

def test_stop_de_following_vai_ending_e_reset_volta_idle():
    pf = _pf()
    pf.start(); pf.tick(0.0, _clusters_at(2.5), POSE)
    pf.stop(); assert pf.state == 'ending'
    pf.reset(); assert pf.state == 'idle' and pf.target is None
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k "tick or stop" -v`
Expected: FAIL (no attribute 'tick').

- [ ] **Step 3: Implementação mínima** (no `__init__` add: `self._start_req=False; self.just_spoke=None; self.no_target=False; self._last_seen=0.0; self._lost_since=0.0`; e os métodos)

```python
    def start(self):
        self._start_req = True
        self.no_target = False

    def stop(self):
        if self.state in ('following', 'lost'):
            self.state = 'ending'

    def reset(self):
        self.state = 'idle'
        self.target = None
        self._start_req = False
        self._driving = False
        self.just_spoke = None
        self.no_target = False

    def tick(self, t, clusters, pose):
        if self.state == 'idle':
            if self._start_req:
                self._start_req = False
                tgt = self.acquire(clusters, pose)
                if tgt is not None:
                    self.target = tgt
                    self.state = 'following'
                    self.just_spoke = 'start'
                    self._last_seen = t
                else:
                    self.no_target = True
            return 0.0, 0.0

        if self.state == 'following':
            m = self.associate(clusters)
            if m is not None:
                self._last_seen = t
                dist, bearing = _rel(m.cx, m.cy, pose)
                return self.control(dist, bearing)
            if t - self._last_seen > self.cfg.lost_grace:
                self.state = 'lost'
                self._lost_since = t
                self.just_spoke = 'lost'
            return 0.0, 0.0

        if self.state == 'lost':
            m = self.associate(clusters)
            if m is not None:
                self.state = 'following'
                self._last_seen = t
                dist, bearing = _rel(m.cx, m.cy, pose)
                return self.control(dist, bearing)
            if t - self._lost_since > self.cfg.lost_timeout:
                self.state = 'ending'
            return 0.0, 0.0

        # ending
        return 0.0, 0.0
```

- [ ] **Step 4: Rodar e ver passar** (a suíte inteira)

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -v`
Expected: PASS (todos).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/test/test_person_follower.py
git commit -m "feat(person_follower): maquina de estados tick (start/following/lost/timeout/stop)"
```

---

## Task 5: Arquivo de estado pra cara (`FollowFaceFile`)

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/person_follower.py`
- Test: `ros2_packages/robot_nav/test/test_person_follower.py`

**Interfaces:**
- Consumes: nada novo (padrão copiado do `FaceStateFile` do motion_guard).
- Produces: `FollowFaceFile(path='/tmp/person_follow_face.json', min_period=0.2)` com `.update(t, state, speak=None, bearing_deg=None) -> bool`. Grava JSON atômico `{ts, follow_state, speak, cbear_deg}`. `speak` é `'start'|'lost'|None` (evento de fala one-shot). I/O nunca propaga (try/except OSError → `last_error`).

- [ ] **Step 1: Escrever o teste que falha**

```python
import json, os, tempfile
from robot_nav.person_follower import FollowFaceFile

def test_followfacefile_grava_estado_e_fala(tmp_path):
    p = str(tmp_path / 'ff.json')
    ff = FollowFaceFile(path=p, min_period=0.0)
    assert ff.update(1.0, 'following', speak='start', bearing_deg=12) is True
    d = json.load(open(p))
    assert d['follow_state'] == 'following' and d['speak'] == 'start' and d['cbear_deg'] == 12

def test_followfacefile_io_error_nao_propaga():
    ff = FollowFaceFile(path='/proc/nao_pode/x.json', min_period=0.0)
    assert ff.update(1.0, 'following') is False and ff.last_error is not None
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -k followface -v`
Expected: FAIL (ImportError FollowFaceFile).

- [ ] **Step 3: Implementação mínima** (adicionar classe no módulo)

```python
import json
import os


class FollowFaceFile:
    """Estado do seguir pro face_web. Atômico, ≤5Hz, I/O nunca propaga."""

    def __init__(self, path: str = '/tmp/person_follow_face.json',
                 min_period: float = 0.2):
        self.path = path
        self.min_period = min_period
        self.last_error = None
        self._last_write_t = -math.inf

    def update(self, t, state, speak=None, bearing_deg=None) -> bool:
        # fala (speak) sempre grava na hora; estado periódico respeita min_period
        if speak is None and t - self._last_write_t < self.min_period:
            return False
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'ts': round(t, 3), 'follow_state': state,
                           'speak': speak, 'cbear_deg': bearing_deg}, f)
            os.replace(tmp, self.path)
        except OSError as e:
            self.last_error = str(e)
            return False
        self._last_write_t = t
        return True
```

- [ ] **Step 4: Rodar e ver passar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_person_follower.py -v`
Expected: PASS (suíte inteira).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/test/test_person_follower.py
git commit -m "feat(person_follower): FollowFaceFile (estado+fala one-shot pra cara)"
```

---

## Task 6: Cola ROS — nó `person_follower` + entry point

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/person_follower.py` (adicionar `main()`)
- Modify: `ros2_packages/robot_nav/setup.py:39` (entry point)

**Interfaces:**
- Consumes: `PersonFollower`, `FollowConfig`, `FollowFaceFile`, `_rel`.
- Produces: nó ROS `person_follower` que:
  - assina `follow_person_targets` (`std_msgs/Float32MultiArray`, layout `[cx0,cy0,cx1,cy1,...]`), `follow_cmd` (`std_msgs/String`, `"START"|"STOP"`), `odom` (pose).
  - publica `follow_person_vel` (`geometry_msgs/Twist`), `follow_person_state` (`std_msgs/String`, latched).
  - roda `tick()` a ~10 Hz com o relógio `self.get_clock().now()`; escreve `FollowFaceFile`; publica vel só em `following`; em `ending` publica state e chama `reset()`.

**Nota:** cola de I/O → marcar `# pragma: no cover` e validar no sim (Task 11). Sem teste unit.

- [ ] **Step 1: Adicionar `main()`** (esqueleto; segue o padrão de `motion_guard.main()` — EventsExecutor, QoS latched pro state, timer 10 Hz)

```python
def main(args=None):  # pragma: no cover - cola de I/O, validar no sim
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Float32MultiArray
    from nav_msgs.msg import Odometry

    class PersonFollowerNode(Node):
        def __init__(self):
            super().__init__('person_follower')
            self.declare_parameter('follow_enabled', False)
            # (declarar os demais knobs do §7 do spec e montar FollowConfig)
            self.pf = PersonFollower(FollowConfig())
            self.face = FollowFaceFile()
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(Twist, 'follow_person_vel', 10)
            self.pub_state = self.create_publisher(String, 'follow_person_state', latched)
            self._targets = []
            self._pose = (0.0, 0.0, 0.0)
            self.create_subscription(Float32MultiArray, 'follow_person_targets',
                                     self._on_targets, 10)
            self.create_subscription(String, 'follow_cmd', self._on_cmd, 10)
            self.create_subscription(Odometry, 'odom', self._on_odom, 10)
            self.create_timer(0.1, self._tick)
            self._publish_state()

        def _on_targets(self, msg):
            d = list(msg.data)
            self._targets = [(d[i], d[i+1]) for i in range(0, len(d) - 1, 2)]

        def _on_cmd(self, msg):
            if msg.data == 'START':
                self.pf.start()
            elif msg.data == 'STOP':
                self.pf.stop()

        def _on_odom(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self._pose = (p.x, p.y, yaw)

        def _tick(self):
            t = self.get_clock().now().nanoseconds * 1e-9
            vx, wz = self.pf.tick(t, self._targets, self._pose)
            if self.pf.state == 'following':
                m = Twist(); m.linear.x = float(vx); m.angular.z = float(wz)
                self.pub.publish(m)
            speak = self.pf.just_spoke
            self.pf.just_spoke = None
            _, bearing = (_rel(self.pf.target.cx, self.pf.target.cy, self._pose)
                          if self.pf.target else (0.0, None))
            self.face.update(t, self.pf.state, speak=speak, bearing_deg=bearing)
            self._publish_state()
            if self.pf.state == 'ending':
                self.pf.reset()

        def _publish_state(self):
            self.pub_state.publish(String(data=self.pf.state))

    rclpy.init(args=args)
    node = PersonFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Registrar entry point** — editar `ros2_packages/robot_nav/setup.py`, na lista `console_scripts` (após a linha `motion_guard`, ~linha 39):

```python
            'person_follower = robot_nav.person_follower:main',
```

- [ ] **Step 3: Build**

Run: `cd ~/Workspace/Controle_robo_web && source /opt/ros/jazzy/setup.bash && colcon build --packages-select robot_nav --symlink-install`
Expected: `Finished <<< robot_nav`.

- [ ] **Step 4: Fumaça — o nó sobe e para de propósito**

Run: `source install/setup.bash && ros2 run robot_nav person_follower --ros-args -p follow_enabled:=true & sleep 3; ros2 topic echo /follow_person_state --once; kill %1`
Expected: imprime `data: idle`.

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py ros2_packages/robot_nav/setup.py
git commit -m "feat(person_follower): no ROS (subs targets/cmd/odom, pubs vel/state) + entry point"
```

---

## Task 7: `motion_guard` publica `follow_person_targets`

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py`
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Consumes: os clusters que o guard já calcula em `_cluster()`/`_person_centroid()`.
- Produces: método puro `MotionGuard.person_centroids(pts, pose) -> list[(cx,cy)]` (todos os clusters candidatos a pessoa, no odom) — testável. O nó publica isso como `Float32MultiArray` `[cx0,cy0,...]` no tópico `follow_person_targets`, atrás do param `publish_follow_targets` (default False → nem cria o publisher).

- [ ] **Step 1: Escrever o teste que falha** (em `test_motion_guard.py`)

```python
def test_person_centroids_lista_clusters():
    g = _guard()
    # dois grupos separados de pontos = dois clusters
    pts = [(2.0, 0.0), (2.05, 0.05), (2.0, 3.0), (2.05, 3.05)]
    cs = g.person_centroids(pts, POSE)
    assert len(cs) == 2
    xs = sorted(c[1] for c in cs)   # cy ~0 e ~3
    assert abs(xs[0] - 0.0) < 0.2 and abs(xs[1] - 3.0) < 0.2
```

- [ ] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_motion_guard.py -k person_centroids -v`
Expected: FAIL (no attribute 'person_centroids').

- [ ] **Step 3: Implementação mínima** — adicionar em `MotionGuard` um método que reusa `_cluster` e devolve o centróide de cada cluster:

```python
    def person_centroids(self, pts, pose):
        out = []
        for cl in self._cluster(pts):
            n = len(cl)
            cx = sum(p[0] for p in cl) / n
            cy = sum(p[1] for p in cl) / n
            out.append((cx, cy))
        return out
```

> Se `_cluster` trabalha em coordenadas do sensor, converter pelo mesmo TF que `_person_centroid` usa (checar o corpo de `_person_centroid`, linhas ~858-873, e replicar a transformação pra odom). Manter consistente com o frame que o `person_follower` espera (odom).

- [ ] **Step 4: Rodar e ver passar** (+ suíte do guard inteira, pra garantir 0 regressão)

Run: `cd ros2_packages/robot_nav && python -m pytest test/test_motion_guard.py -v`
Expected: PASS (todos, inclusive os antigos).

- [ ] **Step 5: Publisher no nó** — no `main()` do motion_guard, atrás do param `publish_follow_targets` (default False): criar `create_publisher(Float32MultiArray, 'follow_person_targets', 10)` e, no tick de scan (junto do `_face_tick`), achatar `person_centroids(...)` em `[cx0,cy0,...]` e publicar. Marcar a cola `# pragma: no cover`.

- [ ] **Step 6: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/motion_guard.py ros2_packages/robot_nav/test/test_motion_guard.py
git commit -m "feat(motion_guard): publica follow_person_targets (clusters-pessoa, atras de param)"
```

---

## Task 8: Wiring do pipeline — `twist_mux_auto` + launch

**Files:**
- Modify: `ros2_packages/robot_nav/config/twist_mux_auto.yaml`
- Modify: `ros2_packages/robot_nav/launch/robot.launch.py`
- Modify: `ros2_packages/robot_nav/launch/sim.launch.py`

**Interfaces:**
- Consumes: nó `person_follower` (Task 6), publisher `follow_person_targets` (Task 7).
- Produces: `follow_person_vel` arbitrado no mux de autonomia (prio 17, entre door=20 e follower=15) e os nós subindo no launch real e no sim.

- [ ] **Step 1: Adicionar a entrada no mux** — em `twist_mux_auto.yaml`, dentro de `topics:` (após `door`, antes de `follower`):

```yaml
      follow_person:
        topic: follow_person_vel   # person_follower (seguir pessoa) — rota pausada no seguir
        timeout: 0.5
        priority: 17
```

Atualizar o comentário do topo do arquivo pra citar `person_follower(follow_person_vel)` na cadeia e a ordem `door(20) > follow_person(17) > follower(15) > nav(10)`.

- [ ] **Step 2: Subir o nó no launch real** — em `robot.launch.py`, junto dos outros nós do robot_nav, adicionar (com `publish_follow_targets:=true` no motion_guard e `follow_enabled:=true` no person_follower):

```python
    person_follower = Node(
        package='robot_nav', executable='person_follower', name='person_follower',
        parameters=[{'follow_enabled': True}], output='screen',
    )
    # ...adicionar person_follower à lista de nós retornada (LaunchDescription)
    # ...e no motion_guard, acrescentar {'publish_follow_targets': True} aos parameters
```

- [ ] **Step 3: Subir o nó no sim** — mesma coisa em `sim.launch.py` (com `use_sim_time: True` nos parameters, como os outros nós de lá).

- [ ] **Step 4: Build + fumaça do grafo**

Run: `colcon build --packages-select robot_nav --symlink-install && source install/setup.bash`
Depois subir o sim (Task 11) e conferir: `ros2 node list | grep person_follower` e `ros2 topic info /follow_person_vel`.
Expected: nó presente; `follow_person_vel` com 1 publisher (person_follower) e 1 subscriber (twist_mux_auto).

- [ ] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/config/twist_mux_auto.yaml ros2_packages/robot_nav/launch/robot.launch.py ros2_packages/robot_nav/launch/sim.launch.py
git commit -m "feat(follow): person_follower no pipeline (twist_mux_auto prio 17 + launch real/sim)"
```

---

## Task 9: `app.py` — orquestração (start/stop + pausa/retoma rota)

**Files:**
- Modify: `controle_web/app.py`
- Modify: `controle_web/map_bridge.py`
- Modify: template + JS da GUI (botão "Parar de seguir")

**Interfaces:**
- Consumes: `follow_cmd` (String) consumido pelo person_follower; `follow_person_state` (String) publicado por ele.
- Produces:
  - `map_bridge`: publisher `follow_cmd`, subscriber `follow_person_state`; métodos `follow_start()` (pausa waypoints guardando o índice atual, publica `follow_cmd=START`) e `follow_stop()` (publica `STOP`); ao ver `follow_person_state` chegar a `ending`/`idle` vindo de `following`/`lost`, **retoma** os waypoints do índice guardado (se havia) via a mesma máquina do `start_waypoints`.
  - `app.py`: rota HTTP/socket `handle_follow(action)` → `map_bridge.follow_start()/follow_stop()`; reemite `follow_person_state` pros clientes.

- [ ] **Step 1: `map_bridge` — pub/sub e start/stop** — adicionar (seguindo o padrão de `send_goal`/`start_waypoints` já no arquivo):

```python
# no __init__ do bridge ROS:
self._follow_cmd_pub = self.create_publisher(String, 'follow_cmd', 10)
self.create_subscription(String, 'follow_person_state', self._on_follow_state, 10)
self._follow_prev_state = 'idle'
self._route_resume_idx = None   # índice do waypoint pra retomar

def follow_start(self):
    self._route_resume_idx = self._current_wp_index()  # guarda onde a rota estava
    self.stop_waypoints()                              # pausa o runner (sem apagar a lista)
    self._follow_cmd_pub.publish(String(data='START'))

def follow_stop(self):
    self._follow_cmd_pub.publish(String(data='STOP'))

def _on_follow_state(self, msg):
    s = msg.data
    if self._follow_prev_state in ('following', 'lost') and s in ('ending', 'idle'):
        if self._route_resume_idx is not None:
            self._resume_waypoints_from(self._route_resume_idx)  # retoma rota
            self._route_resume_idx = None
    self._follow_prev_state = s
```

> `_current_wp_index`, `stop_waypoints` (já existe), e `_resume_waypoints_from` devem casar com a implementação real do runner de waypoints no `map_bridge`. Se o runner não expõe o índice, guardar a sublista de waypoints restante em vez do índice. Ler o código do runner (`start_waypoints`/`stop_waypoints`) antes de implementar e adaptar.

- [ ] **Step 2: `app.py` — endpoint** — adicionar handler (socket, no padrão de `handle_start_waypoints`):

```python
@socketio.on('follow')
def handle_follow(data):
    action = (data or {}).get('action')
    if action == 'start':
        map_bridge.follow_start()
    elif action == 'stop':
        map_bridge.follow_stop()
    emit('follow_ack', {'ok': True, 'action': action})
```

- [ ] **Step 3: GUI — botão "Parar de seguir"** — no template/JS da GUI principal, adicionar um botão que só aparece quando `follow_person_state ∈ {following, lost}` (reemitido pelo bridge) e emite `socket.emit('follow', {action:'stop'})`. (O START pela GUI é opcional; o principal é o "freio" remoto — incluir também um "Seguir alguém à frente" é bônus, não obrigatório no v1.)

- [ ] **Step 4: Fumaça** — subir o controle_web e, com o sim rodando, emitir `follow start` e ver `ros2 topic echo /follow_cmd` receber `START`; emitir `stop` → `STOP`.

Run (manual, com sim ativo): abrir a GUI, clicar o botão / usar o console do navegador `socket.emit('follow',{action:'start'})`.
Expected: `/follow_cmd` publica `START`; ao parar, `STOP`.

- [ ] **Step 5: Rodar os testes do controle_web** (garantir 0 regressão)

Run: `cd controle_web && python -m pytest -q`
Expected: PASS (suíte existente).

- [ ] **Step 6: Commit**

```bash
git add controle_web/app.py controle_web/map_bridge.py controle_web/templates controle_web/static
git commit -m "feat(follow): app.py orquestra seguir (start/stop + pausa/retoma rota) + botao GUI"
```

---

## Task 10: Cara (iPad :7000) — botões Seguir/Parar + falas

**Files:**
- Create: `face_web/static/seguir_inicio.mp3`, `face_web/static/nao_te_vejo.mp3`
- Create/Modify: `face_web/tools/gen_tts.py` (gerador gTTS, se não existir)
- Modify: `face_web/face_app.py` (`GET /state` inclui o follow state; `POST /follow` repassa pro controle_web)
- Modify: `face_web/static/face.js` (botões + falas)
- Test: `face_web/test_face_app.py`

**Interfaces:**
- Consumes: `/tmp/person_follow_face.json` (Task 5) — `{follow_state, speak, cbear_deg}`.
- Produces: `face_app.GET /state` com `follow_state` e `speak`; `POST /follow {action}` que repassa pro controle_web (:5000) via socket/HTTP; `face.js` mostra botão "Seguir" (toque na cara, quando idle) e "Parar" (quando following/lost), e toca os mp3 nas transições.

- [ ] **Step 1: Gerar os mp3** (gTTS pt-BR, no molde de ola/licenca):

```python
# face_web/tools/gen_tts.py
from gtts import gTTS
gTTS('Irei te seguir, tente ficar próximo e ir devagar', lang='pt-br').save('face_web/static/seguir_inicio.mp3')
gTTS('Não estou mais te vendo, poderia se aproximar?', lang='pt-br').save('face_web/static/nao_te_vejo.mp3')
```

Run: `python face_web/tools/gen_tts.py` → confere que os 2 mp3 existem e tocam.

- [ ] **Step 2: `/state` inclui follow** — teste em `test_face_app.py`:

```python
def test_state_inclui_follow(tmp_path, monkeypatch):
    # aponta o face_app pro json de follow de teste com follow_state=following
    ...
    r = client.get('/state')
    assert r.get_json()['follow_state'] == 'following'
```

Implementar em `face_app.py`: ler `/tmp/person_follow_face.json` (mesmo padrão do `motion_guard_face.json` já lido) e incluir `follow_state`/`speak` na resposta de `/state`.

- [ ] **Step 3: Rodar o teste**

Run: `cd face_web && python -m pytest test_face_app.py -v`
Expected: PASS.

- [ ] **Step 4: `face.js` — botões + falas** (ES5 puro; passa o teste de léxico anti-ES6 do repo):
  - `var sndSeguir = new Audio('/static/seguir_inicio.mp3'); var sndNaoVejo = new Audio('/static/nao_te_vejo.mp3');`
  - No polling do `/state` (já existe a cada 300ms): quando `st.speak === 'start'` → `sndSeguir.play()`; `st.speak === 'lost'` → `sndNaoVejo.play()` (com throttle, como o `licenca`).
  - Botão "Seguir" visível quando `st.follow_state === 'idle'` e há alguém (`st.cbear_deg != null`); toque → `POST /follow {action:'start'}`.
  - Botão "Parar" visível quando `st.follow_state` é `following`/`lost`; toque → `POST /follow {action:'stop'}`.
  - Áudio destrava no 1º tap (mecanismo `unlockAudio()` já existe).

- [ ] **Step 5: Rodar a suíte face_web + o teste de léxico ES5**

Run: `cd face_web && python -m pytest -v`
Expected: PASS (inclui o guard de sintaxe pós-ES5).

- [ ] **Step 6: Commit**

```bash
git add face_web/static/seguir_inicio.mp3 face_web/static/nao_te_vejo.mp3 face_web/tools/gen_tts.py face_web/face_app.py face_web/static/face.js face_web/test_face_app.py
git commit -m "feat(face): botoes Seguir/Parar + falas (inicio/nao-te-vejo) + follow_state no /state"
```

---

## Task 11: Validação no SIM (checklist — sem deploy no real antes disso fechar)

**Files:** nenhum (validação). Mundo `sala_grande` + `bin/teleop-pernas`.

- [ ] **Step 1: Subir o sim** com a stack + controle_web + face_app locais. Conferir grafo: `ros2 node list | grep person_follower`, `ros2 topic info /follow_person_vel` (1 pub, 1 sub no twist_mux_auto), `ros2 topic echo /follow_person_state` = `idle`.

- [ ] **Step 2: Caso 1 — segue e mantém 1,5 m.** Teleop a pessoa-2-pernas reto pra longe do robô; disparar `follow start`. Esperado: robô encara, anda, **para a ~1,5 m** quando a pessoa para. Medir a distância no `follow_person_state`/echo.

- [ ] **Step 3: Caso 2 — perde e retoma/desiste.** Levar as pernas pra trás de uma parede. Esperado: robô **para**, `follow_person_state=lost`, a cara fala "não te vejo". Reaparecer perto → volta a seguir. Não reaparecer por >12 s → `ending` e **a rota anterior retoma** (se havia rota ativa antes do follow).

- [ ] **Step 4: Caso 3 — segurança (pipeline intacto).** Com o robô seguindo, colocar OUTRO obstáculo/pernas na frente **dele** a <1 m. Esperado: `motion_guard` entra em `blocked` e **zera tudo** (o follow não fura) — prova do "mesmo medo".

- [ ] **Step 5: Caso 4 — parar de propósito.** Botão "Parar" na cara e na GUI, em `following` e em `lost`. Esperado: `ending` → rota retoma (ou para, se não havia rota).

- [ ] **Step 6: Iterar knobs** (`stop_dist`, `wz_cap`, `drive_align_deg`, `assoc_gate`, `lost_timeout`) ao vivo via `ros2 param set /person_follower ...` até o dono aprovar o comportamento. Anotar os valores finais no `ESTADO_PROJETO.md`.

- [ ] **Step 7: Commit** (se algum default mudou na iteração)

```bash
git add ros2_packages/robot_nav/robot_nav/person_follower.py
git commit -m "tune(person_follower): defaults ajustados no sim (sala_grande + teleop-pernas)"
```

---

## Task 12: Validação no REAL (checklist — dono presente, mão no E-stop)

**Files:** nenhum (validação de campo). Deploy pela Pi (`git fetch && reset --hard` + `colcon build robot_nav` + relaunch + restart face_app).

- [ ] **Step 1:** Deploy da branch na Pi + relaunch limpo (launch.sh faz a faxina) + restart `face_web` + recarregar iPad.
- [ ] **Step 2:** Run curta e controlada, cone livre, **mão no E-stop**. Repetir os 5 casos da Task 11 com pessoa de verdade.
- [ ] **Step 3:** Confirmar o piso de 1,5 m e que o guard para se alguém entra na frente. Puxar os CSVs pra revisar (o `t` casa com `motion_guard.csv`).
- [ ] **Step 4:** Se OK, atualizar `ESTADO_PROJETO.md` e **merge da branch → main**.

---

## Notas de decisão (do spec, pra quem executa)

- **Retomada de rota (Task 9) é a peça mais acoplada.** Se o runner de waypoints não expõe índice/resto de forma limpa, é aceitável entregar o v1 com "fim do seguir = para no lugar" e fazer o resume como sub-fase própria depois. O dono pediu resume, então é a meta — mas não trave o resto do pacote por causa dele.
- **Sem câmera, troca de alvo é limitação assumida do v1** — mitigada pelo "aproxime-se". Não implementar re-ID agora.
- **`follow_person_vel` NUNCA fura a segurança.** Se em algum teste o robô avançar com o guard em `blocked`, é BUG de wiring (velocidade indo pro tópico errado) — conferir que passa por `twist_mux_auto`, não pelo mux final.
