# Fluidez dos giros do path_follower — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduzir as micro-paradas do path_follower (56% do tempo em turning na run real de 07-02) com 2 mudanças pequenas medidas separadas no sim: `rot_min` 2.0→2.4 e alvo de giro congelado.

**Architecture:** Tudo na lógica pura `DecisiveFollower` de `ros2_packages/robot_nav/robot_nav/path_follower.py` (testável sem ROS). Validação por métricas do `controle_web/logs/follow_debug.csv` em runs de sim com a mesma rota, comparando baseline → P1 → P1+P2.

**Tech Stack:** Python/rclpy, pytest, gz Harmonic via `./launch.sh --sim --nav2`.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-07-02-fluidez-giros-path-follower-design.md`.
- NÃO mexer: unstuck, collision_monitor, replan do nav2, `lookahead`/`turn_enter`, estado `goal_turn`.
- Commits sem rodapé de co-autoria (preferência do dono).
- Cada passo do sim usa a MESMA rota de goals; regressão em métrica → reverte o passo.
- Robô real fica FORA deste plano (deploy/validação real é decisão do dono depois).

---

### Task 1: Baseline no sim (ANTES de mudar código)

**Files:**
- Create: `<scratchpad>/fluidez/analyze_follow.py` (analisador, fora do repo)
- Create: `<scratchpad>/fluidez/send_route.sh` (rota fixa de goals)

**Interfaces:**
- Produces: `analyze_follow.py <csv>` imprime `turning_pct`, `driving_median_s`, `turns_over_5s`, `wz_flips`; `send_route.sh` manda a rota (4 goals em sequência, espera cada um).

- [x] **Step 1: Escrever o analisador**

```python
#!/usr/bin/env python3
"""analyze_follow.py <follow_debug.csv> — métricas de fluidez da spec 07-02."""
import csv, statistics, sys

rows = list(csv.DictReader(open(sys.argv[1])))
segs = []
for r in rows:
    t, st = float(r['t']), r['state']
    if segs and segs[-1][0] == st:
        segs[-1][2] = t
    else:
        segs.append([st, t, t])
tot = {}
for st, a, b in segs:
    tot[st] = tot.get(st, 0.0) + (b - a)
ativo = tot.get('turning', 0) + tot.get('driving', 0)
turns = [b - a for st, a, b in segs if st == 'turning']
drives = [b - a for st, a, b in segs if st == 'driving']
wz = [float(r['wz']) for r in rows if r['state'] == 'turning']
flips = sum(1 for x, y in zip(wz, wz[1:]) if x * y < 0)
print(f"turning_pct={100*tot.get('turning',0)/ativo:.1f}")
print(f"driving_median_s={statistics.median(drives):.2f}" if drives else "driving_median_s=nan")
print(f"turns_over_5s={sum(1 for d in turns if d > 5)}")
print(f"turning_episodes={len(turns)}")
print(f"wz_flips={flips}")
```

- [x] **Step 2: Escrever a rota fixa**

```bash
#!/usr/bin/env bash
# send_route.sh — 4 goals em sequência no mundo sala.sdf / mapa sim_sala.
# Antes de rodar a 1ª vez: conferir células livres com
#   python3 -c "from PIL import Image; im=Image.open('maps/sim_sala.pgm'); print(im.size)"
# e ajustar as coordenadas se algum goal cair em parede (planner loga "no valid path").
set -e
source /opt/ros/jazzy/setup.bash; source install/setup.bash
goal() {
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {z: $3, w: $4}}}}" \
    --feedback > /dev/null
}
goal 2.5  0.0  0.0    1.0     # atravessa a porta
goal 2.5 -1.5  -0.707 0.707   # canto de baixo
goal -2.0 -1.4 1.0    0.0     # volta pela porta, lado esquerdo
goal 0.0  0.0  0.0    1.0     # origem
echo ROTA_COMPLETA
```

- [x] **Step 3: Rodar o sim com o HEAD atual e a rota**

Run: `./launch.sh --sim --nav2 --world=worlds/sala.sdf --map=maps/sim_sala.yaml` (background), esperar nav ativo (`ros2 topic echo /follow_state --once`), então `bash <scratchpad>/fluidez/send_route.sh`.
Expected: `ROTA_COMPLETA` sem abort; `controle_web/logs/follow_debug.csv` gravado.

- [x] **Step 4: Medir e guardar a baseline**

Run: `python3 <scratchpad>/fluidez/analyze_follow.py controle_web/logs/follow_debug.csv | tee <scratchpad>/fluidez/baseline.txt` e copiar o csv pra `<scratchpad>/fluidez/baseline_follow_debug.csv`.
Expected: 4 números impressos (turning_pct alto ~50%+ reproduz o sintoma; se turning_pct < 20% na baseline o sim NÃO reproduz o problema → PARAR e discutir com o dono antes de seguir).

### Task 2: P1 — `rot_min` 2.0 → 2.4

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/path_follower.py:82` (dataclass) e `:202` (declare_parameter)
- Test: `ros2_packages/robot_nav/test/test_path_follower.py`

**Interfaces:**
- Produces: `FollowConfig.rot_min == 2.4` (default novo; testes existentes usam `cfg.rot_min` relativo, não quebram).

- [x] **Step 1: Teste do default novo (falhando)**

```python
def test_rot_min_default_beats_deadzone_crawl():
    # 2026-07-02: rot_min 2.0 comandado ≈ 10°/s real (zona-morta 1.7 +
    # resposta 0.6·(cmd−1.7)) = rastejo que parece parada. 2.4 ≈ 25°/s.
    assert FollowConfig().rot_min == pytest.approx(2.4)
```

- [x] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_path_follower.py::test_rot_min_default_beats_deadzone_crawl -v`
Expected: FAIL (`2.0 != 2.4`)

- [x] **Step 3: Mudar os 2 defaults**

Em `path_follower.py` linha 82: `rot_min: float = 2.4` (e atualizar o comentário: `# rad/s — piso do giro (2.0 dava ~10°/s real = rastejo; 2.4 ≈ 25°/s, ver spec 07-02)`).
Na linha 202 do `main()`: `('rot_k', 3.0), ('rot_min', 2.4), ('rot_max', 4.5),`

- [x] **Step 4: Suíte inteira**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/ -q`
Expected: tudo PASS (nenhum teste fixa 2.0 absoluto).

- [x] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/path_follower.py ros2_packages/robot_nav/test/test_path_follower.py
git commit -m "path_follower: rot_min 2.0->2.4 — correcao pequena deixa de rastejar na zona-morta (spec fluidez 07-02)"
```

- [x] **Step 6: Medir P1 no sim**

Mesmo protocolo da Task 1 Steps 3-4 (relançar sim, mesma rota), salvar em `<scratchpad>/fluidez/p1.txt`.
Expected: `turning_pct` e `turns_over_5s` CAEM vs baseline; `wz_flips` não sobe. Regressão → reverter commit e discutir.

### Task 3: P2 — alvo de giro congelado

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/path_follower.py` (classe `DecisiveFollower`)
- Test: `ros2_packages/robot_nav/test/test_path_follower.py`

**Interfaces:**
- Produces: atributo interno `DecisiveFollower._turn_target: Optional[float]` (bearing map-frame congelado ao entrar em turning; `None` fora do giro).

- [x] **Step 1: Testes do congelamento (falhando)**

```python
def test_turn_target_frozen_while_plan_shifts():
    # entra girando pra +90° (path +y); no meio do giro o plano vira pra -y.
    # SEM freeze ele inverteria o giro (caça alvo móvel); COM freeze segue +.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    assert cmd.state == 'turning' and cmd.wz > 0.0
    path_down = [(0.0, -y * 0.1) for y in range(40)]
    cmd2 = f.update((0.0, 0.0, math.radians(45)), path_down, goal_active=True,
                    goal_yaw=-math.pi / 2)
    assert cmd2.state == 'turning'
    assert cmd2.wz > 0.0          # continua no alvo congelado (+90°), não flipa


def test_turn_target_cleared_after_alignment():
    # alinhou com o alvo congelado -> driving e o próximo giro re-mira o plano novo.
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, math.pi / 2), path_up, goal_active=True,
                   goal_yaw=math.pi / 2)
    assert cmd.state == 'driving'
    assert f._turn_target is None


def test_turn_target_reset_when_goal_lost():
    f = _fol()
    path_up = [(0.0, y * 0.1) for y in range(40)]
    f.update((0.0, 0.0, 0.0), path_up, goal_active=True, goal_yaw=math.pi / 2)
    cmd = f.update((0.0, 0.0, 0.0), path_up, goal_active=False, goal_yaw=None)
    assert cmd.state == 'idle'
    assert f._turn_target is None
```

- [x] **Step 2: Rodar e ver falhar**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/test_path_follower.py -k turn_target -v`
Expected: 3 FAIL (`AttributeError: _turn_target` / wz flipado).

- [x] **Step 3: Implementar o freeze**

Em `DecisiveFollower.__init__`: `self._turn_target: Optional[float] = None`.
No `update()`, branch idle (linha ~121) e branch arrived (~139): `self._turn_target = None` antes do return.
Substituir o bloco de histerese (passo 3 do update):

```python
        # 3) HISTERESE + ALVO CONGELADO: ao ENTRAR no giro trava o bearing-alvo
        #    (replans ~1Hz moviam o carrot NO MEIO do giro -> caçava alvo móvel,
        #    giros de 8-19s na run real de 07-02). Sai do giro -> re-olha o plano.
        if self.state == 'turning':
            if self._turn_target is not None:
                herr = wrap(self._turn_target - yaw)
            if abs(herr) <= c.turn_exit:
                self.state = 'driving'
                self._turn_target = None
        else:
            if abs(herr) >= c.turn_enter:
                self.state = 'turning'
                self._turn_target = bearing
            else:
                self.state = 'driving'
```

(`goal_turn` NÃO muda — o yaw do goal já é alvo fixo.)

- [x] **Step 4: Suíte inteira**

Run: `cd ros2_packages/robot_nav && python3 -m pytest test/ -q`
Expected: tudo PASS (os 2 testes de histerese existentes seguem valendo: com estado forçado `turning` sem `_turn_target`, o herr do plano é usado — comportamento antigo preservado).

- [x] **Step 5: Commit**

```bash
git add ros2_packages/robot_nav/robot_nav/path_follower.py ros2_packages/robot_nav/test/test_path_follower.py
git commit -m "path_follower: congela o bearing-alvo durante o giro — nao caça replan no meio do point-turn (spec fluidez 07-02)"
```

- [x] **Step 6: Medir P1+P2 no sim**

Mesmo protocolo (relançar sim, mesma rota), salvar em `<scratchpad>/fluidez/p1p2.txt`.
Expected: `turns_over_5s` → ~0; `driving_median_s` SOBE vs baseline; `wz_flips` estável. Regressão → reverter SÓ o commit do P2.

### Task 4: Fechamento

**Files:**
- Modify: `ESTADO_PROJETO.md` (seção 07-02)
- Modify: `docs/superpowers/plans/2026-07-02-fluidez-giros-path-follower.md` (checkboxes)

- [x] **Step 1: Smoke-test do nó** (lição de 06-28: teste unitário não pega bug de `self.X`)

Run: `source install/setup.bash && timeout 5 ros2 run robot_nav path_follower; echo "exit=$?"`
Expected: `exit=124` (timeout matou = nó vivo 5s sem crash).

- [x] **Step 2: Tabela comparativa + ESTADO**

Escrever em `ESTADO_PROJETO.md` a tabela baseline×P1×P1+P2 (4 métricas) + status "⏳ validar no real quando o dono quiser". Commit: `git commit -m "docs(estado): 07-02 fluidez path_follower — resultados sim P1/P2"`.

- [x] **Step 3: Push**

Run: `git push`
Expected: main atualizada; deploy na Pi fica a cargo do dono (fora do escopo).
