# Plano: motion_guard solta por PLANO ASSENTADO

**Spec**: `docs/superpowers/specs/2026-07-20-motion-guard-settling-design.md`
**Arquivo tocado**: `ros2_packages/robot_nav/robot_nav/motion_guard.py`
**Testes**: `ros2_packages/robot_nav/test/test_motion_guard.py`
**Nada mais é tocado** — `unstuck_supervisor.py` fica intacto de propósito.

## Estado do working tree antes de começar

Há trabalho NÃO commitado do pacote v2 do zigue-zague em
`path_follower.py` + `test_path_follower.py` + `ESTADO_PROJETO.md`. Este
plano não encosta nesses arquivos, mas **não commitar junto**: o commit
desta mudança deve citar só `motion_guard.py` e `test_motion_guard.py`.

## Decisão de projeto: `settling` só no CSV

`filter()` passa a poder retornar a string `'settling'`. O **nó** traduz
pra `'blocked'` antes de publicar em `motion_guard/state`, e escreve a
string crua no CSV. Assim:

- `unstuck_supervisor.py:1240` (`msg.data == "blocked"`) continua casando →
  standdown preservado, zero risco do BO de 07-10.
- `_face_tick` também vê `blocked` → a cara segue pedindo licença.
- O CSV distingue `blocked` real de `settling` pra análise.

---

## Task 1 — knobs no `GuardConfig`

Em `motion_guard.py`, depois de `wall_ghost_frac` (linha ~119-126), no fim
do dataclass:

```python
    settle_enabled: bool = True     # soltar o blocked por PLANO ASSENTADO e
                                    # não só pelo relógio (bug da curva ~70°
                                    # ao retomar pós-blocked, repro no sim
                                    # 07-20): no fim do clear_time o global
                                    # plan ainda nasce CONTORNANDO a pessoa
                                    # que segue no costmap; o robô arranca
                                    # comprometido com um desvio que morre em
                                    # ~2s. False = comportamento pré-07-20.
    settle_window: float = 1.0      # s — janela deslizante do rumo do plano
    settle_tol_deg: float = 8.0     # ° — amplitude (máx-mín) na janela abaixo
                                    # disso = assentado. ~metade do erro de
                                    # mira medido no release ruim (15-19°).
    settle_max: float = 4.0         # s — teto do settling desde o fim do
                                    # clear_time. Folgado abaixo do
                                    # guard_hold_max=20s do unstuck.
    settle_min_samples: int = 3     # não declarar assentado com 1 amostra
    settle_plan_stale: float = 1.0  # s sem /plan fresco -> libera (fail-open)
    settle_lookahead: float = 0.6   # m — comprimento de arco do início do
                                    # plano até o ponto que define o rumo
```

`settle_lookahead` não estava na tabela de 6 knobs do spec (lá era "~0.6 m"
fixo). Virou knob porque custa uma linha e o A/B fica ao vivo. Se você
preferir constante, diga.

**Verificação**: `python3 -c "from robot_nav.motion_guard import GuardConfig; print(GuardConfig().settle_tol_deg)"` → `8.0`.

---

## Task 2 — estado interno + `observe_plan()` na classe pura

Em `MotionGuard.__init__` (após `_watch_corridor`, linha ~216):

```python
        # assentamento do plano (07-20): rumo do início do /plan numa janela
        # deslizante; o release pós-blocked espera a amplitude cair.
        self._plan_hdg = deque()        # (t, rumo em frame map, rad)
        self._last_plan_t: float = -math.inf
        self._was_blocked: bool = False  # esteve em blocked desde o último idle
        self._settle_since: float = -math.inf  # t em que o clear_time venceu
```

Método novo, logo depois de `observe()` (antes de `_cluster`, ou junto dos
helpers — colocar imediatamente após `observe` termina):

```python
    def observe_plan(self, t: float, poses: List[Pt]) -> None:
        """rumo do início do plano global (frame map) na janela deslizante.

        Rumo ABSOLUTO de propósito: relativo ao robô, o giro do próprio robô
        entraria na medida e um plano parado pareceria instável.
        """
        c = self.cfg
        if len(poses) < 2:
            return
        self._last_plan_t = t
        x0, y0 = poses[0]
        tip = poses[-1]
        acc = 0.0
        for a, b in zip(poses, poses[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= c.settle_lookahead:
                tip = b
                break
        dx, dy = tip[0] - x0, tip[1] - y0
        if math.hypot(dx, dy) < 1e-6:
            return          # plano degenerado (robô em cima do goal)
        self._plan_hdg.append((t, math.atan2(dy, dx)))
        while self._plan_hdg and t - self._plan_hdg[0][0] > c.settle_window:
            self._plan_hdg.popleft()

    def _plan_settled(self, t: float) -> bool:
        """AMPLITUDE (máx-mín) na janela < settle_tol_deg.

        Amplitude e não delta entre replans: um plano que gira devagar e
        constante tem delta pequeno a cada ciclo e amplitude grande — é
        exatamente o caso da pessoa saindo ANDANDO, o pior do bug.
        Todo caminho de dúvida devolve True (fail-open: settling só PARA).
        """
        c = self.cfg
        if t - self._last_plan_t > c.settle_plan_stale:
            return True                     # sem plano fresco -> libera
        while self._plan_hdg and t - self._plan_hdg[0][0] > c.settle_window:
            self._plan_hdg.popleft()
        if len(self._plan_hdg) < c.settle_min_samples:
            return True
        ref = self._plan_hdg[0][1]
        rel = [(h - ref + math.pi) % (2 * math.pi) - math.pi
               for _, h in self._plan_hdg]
        return (max(rel) - min(rel)) <= math.radians(c.settle_tol_deg)
```

`Pt` já é `Tuple[float, float]` (linha 40) e `deque` já está importado
(usado em `_snaps`).

---

## Task 3 — ramo `settling` no `filter()`

`filter()` hoje (linhas 411-433) decide em cascata. A mudança entra
**entre** o ramo `blocked` e o ramo `slowing`:

```python
        c = self.cfg
        if not c.enabled or t - self._last_scan_t > c.scan_stale:
            return vx, wz, 'passthrough'
        freeze = (t - self._last_moving_t < c.clear_time
                  and self._last_nearest < c.freeze_dist)
        if freeze or t - self._last_corridor_t < c.clear_time:
            self._was_blocked = True
            self._settle_since = -math.inf     # o relógio ainda nem venceu
            # parada TOTAL: wz TAMBÉM zera (dono 07-02: ...)   [comentário existente]
            return (0.0 if vx > 0.0 else vx), 0.0, 'blocked'
        # o clear_time venceu. Antes de arrancar, o plano tem que estar
        # ASSENTADO — senão o robô sai comprometido com o contorno da pessoa
        # que ainda está no costmap (curva ~70° do bug de 07-20).
        if self._was_blocked and c.settle_enabled:
            if self._settle_since == -math.inf:
                self._settle_since = t
            if (t - self._settle_since < c.settle_max
                    and not self._plan_settled(t)):
                return (0.0 if vx > 0.0 else vx), 0.0, 'settling'
        self._was_blocked = False
        self._settle_since = -math.inf
        if t - self._last_moving_t < c.clear_time:
            ...   # ramo slowing, INTACTO
```

Notas de correção:

- A ré (`vx < 0`) passa no `settling` pela mesma expressão do `blocked`.
- `_was_blocked` só zera quando o settling termina (por assentar ou por
  `settle_max`), então um `blocked` piscante não perde o latch.
- Sem `settle_enabled`, ou sem `/plan` jamais recebido
  (`_last_plan_t = -inf` → stale → `True`), o caminho é bit-a-bit o de hoje.

---

## Task 4 — nó: assinar `/plan` e traduzir o estado

**4a.** Import: `nav_msgs.msg` já traz `Odometry` e `OccupancyGrid` — somar
`Path` ao import existente de `nav_msgs.msg` no bloco do `main()`.

**4b.** `_CFG_PARAMS` (linha 516-523): somar ao final da tupla

```python
                       'settle_enabled', 'settle_window', 'settle_tol_deg',
                       'settle_max', 'settle_min_samples',
                       'settle_plan_stale', 'settle_lookahead')
```

**4c.** Subscrição, junto das outras no `__init__`:

```python
            self.create_subscription(Path, 'plan', self._on_plan, 10)
```

`plan` relativo, como os outros tópicos do nó (o launch resolve o
namespace). O plano já vem em frame `map` → sem TF.

**4d.** Callback:

```python
        def _on_plan(self, msg: Path):
            self.guard.observe_plan(
                self._now(),
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses])
```

**4e.** `_on_cmd` (linha 711): traduzir antes de publicar, CSV com a crua.

```python
            vx, wz, state = self.guard.filter(t, msg.linear.x, msg.angular.z)
            # settling é blocked NO FIO: unstuck_supervisor casa string exata
            # 'blocked' pro standdown (BO 07-10: ré em cima de pessoa). O
            # estado fino fica só no CSV.
            wire = 'blocked' if state == 'settling' else state
            ...
            if wire != self._last_state:
                self._last_state = wire
                self.pub_state.publish(String(data=wire))
                if wire == 'passthrough':
                    ...
```

e o `self._csv.writerow([... state ...])` continua com `state` (cru).

**4f.** Log de boot: somar `settle %.1f°/%.1fs` ao `info` da linha 576 pra
o estado dos knobs aparecer no launch.

---

## Task 5 — testes (TDD: escrever antes do código de cada task)

Em `test_motion_guard.py`, usando o helper `_guard(**kw)` já existente.
Precisa de um helper local que gere um plano reto com rumo dado:

```python
def _plan(hdg, n=6, step=0.2):
    return [(i * step * math.cos(hdg), i * step * math.sin(hdg))
            for i in range(n)]
```

E de um helper que ponha o guard em `blocked` e avance o relógio — os
testes existentes (`test_filter_resumes_after_clear_time`, linha 290) já
mostram o padrão; reusar.

1. `test_settling_releases_when_plan_stable` — plano no mesmo rumo em
   t=0.0/0.3/0.6, `clear_time` vencido → estado `idle`/`slowing`, não
   `settling`. (sem regressão no caso bom)
2. `test_settling_holds_while_plan_oscillates` — rumos ±20° alternando →
   `settling`, `vx == 0.0`.
3. `test_settling_holds_on_slow_constant_drift` — rumos 0°,6°,12°,18° a
   0.3 s de intervalo (delta 6° < tol, amplitude 18° > tol) → `settling`.
   **Este é o teste que falha com critério de delta e passa com
   amplitude** — o caso da pessoa saindo andando.
4. `test_settling_force_releases_after_settle_max` — plano instável o tempo
   todo, `t` além do `settle_max` → sai do `settling`.
5. `test_settling_no_plan_behaves_like_today` — nunca chamar
   `observe_plan` → sequência de estados idêntica ao teste 290 de hoje.
6. `test_settling_does_not_zero_reverse` — `vx=-0.2` durante settling passa.
7. `test_settling_disabled_behaves_like_today` — `settle_enabled=False`.
8. `test_plan_heading_uses_lookahead_arc` — plano que curva depois de 1 m:
   com `settle_lookahead=0.6` o rumo é o do trecho inicial, não o do fim.

**Verificação**: `colcon test --packages-select robot_nav
--event-handlers console_direct+` — os 3 testes de `settling` devem falhar
antes do código e passar depois; os ~246 existentes seguem verdes.

---

## Task 6 — validação, na ordem (nada vai blind pro campo)

Nó de segurança → a ordem não é negociável:

1. **Unitários** verdes (Task 5) + suíte inteira sem regressão.
2. **Sim** `sala_grande`, o mesmo rig da repro de 07-20: pessoa saindo
   **ANDANDO** (~0.6 m/s), que é o caso feio. Matar `parameter_bridge`
   junto ao derrubar o launch (senão o órfão de `/clock` trava o AMCL).
3. **A/B do pico de desvio**: mesma rota, `settle_enabled` `True` vs
   `False` via `ros2 param set` (é live, não precisa relaunch). Métrica =
   pico de desvio angular pós-release, do CSV `motion_guard.csv` +
   `path_follower.csv`. Alvo: sair dos 60-87° medidos.
4. **Só então real**, e com o dono avisado de qual knob mexer se enrolar
   (`settle_tol_deg` pra cima solta mais cedo; `settle_max` pra baixo
   limita o quanto ele pode segurar).

Eu preparo os comandos de launch/coleta; você roda e eu leio os CSVs.

## Riscos

- **Robô parado mais tempo** perto de gente. É o trade explícito, limitado
  a `settle_max = 4 s` e coberto pelo `guard_hold_max = 20 s` do unstuck.
- **`/plan` com nome/namespace diferente** do que o nó espera → nunca chega
  plano → fail-open, comportamento de hoje. Conferir no sim com
  `ros2 topic info` antes de concluir que o settling "não funcionou".
- **Plano publicado a taxa baixa** (< 3 Hz) → `settle_min_samples=3` numa
  janela de 1 s nunca enche → sempre "assentado" → fail-open. Medir a
  taxa real de `/plan` no passo 2 e, se for baixa, subir `settle_window`.
