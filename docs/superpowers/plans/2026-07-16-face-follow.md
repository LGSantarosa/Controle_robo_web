# Cara fase 2 — olhos seguem a pessoa: plano de implementação

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans
> to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.
> (Preferência do dono: execução INLINE, sem subagentes.)

**Goal:** Olhos do face_web travam na pessoa (cluster móvel do motion_guard);
sem pessoa, voltam a vagar.

**Architecture:** motion_guard grava `/tmp/motion_guard_face.json` (atômico,
≤5Hz) → face_app `GET /state` lê e mapeia pra `x` de -1..1 → face.js (ES5!)
faz poll XHR 300ms e trava o `gazeTarget` com hold de 3s.
Spec: `docs/superpowers/specs/2026-07-16-face-follow-design.md`.

**Tech Stack:** Python puro (writer/reader), Flask (rota), JS ES5 (iPad 2).

## Global Constraints

- face.js é **ES5 PURO** — `test_face_js_es5_puro` barra tokens pós-2015
  (sem fetch/Promise/arrow/let/const/template/class/spread). XHR só.
- O guard NUNCA pode cair por causa da cara: I/O do JSON engole `OSError`.
- Escrita atômica: `.tmp` + `os.replace` (leitor nunca vê JSON parcial).
- Testes: `cd ros2_packages/robot_nav && python3 -m pytest test/ -q` (42
  verdes hoje) e, na raiz, `python3 -m pytest face_web/test_face_app.py -q`
  (3 verdes + 1 skip de flask hoje). Nada pode regredir.
- Rodar face_app manual: `controle_web/.venv/bin/python3 face_web/face_app.py`
  (o venv tem flask; o sistema não).

---

### Task 1: `FaceStateFile` no motion_guard

**Files:**
- Modify: `ros2_packages/robot_nav/robot_nav/motion_guard.py` (classe nova
  módulo-level + chamada no `_on_cmd`, linha ~656 onde `cbear` já existe)
- Test: `ros2_packages/robot_nav/test/test_motion_guard.py`

**Interfaces:**
- Produces: `FaceStateFile(path='/tmp/motion_guard_face.json',
  min_period=0.2)`, método `update(t: float, cbear_deg: int|None) -> bool`
  (True = gravou); atributo `last_error: str|None`. JSON gravado:
  `{"ts": 10.0, "cbear_deg": 30}` (ou `null`). Task 2 lê esse arquivo.

- [x] **Step 1: testes que falham** — acrescentar ao fim de
  `test/test_motion_guard.py`:

```python
# ---- FaceStateFile (cara fase 2: rumo da pessoa pro face_web) -----------

def test_face_state_file_grava_e_throttla(tmp_path):
    import json
    from robot_nav.motion_guard import FaceStateFile
    p = str(tmp_path / 'face.json')
    w = FaceStateFile(path=p, min_period=0.2)
    assert w.update(10.0, 30) is True
    assert json.load(open(p)) == {'ts': 10.0, 'cbear_deg': 30}
    assert w.update(10.1, 35) is False           # dentro do throttle
    assert w.update(10.3, 35) is True            # passou 0.2s
    assert json.load(open(p))['cbear_deg'] == 35


def test_face_state_file_transicao_null_uma_vez(tmp_path):
    import json
    from robot_nav.motion_guard import FaceStateFile
    p = str(tmp_path / 'face.json')
    w = FaceStateFile(path=p, min_period=0.2)
    assert w.update(10.0, None) is False         # sem pessoa antes: nada
    w.update(10.0, 30)
    assert w.update(10.05, None) is True         # transição FURA o throttle
    assert json.load(open(p))['cbear_deg'] is None
    assert w.update(10.1, None) is False         # já silenciou


def test_face_state_file_io_error_nao_propaga(tmp_path):
    from robot_nav.motion_guard import FaceStateFile
    w = FaceStateFile(path=str(tmp_path / 'nao_existe' / 'face.json'))
    assert w.update(10.0, 30) is False           # dir não existe: engole
    assert w.last_error
```

- [x] **Step 2: rodar e ver falhar** —
  `cd ros2_packages/robot_nav && python3 -m pytest test/test_motion_guard.py -q`
  Esperado: 3 FAIL com `ImportError: cannot import name 'FaceStateFile'`.

- [x] **Step 3: implementar** — em `motion_guard.py`: adicionar `import json`
  e `import os` no bloco de imports (linha ~33); classe módulo-level antes de
  `def main(...)` (linha ~451):

```python
class FaceStateFile:
    """Rumo da pessoa pro face_web (cara fase 2): JSON minúsculo em tmpfs.

    Atômico (tmp + os.replace), ≤5Hz com cluster; na transição pra
    sem-cluster grava UMA vez cbear_deg=null e silencia. I/O NUNCA propaga
    (a cara é decorativa; o guard não pode cair por ela).
    """

    def __init__(self, path: str = '/tmp/motion_guard_face.json',
                 min_period: float = 0.2):
        self.path = path
        self.min_period = min_period
        self.last_error: 'str|None' = None
        self._last_write_t = -math.inf
        self._had_person = False

    def update(self, t: float, cbear_deg: 'int|None') -> bool:
        if cbear_deg is None:
            if not self._had_person:
                return False
            self._had_person = False
        else:
            if t - self._last_write_t < self.min_period:
                return False
            self._had_person = True
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'ts': round(t, 3), 'cbear_deg': cbear_deg}, f)
            os.replace(tmp, self.path)
        except OSError as e:
            self.last_error = str(e)
            return False
        self._last_write_t = t
        return True
```

  No node (dentro de `main`): no `__init__`, junto dos outros atributos,
  `self._face = FaceStateFile()`; no `_on_cmd`, logo APÓS o bloco que calcula
  `cx/cy/cbear` (antes do `pose = self._last_pose or ...`):

```python
            self._face.update(t, cbear if cbear != '' else None)
            if self._face.last_error:
                self.get_logger().warn(
                    'face state: ' + self._face.last_error,
                    throttle_duration_sec=10.0)
                self._face.last_error = None
```

- [x] **Step 4: rodar e ver passar** — mesmo comando do Step 2.
  Esperado: 45 passed.

- [x] **Step 5: commit** —
  `git add ros2_packages/robot_nav && git commit -m "motion_guard: publica rumo da pessoa pro face_web (JSON atômico em /tmp)"`

---

### Task 2: `face_state.py` + rota `/state` no face_app

**Files:**
- Create: `face_web/face_state.py` (lógica pura, sem flask — testável no
  pytest do sistema)
- Modify: `face_web/face_app.py` (rota `/state` no lugar do comentário-gancho)
- Test: `face_web/test_face_app.py`

**Interfaces:**
- Consumes: JSON da Task 1 (`{"ts": ..., "cbear_deg": int|null}`).
- Produces: `face_state.read_state(path: str, now: float, sign: float=1.0)
  -> dict` — `{'person': False}` ou `{'person': True, 'x': -1..1}`;
  `GET /state` devolve isso em JSON. Task 3 consome via XHR.

- [x] **Step 1: testes que falham** — acrescentar ao `test_face_app.py`
  (usa o `sys.path.insert` que já existe lá):

```python
# ---- fase 2: /state (olhos seguem a pessoa) ------------------------------

def _grava_json(tmp_path, cbear, idade_s=0.0):
    import json
    import time
    p = tmp_path / 'face.json'
    p.write_text(json.dumps({'ts': 0, 'cbear_deg': cbear}))
    if idade_s:
        velho = time.time() - idade_s
        os.utime(str(p), (velho, velho))
    return str(p)


def test_state_sem_arquivo():
    import time
    import face_state
    assert face_state.read_state('/nao/existe.json', time.time()) == \
        {'person': False}


def test_state_stale(tmp_path):
    import time
    import face_state
    p = _grava_json(tmp_path, 30, idade_s=5.0)
    assert face_state.read_state(p, time.time()) == {'person': False}


def test_state_null_e_pessoa_atras(tmp_path):
    import time
    import face_state
    assert face_state.read_state(_grava_json(tmp_path, None),
                                 time.time()) == {'person': False}
    assert face_state.read_state(_grava_json(tmp_path, 135),
                                 time.time()) == {'person': False}


def test_state_pessoa_na_frente_mapeia_e_flipa(tmp_path):
    import time
    import face_state
    p = _grava_json(tmp_path, 45)
    assert face_state.read_state(p, time.time()) == \
        {'person': True, 'x': 0.5}
    assert face_state.read_state(p, time.time(), sign=-1.0) == \
        {'person': True, 'x': -0.5}


def test_state_json_corrompido(tmp_path):
    import time
    import face_state
    p = tmp_path / 'face.json'
    p.write_text('{meia lin')
    assert face_state.read_state(str(p), time.time()) == {'person': False}


def test_state_route(tmp_path):
    pytest.importorskip('flask')
    import face_app
    face_app.STATE_FILE = _grava_json(tmp_path, 45)
    st = face_app.app.test_client().get('/state').get_json()
    assert st == {'person': True, 'x': 0.5}
```

- [x] **Step 2: rodar e ver falhar** —
  `python3 -m pytest face_web/test_face_app.py -q`
  Esperado: 5 FAIL (`No module named 'face_state'`) + rota skip local.

- [x] **Step 3: implementar** — criar `face_web/face_state.py`:

```python
"""Lê o /tmp/motion_guard_face.json (motion_guard, fase 2 da cara) e vira
estado do olhar. SEM flask e SEM ROS de propósito: testável no pytest do
sistema e reaproveitável (o futuro MODO INTERAÇÃO lê o mesmo arquivo)."""
import json
import os

STALE_S = 1.5     # arquivo mais velho que isso = stack caída, sem pessoa
MAX_DEG = 90.0    # pessoa atrás da tela: ninguém vê a cara, ignora


def read_state(path, now, sign=1.0):
    try:
        if now - os.stat(path).st_mtime > STALE_S:
            return {'person': False}
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {'person': False}
    cbear = data.get('cbear_deg')
    if cbear is None or abs(cbear) > MAX_DEG:
        return {'person': False}
    return {'person': True, 'x': round(sign * cbear / MAX_DEG, 3)}
```

  Em `face_app.py`: trocar o import e o comentário-gancho da fase 2 por:

```python
import os
import time

from flask import Flask, jsonify, render_template

import face_state

app = Flask(__name__)

# Sinal do espelhamento olho×mundo: depende de pra onde o iPad aponta no
# tripé — flipar pra -1.0 na demo se o olho seguir pro lado errado.
FACE_GAZE_SIGN = 1.0
STATE_FILE = os.environ.get('FACE_STATE_FILE',
                            '/tmp/motion_guard_face.json')
```

  e a rota (no lugar do comentário "Gancho da fase 2"):

```python
@app.route('/state')
def state():
    """Fase 2 (cara reativa): rumo da pessoa vindo do motion_guard."""
    return jsonify(face_state.read_state(STATE_FILE, time.time(),
                                         sign=FACE_GAZE_SIGN))
```

  (docstring do módulo: atualizar a frase do gancho pra dizer que o /state
  existe e de onde vem o dado.)

- [x] **Step 4: rodar e ver passar** —
  `python3 -m pytest face_web/test_face_app.py -q` → esperado: 9 passed,
  2 skipped (os 2 de flask — o venv do controle_web tem flask mas NÃO tem
  pytest, verificado; a rota real é exercitada no smoke manual da Task 3).

- [x] **Step 5: commit** —
  `git add face_web && git commit -m "face_web: rota /state — rumo da pessoa lido do JSON do motion_guard"`

---

### Task 3: face.js — olhar trava na pessoa (ES5!)

**Files:**
- Modify: `face_web/static/face.js` (estado novo + poll XHR + gate no tick)
- Test: `face_web/test_face_app.py` (léxico já cobre; ampliar teste de
  pedaços obrigatórios)

**Interfaces:**
- Consumes: `GET /state` da Task 2 (`{"person":true,"x":0.5}`).

- [x] **Step 1: ampliar teste de pedaços** — em
  `test_face_js_expressoes_completas`, acrescentar à lista `pedaco`:
  `'pollState'`, `"'/state'"`, `'personHoldUntil'`.

- [x] **Step 2: rodar e ver falhar** —
  `python3 -m pytest face_web/test_face_app.py -q` → 1 FAIL
  ("sumiu do face.js: pollState").

- [x] **Step 3: implementar** — em `face.js`:

  (a) junto do bloco de estado (após `var nextGazeAt = now() + 2;`):

```js
  // Fase 2: enquanto now() < personHoldUntil, tem pessoa na mira — o
  // pollState manda no gazeTarget e o vagar/focused ficam de fora. O hold
  // de 3s segura o olhar quando a pessoa PARA (ela some dos clusters
  // móveis do lidar) — sem ping-pong pessoa/vagando.
  var personHoldUntil = 0;
```

  (b) o começo do `tick(t)` vira (gate novo por cima dos dois ramos atuais):

```js
    if (t < personHoldUntil) {
      // pessoa na mira: gazeTarget é do pollState, ninguém mexe
    } else if (mood === 'focused') {
      gazeTarget.x = 0;
      gazeTarget.y = 0;
    } else if (t >= nextGazeAt) {
      gazeTarget.x = rand(-1, 1);
      gazeTarget.y = rand(-0.5, 0.5);
      nextGazeAt = t + rand(4, 10);
    }
```

  (c) antes do bloco final (`function frame()`):

```js
  // ---- fase 2: olhos seguem a pessoa (poll no /state) --------------------
  // XHR puro (iPad 2!). Qualquer falha — rede, JSON, timeout — é tratada
  // como "sem pessoa": o hold expira sozinho e a cara volta a vagar.
  function pollState() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/state', true);
    xhr.timeout = 250;
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4 || xhr.status !== 200) return;
      var st = null;
      try { st = JSON.parse(xhr.responseText); } catch (e) { return; }
      if (st && st.person) {
        gazeTarget.x = st.x;
        gazeTarget.y = 0.1;
        personHoldUntil = now() + 3;
      }
    };
    xhr.send();
  }
  setInterval(pollState, 300);
```

- [x] **Step 4: rodar e ver passar** —
  `python3 -m pytest face_web/test_face_app.py -q` → tudo verde (léxico ES5
  incluso).

- [x] **Step 5: smoke manual no dev** — rodar
  `controle_web/.venv/bin/python3 face_web/face_app.py`, abrir
  `http://localhost:7000`, e num outro terminal simular a pessoa andando:

```bash
for d in -80 -40 0 40 80 40 0; do
  echo "{\"ts\": 0, \"cbear_deg\": $d}" > /tmp/motion_guard_face.json
  sleep 1
done
```

  Esperado: olhos varrem esquerda→direita acompanhando; 1,5s após a última
  escrita (stale) + 3s de hold, voltam a vagar. `curl localhost:7000/state`
  pra ver o JSON cru.

- [x] **Step 6: commit** —
  `git add face_web && git commit -m "face_web: olhos travam na pessoa (poll ES5 no /state, hold 3s)"`

---

## Depois (fora deste plano)

Deploy na próxima ligada da Pi: `git fetch && git reset --hard origin/main`
+ `colcon build` do robot_nav + restart do face_web; demo com o dono na
frente pra acertar `FACE_GAZE_SIGN`. Registrar no ESTADO_PROJETO.md.
