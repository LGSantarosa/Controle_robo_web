# motion_guard: release por corredor livre + probe blindado

**Data:** 2026-07-21
**Componente:** `ros2_packages/robot_nav/robot_nav/motion_guard.py` (+ `test/test_motion_guard.py`)
**Escopo:** só o "confirmar que pode sair". O arranque tardio/veio-pra-cima (#1) fica **aberto e separado**.

## Problema

Run 07-20 (16:04): pessoa parada no caminho bloqueou o robô (correto), depois saiu, mas
o guard ficou `blocked` **~27s seguidos** — sendo ~24s com `n_moving=0` (ninguém em
movimento) e nada real atrapalhando. Evidência: `motion_guard.csv`, `blocked` de rel 73,1s
até 100,3s; freeze_capture mostra o robô parado em `px=1,44` o tempo todo.

**Causa-raiz (código):** a VIGÍLIA (`observe()` linhas 386-406, `hold_still_max=20.0`).
Quando um móvel bloqueia e para, a vigília renova o latch (`self._last_moving_t = t`,
linha 403) enquanto `_presence()` achar **qualquer** ponto de scan dentro de
`hold_still_radius=0.5m` do **último centróide** (ponto velho), até o teto de 20s + 5s de
`clear_time`. Ou seja: segura enquanto sobra qualquer retorno perto de onde a pessoa
esteve — não olha para onde o robô precisa ir. Depois do "standdown total enquanto guard
bloqueia" (`2f4b082`), nada resgata: o robô só espera os 20s vencerem.

A vigília foi criada de propósito (07-10) para o robô **não empurrar quem parou**. O
falso-positivo é o outro lado da mesma moeda.

## Objetivo

Inverter a lógica: em vez de **segurar enquanto houver algo perto do ponto velho**,
**soltar assim que o corredor à frente do plano estiver comprovadamente livre** — e só
fazer um micro-passo de teste se travar demais por um retorno que **não parece pessoa**.

Preserva a intenção original: pessoa parada **no caminho** → segue travado. Corrige o
falso: pessoa saiu → solta rápido.

## Decisões (fechadas no brainstorm)

1. **Misto:** release passivo quando limpa; micro-passo ativo só como último recurso.
2. **Zona livre = corredor à frente do PLANO** (arco inicial do `/plan`), não o ponto velho.
3. **Timing equilibrado:** corredor limpo por **1,2s** → solta; **10s** travado → considera o probe.
4. **Probe blindado:** só cutuca se o retorno persistente **não é tamanho-de-pessoa**;
   se parece gente, **segue travado**. Bolha dura / collision_monitor como backstop.

## Design

### 1. Checagem de corredor livre (novo — roda no `observe()`, que já tem o scan)

O ponto essencial que difere do `in_corridor` de hoje: a checagem de release olha
**todos os pontos do scan** (`pts`), não só `moving_clusters`. Pessoa parada some do diff
de movimento mas **continua no scan** — é isso que tem que segurar o release.

- Geometria: corredor ao longo do **rumo inicial do plano**. Reusa o arco que
  `observe_plan()` já amostra (`_plan_hdg`, `settle_lookahead`); o rumo do 1º trecho do
  plano define a direção. Sem plano fresco (`settle_plan_stale`), cai no rumo do robô
  (base_link), como fail-open.
- Faixa: meia-largura **`corridor_half_w` (0.35m, reusa)**, comprimento **`release_len`
  (novo, 1,5m)** — curto: só o que o robô vai pisar já-já, não o corredor inteiro de 2,5m.
- Filtro: descarta ponto colado em **parede do mapa** (`ghost_map.occupied_near`, mesmo
  critério `wall_near` da vigília) — parede não é presença.
- Resultado por scan: `self._corridor_occupied = (existe ponto não-parede na faixa)`.
- `self._corridor_clear_since`: marcado quando o corredor fica limpo; resetado quando ocupa.

### 2. Latch / release (`filter()`)

- Enquanto `_corridor_occupied` → **mantém `blocked`** (substitui a vigília por ponto velho).
- Corredor limpo por **`release_confirm` (1,2s)** contínuos → **libera** (segue então pelo
  caminho normal de hoje: `clear_time` + settle do plano, que já existem e não mudam).
- **Fail-open:** `settle_enabled=False` ou sem `/plan` fresco → comportamento pré-mudança
  exato (vigília atual). A troca é aditiva e reversível por flag.
- Anti-flicker: os 1,2s de confirmação já cobrem; sem release em 1 scan solto.

### 3. Micro-passo blindado (último recurso)

- Condição: `blocked` contínuo **> `probe_after` (10s)** E o(s) ponto(s) que ocupam o
  corredor formam cluster **não tamanho-de-pessoa**.
  - "não é pessoa" = cluster do corredor com menos de `probe_person_min_pts` pontos
    **ou** extensão espacial abaixo de `probe_person_min_span` (fantasma/reflexo/quina
    esparsa). Reusa `_cluster()` e `min_cluster_points`.
- Ação: emite creep **`probe_vx` (0,05 m/s)** por no máximo **`probe_dist` (0,15m)**,
  depois reavalia o corredor. `wz=0` (sem giro, regra do dono).
- Se o retorno **parece pessoa** → **não cutuca**, segue `blocked` indefinido (até a
  pessoa sair e o corredor limpar). Bolha dura (`freeze_dist`) e collision_monitor
  seguem ativos como backstop físico durante o creep.

### 4. Parâmetros novos no `GuardConfig`

| nome | default | papel |
|------|---------|-------|
| `release_by_corridor` | True | flag-mestra: False = vigília atual exata (rollback) |
| `release_len` | 1.5 | m — alcance do corredor de release à frente |
| `release_confirm` | 1.2 | s — corredor limpo contínuo p/ soltar |
| `probe_after` | 10.0 | s — travado antes de considerar o micro-passo |
| `probe_vx` | 0.05 | m/s — creep do micro-passo |
| `probe_dist` | 0.15 | m — deslocamento máx por probe |
| `probe_person_min_pts` | 5 | cluster ≥ isto pontos = parece pessoa → não cutuca |
| `probe_person_min_span` | 0.12 | m — extensão do cluster ≥ isto = parece pessoa |

Reusa: `corridor_half_w`, `wall_near`, `settle_lookahead`, `settle_plan_stale`,
`min_cluster_points`, `ghost_map`, `_cluster`, `observe_plan`/`_plan_hdg`.

## Fluxo de dados

`observe(pts, pose)` → calcula `_corridor_occupied` + `_corridor_clear_since` (usa
`pts`, `_plan_hdg`, `ghost_map`). `observe_plan(poses)` → mantém o rumo do plano (já
existe). `filter(t, vx, wz)` → decide `blocked`/release/probe lendo essas flags. Segue o
padrão atual (observe escreve estado, filter lê).

## Testes (`test/test_motion_guard.py`)

1. **Pessoa sai do corredor** → solta em ~1,2s (não 20s). *(o teste-chave do falso-positivo)*
2. **Pessoa parada no corredor** → segue `blocked` além de 1,2s e além de 20s.
3. **Fantasma pequeno (não-pessoa) após 10s** → probe dispara (`vx>0`, ≤ `probe_vx`).
4. **Cluster tamanho-pessoa após 10s** → **sem** probe (`vx=0`), segue travado.
5. **Fail-open sem plano** (`_plan_hdg` vazio / stale) → comportamento da vigília de hoje.
6. **Ponto só em parede do mapa no corredor** → conta como livre (solta), não segura.

## Risco / rollout

- Mexe só em `motion_guard.py` + testes. Componente delicado → **valida no sim primeiro,
  não vai blind pra Pi** (regra da memória: mudança grande = mega revisar).
- Tudo atrás da flag-mestra `release_by_corridor` (False = vigília atual exata); fail-open
  para o comportamento atual quando falta plano.
- **Fora de escopo:** o #1 (arranque dispara 0,30 m/s reto em cima de pessoa parada, freio
  tardio por lag ~2s + só-detecta-movimento). Item aberto, tratado à parte.
