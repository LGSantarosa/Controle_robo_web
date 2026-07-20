# motion_guard: soltar por PLANO ASSENTADO, não por relógio

**Data**: 2026-07-20
**Bug alvo**: curva de ~70° ao retomar depois de um `blocked`
**Arquivo**: `ros2_packages/robot_nav/robot_nav/motion_guard.py`

## Problema

Robô indo pro goal, pessoa para na frente, o guard trava (`blocked`). A
pessoa sai, o robô retoma — e em vez de seguir em frente **vira ~70° pro
lado**, anda um pouco e só então volta pra rota.

A causa está confirmada nos dois ambientes. No instante em que o guard
solta, o **global plan já nasce torto**, contornando a pessoa que ainda
está no costmap. O `path_follower` só segue fielmente esse plano. Conforme
o costmap limpa, o plano endireita e o robô volta. Isso descarta o
seguidor pegar carrot lateral e o RotationShim inventar giro.

A magnitude depende de quanto da pessoa ainda está no costmap na hora do
release:

| saída da pessoa | erro do plano no release | pico de desvio |
|---|---|---|
| teleporte (sim) | 2-35° | 15-50° |
| **andando ~0.6 m/s** (sim) | 15-19° | **60-87°** |

No real (`log/pi_2026-07-17/`, 3 saídas de blocked da run da rua) o desvio
executado bate 1:1 com o erro de mira no release: +22.4°→+18..33°,
−34.7°→−25.5°, −42.9°→−43.3°.

**Raiz da decisão errada**: o release é puramente temporal
(`motion_guard.py:414-423`, `t - _last_moving_t < clear_time`, 5 s). O
relógio não sabe nada sobre o plano, então o robô arranca **comprometido
com um contorno que vai morrer em 2 s**.

Por que não só afinar o `clear_time`: qualquer valor fixo erra dos dois
lados. Curto demais solta no meio do contorno; longo demais castiga o caso
em que a pessoa sumiu rápido e o plano já está limpo.

## Solução

Trocar o gatilho temporal por uma **condição medida**: depois do
`clear_time` vencer, só liberar de verdade quando o rumo do plano parar de
balançar. Espera exatamente o necessário — pessoa que sumiu rápido libera
rápido, pessoa saindo andando segura até assentar.

Comportamento escolhido enquanto o plano balança: **robô parado**
(estende o `blocked`). Não anda reto nem gira com teto.

### 1. Onde entra

Em `MotionGuard.filter()` (`motion_guard.py:405`), depois do timer vencer
e antes do ramo `slowing`. Um latch marca que o nó esteve em `blocked`;
quando o `clear_time` expira, o release só acontece se o plano estiver
assentado (ou se um dos escapes abaixo disparar).

Ré (`vx < 0`, que afasta do móvel à frente) continua passando, mesma regra
do `blocked` de hoje.

### 2. Métrica de estabilidade

O nó assina `/plan` (`nav_msgs/Path`). O plano já vem em frame `map`, então
não precisa de TF.

A cada plano recebido: calcular o rumo do início do caminho até o ponto a
~0.6 m ao longo dele, e empilhar `(t, rumo)` numa janela deslizante.

**Estável** = amplitude da janela (máx − mín, com wrap em ±π) abaixo de
`settle_tol_deg`, com pelo menos `settle_min_samples` amostras.

Amplitude, e não delta entre replans consecutivos: um plano que gira
devagar e constante tem delta pequeno a cada ciclo mas amplitude grande, e
o critério de delta o leria como estável — exatamente o caso da pessoa
saindo andando, que é o pior do bug.

Rumo em frame absoluto (`map`), não relativo ao robô, pra que a rotação do
próprio robô não contamine a medida.

### 3. Knobs

Todos entram em `_CFG_PARAMS` (`motion_guard.py:512`), portanto afináveis
ao vivo por `ros2 param set`.

| knob | default | razão |
|---|---|---|
| `settle_enabled` | `True` | `False` = comportamento de hoje, exato |
| `settle_window` | `1.0` s | plano endireita em ~2-3 s no sim |
| `settle_tol_deg` | `8.0` | ~metade do erro medido no release ruim (15-19°) |
| `settle_max` | `4.0` s | teto próprio, folgado abaixo do `guard_hold_max` |
| `settle_min_samples` | `3` | não declarar estável com uma amostra solta |
| `settle_plan_stale` | `1.0` s | sem plano fresco → libera |

### 4. Fail-open

Todo caminho de dúvida **libera**:

- `/plan` nunca recebido → libera
- plano mais velho que `settle_plan_stale` → libera
- menos que `settle_min_samples` na janela → libera
- `settle_max` estourado desde o fim do `clear_time` → libera à força
- `settle_enabled=False` → libera

O estado `settling` só consegue **parar** o robô, e é limitado no tempo.
Nenhum caminho novo o deixa andar em situação em que hoje ele pararia.

### 5. Estado publicado: continua `blocked`

`unstuck_supervisor.py:1240` faz match exato `msg.data == "blocked"`.
Publicar um estado novo `settling` derrubaria o standdown do unstuck, que
poderia disparar ré justamente enquanto o robô está parado de propósito —
o BO de campo de 2026-07-10 documentado nesse mesmo trecho (unstuck avançou
em cima de pessoa durante blocked).

Portanto: **no fio (`motion_guard/state`) continua `blocked`**; o
`settling` aparece só na coluna de estado do CSV, pra análise. Zero
mudança no `unstuck_supervisor.py`.

Consequência boa: o teto `guard_hold_max = 20.0` s
(`unstuck_supervisor.py:304`) já cobre este estado como rede de segurança
externa. O `settle_max = 4.0` s existe pra nunca chegar perto dele.

### 6. Testes

Em `test/test_motion_guard.py` (já existe), sobre a classe pura
`MotionGuard`, sem ROS:

1. plano estável no fim do `clear_time` → libera na hora (sem regressão)
2. plano oscilando → segura em `settling`
3. deriva lenta e constante (o caso da pessoa andando) → segura
   — este é o teste que falha com o critério de delta e passa com amplitude
4. `settle_max` estourado → libera à força mesmo com plano instável
5. nenhum plano recebido → comportamento idêntico ao de hoje
6. ré (`vx < 0`) passa durante `settling`
7. `settle_enabled=False` → comportamento idêntico ao de hoje

## Fora de escopo

- **Afinar `clear_time`**: independente disso, e afinável ao vivo.
- **`path_follower` congelado em `turning` com yaw parado** por segundos
  (achado lateral da mesma análise dos CSVs de 07-17, provável guard
  re-bloqueando entre releases). Bug separado.
- **Tornar o `path_follower` afinável ao vivo**: ele lê params uma vez no
  `__init__` e congela em `self.cfg` (`path_follower.py:388-389`), sem
  callback de set. Melhoria real, mas não é este bug.

## Validação

Mudança no nó de segurança → não vai blind pro campo. Ordem: testes
unitários → sim (`sala_grande`, o mesmo rig de repro: pessoa saindo
ANDANDO, que é o caso feio) → comparar pico de desvio com/sem
`settle_enabled` → só então real.
