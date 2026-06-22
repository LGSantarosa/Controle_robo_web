# Door — trava "passo aqui?" geométrica + re-estágio

**Data:** 2026-06-22
**Branch:** `feat/door-para-pra-pessoa`
**Arquivo afetado:** só `ros2_packages/robot_nav/robot_nav/door_crossing.py`

## Problema (pendência A)

Campo 2026-06-19: na 2ª travessia seguida o robô foi **de cara no batente** e parou
~60 cm curto do ponto. A pose estava CERTA (no mapa ele sabia que estava torto) —
**não** era localização. O door **comitou a travessia de uma posição/ângulo ruim**
e atravessou cego. O lateral veio OK do nav2; faltou o **yaw** (apontou pro batente).

Hoje o `crossing` não tem nenhuma verificação de "eu caibo/passo reto daqui?":
só aborta por obstáculo no vão (`gap < gap_min`) e corrige lateral cego
(`cross_k_lat`). Nunca compara a trajetória com a largura da porta.

Existiu uma trava assim (`fit_lat`) no backup `door-redesign-0618-backup`, removida
no revert `4b13a6e`. Era **só lateral** e tripava **colado nos batentes**.

## Decisão

Re-aplicar a trava, mas:
- **geométrica** (pose + os 2 cliques da porta + largura do robô; **sem LiDAR** — o
  LD06 tem fantasma <15 cm na quina e o scan_sanitizer APAGA o batente no crossing,
  ficaria cego justo na colisão que queremos evitar; a falha real já estava na pose);
- **com o yaw** projetado (não só o lateral — foi o yaw que falhou);
- **disparando cedo** (predição, não só colado nos batentes);
- na falha, **re-estagiar sozinho** (recuar reto + re-aproximar + re-checar), reusando
  o `reversing`→`staging` que já existe — NÃO atravessa torto, NÃO depende do nav2
  re-entregar (o ponto-pré-porta é consumido cedo: `navigate_to_pose`/`through_poses`
  já dão o pré-porta por cumprido a <0,15 m, então "torcer pro nav2" não re-entrega).

## Design

### 1. Função pura `will_clear`

```
will_clear(g, s, d, yaw_err, robot_half_width, fit_margin) -> bool
    lat_no_batente = d + (-s) * tan(yaw_err)      # projeta o heading reto até s=0
    fit            = g.half_width - robot_half_width - fit_margin
    return s >= 0 or abs(lat_no_batente) <= fit   # passado o estreito -> sempre passa
```

- `s` = progresso ao longo do eixo de travessia (`<0` = antes da porta), `d` = offset
  lateral, `yaw_err` = erro de ângulo vs o eixo. Todos já calculados no `update()`
  (`door_progress_lateral`, `crossing_yaw`).
- É o `fit_lat` de antes (`half_width − robot_half_width − fit_margin`) + a projeção
  do yaw até o plano dos batentes. Atenção ao SINAL do termo `tan(yaw_err)` na
  implementação (resolver com TDD: angulado-pro-batente tem que reprovar).

### 2. Onde a trava age (dispara cedo)

- **Gate `rotating → crossing`:** só commita a travessia se `aligned AND will_clear`.
  Se alinhou mas a projeção bate → **re-estágio** em vez de `crossing`.
- **Durante o `crossing`, enquanto `s < 0`:** a cada tick, se `not will_clear` →
  **re-estágio**. Passado o batente (`s >= 0`) para de checar (já passou do estreito).

### 3. Re-estágio (mecanismo já existente)

`reversing` (recua RETO, limitado pelo vão traseiro — NUNCA arco) → `staging` (volta
pro ponto a `stage_dist`=0,6 m no eixo) → `rotating` (re-alinha o yaw, `align_yaw`=3°)
→ re-testa `will_clear` → `crossing`.

Limitado por `escape_max_count` (3): re-estagiou 3× e ainda não passa → `_abort` pro
nav2 (último recurso). Reusa o contador/máquina do escape; o re-estágio por fit é só
um **gatilho novo** que entra no mesmo `reversing`.

### 4. Config nova (volta do backup, ROS live-tunable)

- `robot_half_width: 0.25` (m — meia-largura medida roda-a-roda 0,50)
- `fit_margin: 0.13` (m — folga subtraída do vão; **knob de campo nº1**)

Resto reusado. Sem mexer em web, nav2 yaml, nem na estrutura da máquina de estados.

### 5. Testes (TDD)

- Unit `will_clear`: centrado+reto passa; centrado+angulado-pro-batente reprova;
  lateral grande reprova; `s>=0` sempre passa; caso da porta real (~0,93 m → fit
  ~0,085 m).
- Máquina de estados: alinhado-mas-`not will_clear` → `reversing` (não `crossing`);
  deriva de yaw no `crossing` (`s<0`) → `reversing`; 3 re-estágios → `_abort`.
- Manter verdes os 31 testes do door + 11 do `door_geom`.

## Risco residual (tunar em campo)

A projeção reta é pessimista (ignora a correção `cross_k_lat/yaw` andando). Se
re-estagiar à toa, **afrouxar `fit_margin`**. Esse é o ajuste de campo nº1.

## Fora de escopo

- Largura por LiDAR (fase 2, se a geométrica não bastar).
- Melhorar o alinhamento de yaw em si (pendência B) — a trava só impede o ram e
  re-tenta; se o re-align continuar apontando pro batente, é outra frente.
