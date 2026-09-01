# Desenho — reativar o `door_crossing` para a fresta A da arena (2026-09-01)

> **Status: DESENHO, nada implementado.** Nenhuma linha de código foi escrita ou
> alterada para este documento. Escrito para revisão externa (Codex) antes de
> qualquer implementação.
>
> Branch: `arena-galpao`. Prazo da prova: **05/09**. Tudo abaixo foi medido **no
> simulador**; nada foi ao robô real.

> ---
>
> 🔴 **RETRATAÇÃO (2026-09-01, review do Codex) — leia antes de usar este
> documento.** Quatro afirmações abaixo estão **erradas** e uma quinta é
> imprecisa. Elas descrevem o `door_crossing` de **12/06**, não o código em vigor
> (que mudou em **19/06**). Detalhe e conferência em `DIARIO_ARENA.md` §2D;
> BOs 70–73 na §5. Resumo do que **não** vale:
>
> 1. **§3, diagrama** — não existe `IDLE → STAGING`. O arme vai **direto para
>    `rotating`** (`door_crossing.py:381`).
> 2. **§2 ("argumento decisivo") e §4.3, linha "entra em CROSSING"** —
>    `align_lat = 0.08` **não participa de nenhuma decisão** (única ocorrência:
>    string de log, `:613`). O gate real é `|yaw| < 3°` **+** `will_clear()`
>    (`fit = 0,15 m`). A pose de `+0,120` que eu apresentei como barrada **passa**
>    (projeção 0,039); a do 1º raspão (`+0,259`) é que reprova (0,178 > 0,15).
> 3. **§4.9 e §4.10-item-2** — o teste "não pode estar em `crossing`" **não é
>    vermelho** (passa com o nó `idle`), e "não entra com `|lat| > align_lat`"
>    não é critério de coisa nenhuma.
> 4. **§4.4-(a), os 18,7 cm** — descrevem o giro no **ponto de preparação**, que
>    o arme direto **não usa**. O point-turn inicial acontece onde o robô entrar
>    na zona (raio 1,1 m); há posições de arme em que o círculo varrido
>    (r = 0,354) **invade o bloco**.
> 5. **§5.2, "o bloqueador de integração"** — são **dois**. O outro:
>    `_pick_door` exige a porta em `_cleared`, que só é populado por um goal
>    **SUCCEEDED com o robô na zona** (`:330`, `:359-367`). A rota da arena não
>    tem esse goal → o nó **nunca arma**.
>
> **Nada foi implementado. O desenho precisa voltar à prancheta antes de virar
> código.**
>
> ---

---

## 0. Como ler este documento

O leitor externo não tem o histórico. O mínimo necessário:

- O robô é um **skid-steer 0,5 × 0,5 m**, 4 rodas, LiDAR no centro. Ele **não faz
  curva em arco** para realinhar: gira **no lugar** (point-turn), e abaixo de
  ~1,7 de comando o giro não vence o atrito (zona morta medida em campo).
- A stack é **Nav2** (`robot_nav`), com um nó próprio `path_follower` que segue o
  `/plan` do Theta\* em **reto + giro-no-lugar**, e um `collision_monitor` como
  reflexo geométrico final.
- A prova de 05/09 é uma **arena de galpão** com 4 cones e 4 **frestas** (vãos
  entre blocos) de 0,60 / 0,70 / 0,80 / 0,90 m. **Passar pela fresta é atalho
  OPCIONAL** — cada uma tem contorno (`tools/gera_arena_galpao.py:41-46`).
- A meta declarada pelo dono para esta prova é **"destemido + MUITO preciso"**.
  **Velocidade saiu do critério.** Isso importa para avaliar o custo de tempo da
  seção 7.

---

## 1. O problema, medido (não inferido)

### 1.1 O evento

Em 14 voltas simuladas na arena, **12 passaram pela fresta A** (vão de 0,90 m) e
**2 foram pelo contorno**. Das 12, **uma bateu**: a `noguard3` registrou **9
COLISÃO + 48 raspões** e folga mínima **0,0000** (penetração).

### 1.2 A geometria exata

De `tools/gera_arena_galpao.py:43` e `:26`:

```
('A_fresta90', 'x', 7.5, [(0.30, 1.80), (2.70, 4.20)], 0.90, 'contorno por y > 4.20')
ESP_BLOCO = 0.60
```

- Blocos no plano **x = 7,5**, com **0,60 m de espessura** → o túnel ocupa
  **x ∈ [7,20 ; 7,80]**.
- O vão vai de **y = 1,80** a **y = 2,70** → **eixo em y = 2,25**, meia-largura
  **0,45 m**.
- Batentes (os dois cantos do vão): **(7,50 , 1,80)** e **(7,50 , 2,70)**.
- A travessia é ao longo de **+x**; o heading do eixo é **0°**.

### 1.3 O discriminante — e a retratação

Pose de cada volta no plano dos blocos (x = 7,5), extraída de `colisao.csv`
(x/y/yaw/folga a 20 Hz; o oráculo usa o **retângulo exato 0,5 × 0,5 girado pelo
yaw**, `log/sim_ab/colisao.py:18`):

| volta | desvio do eixo (y − 2,25) | yaw | folga mín na fresta A |
|---|---|---|---|
| 11 voltas **sem contato** | **−0,081 … +0,078 m** | −8° … −16° | 0,045 … 0,101 |
| **`noguard3`** (bateu) | **+0,120 m** | **−10,7°** | **0,0000** |

**O yaw da volta que bateu está no meio do pelotão.** O que ela tem de único é o
**desvio lateral**.

> ⚠️ **Retratação registrada (BO 69, `DIARIO_ARENA.md` §2B.10, commit `041112c`).**
> Uma versão anterior deste diagnóstico dizia que a `noguard3` "entrou torta, yaw
> −5,4°, contra −13° a −26° das boas". Isso veio de **uma amostra solta** e está
> **errado**. O item de backlog derivado dela ("nada corrige o **rumo**") estava
> mal formulado: o que não existe é controle do **erro lateral** contra o eixo do
> vão antes da boca. Este desenho substitui aquela formulação.

### 1.4 Onde e como o contato aconteceu

- As 9 COLISÃO estão em **x ∈ [6,90 ; 6,96]** — ou seja **~25 cm ANTES da boca**
  do túnel (que começa em 7,20), com o robô em **y ≈ 2,50** (desvio +0,25). Ele
  encostou na **face frontal do bloco de cima** (o que ocupa y 2,70–4,20)
  enquanto ainda vinha chegando. **Não** foi um aperto dentro do vão.
- Em seguida **raspou por ~3 s** com folga travada em **0,018 m** e yaw
  **congelado em −10,7°**, convergindo o desvio de +0,26 para +0,02: ele se
  **espremeu para dentro raspando**. Nada no sistema reagiu ao contato.
- A volta boa fazia o **oposto**: a `noguard2` chegou em x = 6,90 com desvio
  **+0,170** e yaw **−24,3°** — apontada para **cortar** em direção ao eixo — e
  ainda deu um **giro no lugar na boca** (−24,3° → −12,6° com x parado em 7,33),
  entrando com desvio −0,02. **O yaw grande das voltas boas era a correção, não o
  defeito.**

### 1.5 A causa

A perna `cone1(4,5 ; 1,5) → cone2(11,5 ; 1,8)` chega pela **esquerda-de-cima** e
**nada no sistema controla o erro lateral contra o eixo do vão antes da boca**.
Cada volta converge o quanto o plano do Theta\* por acaso cortou. Quem chega com
**desvio grande e ângulo de convergência pequeno** encosta na face do batente
antes de entrar. A fresta A **sempre** foi passagem no fio: folga mínima de
**0,045 a 0,212 m** nas 14 voltas, abaixo de 8 cm em 4 delas. A `noguard3` não
estreou o risco — cobrou.

### 1.6 Quanta margem existe

- **Física:** vão 0,90 − largura varrida por um 0,5 × 0,5 a −12°
  (`0,5·(|cos| + |sen|) = 0,60`) = **0,30 m**, ou seja **±0,15 m** de erro de eixo
  antes do contato geométrico.
- **Para o planejador:** com `robot_radius: 0.32`
  (`config/nav2_params_arena.yaml:328`) o Nav2 enxerga um **círculo de 0,64 m** →
  **±0,13 m**. É por isso que o erro `"start or goal pose are an obstacle"` mora
  justamente aqui (custou o goal 2 da volta `latchN1`).

**Conclusão operacional:** o sistema precisa garantir |desvio lateral| bem abaixo
de 0,13 m **antes** da boca. Hoje ele não garante nada — só mede depois.

---

## 2. Por que reativar o `door_crossing` em vez de escrever um gate novo

O repositório **já tem** a máquina que resolve exatamente este problema:
`ros2_packages/robot_nav/robot_nav/door_crossing.py` (741 linhas), com
`test/test_door_crossing.py` (599 linhas) e spec própria
(`docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md`).

A tolerância dela é `|lat| < 8 cm` e `|yaw| < 3°`.
Ela foi escrita em 06-12 para o mesmo defeito ("o Nav2 não atravessa portas
estreitas: entra torto") e **validada em campo atravessando uma porta real**. Foi
**desativada em 26/06** (`launch/nav2.launch.py:216-230`, bloco comentado) porque
o `path_follower` novo passava a porta da sala nativamente — a porta da sala tem
**1,09 m** de vão, contra os **0,90 m** da fresta A, e a sala não tinha a perna de
aproximação enviesada que a arena tem.

**Alternativas descartadas** (o dono escolheu esta; registro o porquê para o
revisor poder contestar):

| alternativa | por que não |
|---|---|
| Gate de eixo novo dentro do `path_follower` | Reimplementa staging + point-turn + travessia + abort + ré de escape que já existem e têm 599 linhas de teste. Mudança grande no nó que **dirige**, a 4 dias da prova |
| Waypoint no eixo do vão (dado de rota) | Mais barato, mas **não garante** nada: o Theta\* continua livre para cortar entre waypoints; só reduz a probabilidade |
| Não atravessar a fresta de 0,90 (ir pelo contorno) | Elimina o risco e o atalho. Fica como **fallback** (seção 9), não como plano |

**O argumento decisivo, em número:** a tolerância de alinhamento da máquina é
`align_lat = 0.08 m` (`door_crossing.py:186`). A `noguard3` chegou com **0,120 m**
de desvio. **0,120 > 0,080 → a máquina não a teria deixado atravessar**: ela
teria ficado em `rotating`/re-estagiando até fechar o eixo, ou abortado para o
Nav2. As 11 voltas boas (|desvio| ≤ 0,081) passam raspando na tolerância — o que
significa que a máquina vai **atuar** em quase todas, não só na que bateu. Isso é
custo (seção 7) e é o principal risco a revisar.

---

## 3. O que a máquina já faz (estado atual do código)

Máquina de estados pura (sem ROS) em `DoorCrossing.update()`
(`door_crossing.py:354-510`); cola de I/O em `main()` (`:512-741`).

```
IDLE ──(porta marcada dentro da zona, à frente, goal ativo, nav indo pra frente)──►
STAGING ──(chegou ao ponto de preparação no eixo)──► ROTATING
       ◄──(re-estagia: will_clear reprovou)──┘
ROTATING ──(|lat| < 8 cm E |yaw| < 3°, por 5 ticks)──► CROSSING
CROSSING ──(passou do centro + exit_margin)──► IDLE (devolve pro Nav2)
qualquer ──(abort: pose/goal/scan/vão/timeout)──► IDLE
STAGING/ROTATING ──(nariz na parede ou substuck)──► REVERSING ──► STAGING
```

Interfaces (todas já existentes):

| | |
|---|---|
| **entra** | `/doors` (String JSON, latched), `/scan`, `/nav_vel`, TF `map→base_link`, status das ações `navigate_to_pose` / `navigate_through_poses` |
| **sai** | `/door_vel` (Twist) e `/door_zone` (String JSON: `idle`/`approaching`/`staging`/`rotating`/`crossing`/`reversing`) |
| **arbitragem** | `door_vel` entra no `twist_mux_auto` com **prioridade 20**, acima de `follow_vel` (15) e `nav_vel` (10) — `config/twist_mux_auto.yaml` |

---

## 4. Os 10 itens do checklist, respondidos

O dono exigiu que o desenho fechasse 10 pontos. Cada um abaixo traz **o valor que
já existe no código** e **a conta específica da fresta A**.

### 4.1 Ponto exato onde o alinhamento assume

`zone_radius = 1.1 m` do **centro do vão** (`door_crossing.py:174`), com a porta
dentro de `approach_bearing = 70°` da frente do robô (`:175`) **e** `goal_active`
**e** `nav_forward` (o Nav2 querendo ir em frente, `nav_engaging()`, `:94`).

Para a fresta A: arma quando o robô entra no círculo de raio 1,1 m em torno de
**(7,50 ; 2,25)**.

**Verificação de que isso dispara nesta rota:** as 12 travessias medidas cruzaram
x = 7,5 com y entre 2,169 e 2,370 — todas passaram a menos de 0,13 m do centro,
logo passaram pelo círculo de 1,1 m. A perna vem de (4,5 ; 1,5), então entra pelo
lado x < 7,5 (`side = −1`).

**Item aberto para o revisor:** `zone_radius` (1,1) tem que ser **≥**
`stage_dist` (0,6) — é, com folga. Mas o comentário do código
(`door_crossing.py:169-173`) diz que 1,1 foi escolhido para armar 0,1 m antes de
um **ponto pré-porta de 1,0 m** que existia na rota da sala. **A rota da arena não
tem ponto pré-porta** (`maps/routes/arena_galpao.json` só tem os standoffs dos
cones). O robô vai entrar na zona **em movimento, a ~0,9 m/s**, não parado em um
waypoint. Isso é diferença real em relação ao contexto validado em campo e é o
**risco nº 1** deste desenho (seção 8.1).

### 4.2 Heading-alvo e como é calculado

`door_geometry(a, b)` (`:42`) monta, a partir dos **dois batentes**:
tangente `t = (b − a)/|b − a|` e **normal `n = (−t_y , t_x)`** = eixo de
travessia. `crossing_yaw(g, side)` (`:67`) devolve o heading-alvo como
`atan2(side·n_y , side·n_x)`.

Para a fresta A, com `a = (7,50 ; 1,80)` e `b = (7,50 ; 2,70)`:
`t = (0 ; 1)`, `n = (−1 ; 0)`, e com `side = −1` (aproximação pelo oeste) →
**heading-alvo = 0,0°** (leste puro). Confere com a geometria: o túnel é ao longo
de +x.

**O heading-alvo vem dos dois pontos marcados, não do LiDAR.** Consequência
documentada no código (`:202-207`): a correção lateral persegue o **eixo dos dois
cliques**, não o corredor real — por isso ela é **desligada depois do centro**
(`cross_lat_off_s = 0.0`), para não deixar o robô anguladinho na saída. Se os dois
pontos forem marcados errados, a máquina persegue um eixo errado com convicção.
Ver 5.1: na arena eles são **gerados da mesma fonte que gera o mundo**, não
clicados a olho — o que remove essa classe de erro no sim, mas **não** no robô
real (onde o mapa vem de SLAM e pode estar torto).

### 4.3 Tolerâncias de entrada/saída, e a histerese

| tolerância | valor | onde |
|---|---|---|
| entra em CROSSING | `\|lat\| < 0,08 m` **e** `\|yaw\| < 3,0°` | `:186-187` |
| **histerese temporal** | `align_stable = 5` ticks **consecutivos** dentro da tolerância (a 20 Hz = 0,25 s) | `:188` |
| solta para o Nav2 | passou do centro + `exit_margin = 0,5 m` | `:222` |
| **histerese pós-travessia** | `crossing_cooldown = 8,0 s` — não re-arma depois de cruzar | `:225` |
| **histerese pós-abort** | `retrigger_cooldown = 3,0 s` | `:224` |

A histerese aqui é **de tempo e de evento**, não de banda dupla: o `align_stable`
impede que um tick sortudo dispare a travessia, e os dois cooldowns impedem o
liga-desliga na saída (bug de campo real: a ré pós-porta trazia o robô de volta
para a zona e re-armava a porta que ele **já tinha atravessado**, `:225`).

**Ponto para o revisor:** não existe banda de saída em `lat`/`yaw` — uma vez em
CROSSING, a máquina **não volta** para ROTATING por piora de alinhamento; ela
corrige em malha fechada (`cross_k_lat = 1.5`, `cross_k_yaw = 2.0`, teto
`cross_wz_max = 0.8 rad/s`) ou re-estagia via `will_clear` (4.4). Isso é
deliberado (giro no lugar com meio robô dentro do vão é pior), mas é uma
assimetria que merece revisão explícita.

### 4.4 Garantia de que o point-turn tem espaço para o volume varrido

Duas garantias, uma geométrica e uma dinâmica.

**(a) Onde o giro acontece.** O ponto de preparação é
`centro − side·stage_dist·n`, com `stage_dist = 0.6 m` (`:182`) → para a fresta A,
**(6,90 ; 2,25)**, no eixo.

Um 0,5 × 0,5 girando no lugar varre um **círculo de raio circunscrito
`0,25·√2 = 0,354 m`**. Do ponto (6,90 ; 2,25), o obstáculo mais próximo é o canto
do bloco em **(7,20 ; 2,70)** (e o simétrico em (7,20 ; 1,80)):

```
distância = hypot(7,20 − 6,90 ; 2,70 − 2,25) = hypot(0,30 ; 0,45) = 0,541 m
margem   = 0,541 − 0,354 = 0,187 m
```

**O point-turn no ponto de preparação cabe, com 18,7 cm de folga.** Esta conta
vale **só para a fresta A**; a seção 6 exige repeti-la antes de marcar qualquer
outra fresta (a de 0,60 dá margem menor e provavelmente **não** cabe — ver 6.2).

**(b) Trava "eu passo daqui?" antes de comitar.** `will_clear()` (`:72`) projeta a
trajetória reta até o plano dos batentes considerando o erro de yaw, e compara com
`fit = half_width − robot_half_width − fit_margin`:

```
fit = 0,45 − 0,25 − 0,05 = 0,15 m
```

Se a projeção não passa, a máquina **re-estagia** em vez de tentar. A partir de
`commit_s = −0,15 m` (robô com mais de meio corpo no vão) ela **para de re-checar
e comita para a frente** — dar ré meio-atravessado foi medido como pior.

### 4.5 Travessia reta e critério de término

`CROSSING` (`:481-510`): `vx = cross_speed = 0,22 m/s`, correção de eixo em malha
fechada com teto `cross_wz_max = 0,8 rad/s` (**micro-correção, nunca giro**), e a
correção **lateral desliga passado o centro** (`cross_lat_off_s = 0.0`, `:202-207`) — depois
do centro ele sai **reto**, só segurando o yaw.

**Término:** progresso `s ≥ exit_margin = 0,5 m` além do centro → estado volta a
`idle` e o `door_vel` para de publicar; com o timeout de 0,5 s do
`twist_mux_auto`, o `follow_vel`/`nav_vel` reassume. Para a fresta A: solta em
**x = 8,0 m**, ou seja **0,20 m depois de sair do túnel** (que termina em 7,80).

### 4.6 Interação com `unstuck`, Nav2 e `motion_guard:=false`

**`unstuck_supervisor`:** já existe **standdown** explícito
(`unstuck_supervisor.py:279-290` e `:517-527`): enquanto `/door_zone` indica
`staging`/`rotating`/`crossing`/`reversing`, o unstuck **não manobra**. O motivo
está no código: sem isso, o unstuck girava o robô no meio do alinhamento e o
`door_crossing` nunca fechava a tolerância (ciclo de 15 em 15 s). **Não é código
novo — é código existente que hoje está inerte porque o `door_crossing` nunca
publica.**

Isso resolve, de quebra, um efeito colateral do evento medido: a `noguard3` teve
um disparo do `unstuck` (`reason=near`) aos 50,8 s **antes** da fresta.

**Nav2:** o `door_crossing` só arma com **goal ativo** e com o Nav2 **querendo ir
para a frente**; ele **não cancela nem substitui** o goal — apenas ganha o mux
enquanto conduz, e devolve. Se abortar, o Nav2 nunca soube que saiu do ar.

**`motion_guard:=false` (perfil `--arena`):** o guard é o vigia de **pessoa**; foi
desligado na arena por decisão do dono em 31/08, porque bloqueava ~27 s em cima
dos cones numa prova sem pessoas. Com ele desligado o `twist_mux_auto` publica
**direto** em `auto_vel_raw`. O caminho do `door_vel` fica:

```
door_vel (prio 20) → twist_mux_auto → auto_vel_raw → collision_monitor → auto_vel
                  → twist_mux FINAL (prio 10) → cmd_vel → rodas
```

**Ou seja: o `door_vel` PASSA pelo `collision_monitor`**, com ou sem guard.

> 🔴 **Defeito de documentação encontrado ao escrever este desenho, a corrigir na
> implementação:** dois comentários dentro do `door_crossing.py` (`:226-229`, da
> ré de escape, datados de 16/06; a frase está em `:229`) afirmam que **"door_vel fura o collision"**.
> Isso era verdade no **mux único** de junho; deixou de ser no **2-mux de 26/06**
> (`twist_mux_auto.yaml`). A ré de escape foi desenhada sob a premissa de que
> ninguém a filtrava — hoje o `collision_monitor` a filtra. Consequência prática a
> **medir**, não supor: a ré de escape pode ser **atenuada ou zerada** pelo
> `PolygonStop`/`approach` justamente quando o robô está de nariz na parede, que é
> quando ela existe. Ver 8.2.

### 4.7 Comportamento com scan velho ou vão bloqueado

- **Scan velho:** `scan_stale = 0.6 s` (`:552`). Scan mais velho que isso →
  **abort** → devolve para o Nav2. O `gap` entra no `update()` como `None`, e a
  máquina trata ausência de leitura como "não sei", não como "está livre".
- **Vão bloqueado:** `gap_ahead()` (`:130`) mede o vão livre à frente **mascarando
  os batentes** (raio `jamb_r` em torno dos dois pontos marcados) — senão o próprio
  batente contaria como obstáculo e a máquina nunca atravessaria porta nenhuma.
  Se o vão medido < `gap_min = 0.45 m` → **abort**.
- **Nariz na parede:** obstáculo a menos de `escape_front_gap = 0.20 m` → ré de
  escape (4.8).
- **`/door_zone` também abre a máscara de batente do `scan_sanitizer`**
  (`scan_sanitizer.py:120`, `:127-132`), que é o filtro de fantasma do LD06 — relevante no
  robô real, inerte no sim.

### 4.8 Aborto e fallback seguro

| gatilho | ação |
|---|---|
| TF `map→base_link` sumiu | abort → Nav2 |
| goal morreu | abort → Nav2 |
| scan > 0,6 s de idade | abort → Nav2 |
| vão à frente < 0,45 m | abort → Nav2 |
| não alinhou em `align_timeout = 15 s` (STAGING+ROTATING) | abort → Nav2 |
| manobra inteira > `total_timeout = 40 s` | abort → Nav2 |
| nariz na parede (< 0,20 m) ou 5 s alinhando sem sair do lugar | **ré reta** de até 0,30 m a 0,25 m/s, travada pelo vão traseiro (`escape_rear_margin = 0.10`, `escape_rear_min = 0.10`) |
| 3 escapes na mesma travessia | abort → Nav2 |

**O fallback é sempre o mesmo: soltar o `door_vel` e devolver para o Nav2**, que
é literalmente o comportamento de hoje (o estado atual do sistema = a máquina
sempre abortada). **O pior caso do desenho é o comportamento atual**, mais o tempo
gasto tentando. Isso é o que torna a mudança defensável a 4 dias da prova.

> ⚠️ **A ré NUNCA é em arco** (regra dura do projeto para skid-steer) — a ré de
> escape é reta e travada pelo vão traseiro.

### 4.9 Teste que reproduz a pose da `noguard3`

**Teste vermelho primeiro** (a máquina é lógica pura, sem ROS — dá para alimentar
a pose medida direto):

```
dado: porta A = {a: (7.50, 1.80), b: (7.50, 2.70)}
      pose = (6.90, 2.509, −7.7°)   ← a amostra REAL da noguard3 no 1º raspão
      goal ativo, nav indo pra frente, scan fresco, vão livre
então: o estado NÃO pode ser 'crossing'
       (desvio lateral = +0.259 m > align_lat 0.08 → tem que estagiar/girar)
e:     o vx comandado tem que ser ≤ stage_speed (0.20), não a velocidade de rota
```

E o par que prova o teste sensível (senão ele passa com a máquina desligada):

```
dado: a mesma porta, pose = (6.90, 2.25, 0.0°)  ← já no eixo, já alinhado
então: depois de align_stable ticks o estado TEM que ser 'crossing'
```

Além do teste unitário, **uma volta no sim** partindo da pose de entrada da
`noguard3` (seção 6.3).

### 4.10 Testes de não-regressão das outras entradas

As 11 travessias sem contato têm pose medida (tabela em `DIARIO_ARENA.md`
§2B.10). O teste de não-regressão alimenta **cada uma** e exige:

1. **nenhuma** delas termina em `abort` por timeout de alinhamento (se abortarem,
   a máquina piorou a volta: o robô perde tempo e volta ao comportamento de hoje);
2. **nenhuma** entra em `crossing` com `|lat| > align_lat`;
3. o número de re-estagiamentos (`rotating → staging`) por travessia é **≤ 1** —
   mais que isso é o *thrash* que o `fit_margin` já causou em campo (`:212`).

O item 3 é o que protege contra o modo de falha mais provável deste desenho: a
máquina **atuando demais** em 11 voltas que já passavam.

---

## 5. O que falta construir (as lacunas reais)

Reativar **não** é só descomentar. As lacunas abaixo são o trabalho de verdade.

### 5.1 As frestas não estão marcadas como porta

`/doors` é alimentado por `maps/<mapa>.doors.json`
(`controle_web/map_service.py:211-245`), schema:

```json
{"doors": [{"id": 1, "a": [7.50, 1.80], "b": [7.50, 2.70]}]}
```

(`DoorStore.MIN_W = 0.4`, `MAX_W = 2.0` → 0,90 é aceito.)

**Proposta:** gerar `maps/arena_galpao.doors.json` **a partir de
`tools/gera_arena_galpao.py`**, que é a mesma fonte que gera o mundo. Os batentes
não são clicados a olho: são as bordas dos blocos, exatas. Isso elimina a classe
de erro "eixo marcado torto" **no sim** — e **não** a elimina no robô real, onde o
mapa vem de SLAM (ver 8.3).

**Proposta de escopo: marcar SÓ a fresta A nesta rodada.** É onde o defeito foi
medido, e a regra do projeto é uma mudança pequena por vez. As frestas B (0,70) e
D (0,80) entram depois, com a conta de 4.4-(a) refeita para cada uma. A fresta C
(0,60) **não deve ser marcada**: com `robot_radius 0.32` o Nav2 já a trata como
parede, e marcar uma porta que o planejador não usa só cria uma zona armada
inútil.

### 5.2 O harness do sim não publica `/doors`

**Este é o bloqueador de integração.** `/doors` só é publicado por
`controle_web/app.py` (via `MapBridge`/`DoorStore`), e o harness A/B
(`tools/sim_ab/run_one.sh`) **não sobe o `controle_web`**. Sem isso, o
`door_crossing` sobe, não recebe porta nenhuma e fica `idle` para sempre — e a
volta rodaria **exatamente igual à de hoje**, com a diferença de que eu poderia
achar que "testei".

Duas saídas; **recomendo a (a)**:

| | opção | prós / contras |
|---|---|---|
| **(a)** | **Parâmetro `doors_file` no nó**: se não vazio, carrega o JSON do disco no arranque (e `/doors`, se chegar, sobrescreve) | ~15 linhas na cola de I/O, testável, não arrasta o stack web para dentro do harness, e serve igual no robô real |
| (b) | Subir o `controle_web` dentro do harness A/B | Arrasta Flask + MapBridge para dentro de toda volta A/B; mais superfície, mais órfãos de processo (já houve BO de órfão derrubando o `/clock`) |

### 5.3 Fiação do launch

Descomentar `launch/nav2.launch.py:225-230` **não basta como desenho**: hoje o
bloco sobe **incondicionalmente**. Proposta: subir o `door_crossing` **só quando
houver portas para atravessar**, via launch arg (`door_crossing:=true`, default
**false**, ligado pelo `--arena`) — pelo mesmo mecanismo que o `--arena` já usa
para desligar o `motion_guard`. Motivo: não quero reativar em `--pi`/`--sim`
comuns um nó com prioridade 20 no mux sem que ninguém tenha pedido.

Isso exige **teste de fiação** no mesmo arquivo que já testa a religação do guard
(`test/test_nav2_launch_guard.py`), e o teste tem que ser **sensível** (provado
injetando defeito), não uma asserção vazia — foi exatamente esse o BO 66 desta
branch.

### 5.4 Higiene do que ficou obsoleto

- Corrigir os comentários de 16/06 que dizem que **"door_vel fura o collision"**
  (4.6) — hoje é falso e induz a erro quem for mexer na ré de escape.
- **A docstring do módulo (`door_crossing.py:10`) diz `|yaw|<5°`; o valor em
  vigor é `3.0°` (`:187`, baixado em 19/06).** O comentário do bloco no launch
  repete o 5°. Corrigir os dois — foi o primeiro número que eu li errado ao
  escrever esta spec.
- O `door_crossing.py:584` tem um `_dbg_t` marcado **"DIAG arme (REMOVER)"**.
  Decidir: remover ou promover a log honesto.
- O nó usa `time.monotonic()` para todos os timeouts, mas o launch entrega
  `use_sim_time`. **Em sim que não roda em tempo real, os timeouts de 15 s / 40 s
  disparam em tempo de parede, não em tempo de simulação.** Precisa ser medido
  (qual o *real time factor* do harness) antes de confiar nos timeouts. Não
  proponho trocar o relógio agora — proponho **medir e registrar**.

---

## 6. Plano de teste (nesta ordem)

### 6.1 Vermelho primeiro (lógica pura, sem ROS, sem sim)

1. Teste 4.9 (pose da `noguard3`) — **tem que falhar antes** da porta ser marcada
   e passar depois. Como a máquina é pura, "antes" = sem a porta na lista.
2. Teste 4.10 (as 11 entradas boas, com o limite de re-estagiamento).
3. Teste de fiação do launch (5.3), com defeito injetado para provar sensibilidade.

### 6.2 Conta geométrica por fresta

Antes de marcar **qualquer** fresta além da A, repetir a conta de 4.4-(a)
(distância do ponto de preparação ao canto do bloco **vs** raio circunscrito
0,354 m). **Se a margem for negativa, a fresta não pode ser marcada** — o
alinhamento bateria no batente durante o próprio giro. (Suspeita a confirmar: é o
caso da fresta C de 0,60.)

### 6.3 Sim

4. **Uma** volta com a fresta A marcada, comando da §4.5 do diário (com
   `motion_guard:=false`, que é fácil de esquecer e volta a inserir as paradas de
   27 s sem avisar).
5. Volta com **largada deslocada** para reproduzir a entrada torta da `noguard3`
   (desvio +0,12 m na chegada da fresta). Sem isso, um resultado limpo pode ser
   sorte: **11 de 12 voltas já eram limpas**.
6. Só então, **n voltas** para comparar contra as 14 existentes.

### 6.4 O que é preciso medir em cada volta

| métrica | de onde | critério |
|---|---|---|
| COLISÃO / raspão na `A_fresta90` | `colisao.csv` | **zero** (A5) |
| folga mínima na fresta | `colisao.csv` | **> 0,045 m** (melhor que o pior histórico) |
| desvio lateral em x = 7,20 (boca) | `colisao.csv` | **< 0,08 m** |
| tempo porta-a-porta (entrar na zona → soltar) | `/door_zone` + CSV | registrar; é o custo (7) |
| re-estagiamentos e abortos | `/door_zone` | ≤ 1 re-estágio, 0 abort |
| tempo total da volta e goals cumpridos | `result.json` | **5/5 goals**; tempo só compara com voltas de mesmo nº de goals |

---

## 7. Custo conhecido: tempo

A travessia da máquina é deliberadamente lenta: `stage_speed 0,20` e
`cross_speed 0,22 m/s`, contra `v_max 0,90 m/s` da rota.

```
staging   0,6 m a 0,20 m/s  = 3,0 s
alinhar   giro no lugar     ≈ 1–3 s (depende do erro de yaw na chegada)
travessia 1,1 m a 0,22 m/s  = 5,0 s   (0,6 antes do centro + 0,5 de exit_margin)
                              ─────
                              ~9–11 s
```

Hoje a mesma passagem leva **~1,5 s**. **Custo estimado: +8 a +10 s por fresta
marcada**, sobre voltas de 221–245 s (**+4 % com só a fresta A**; ~+11 % se as
três forem marcadas).

**Isso é aceitável dentro da meta declarada** ("destemido + MUITO preciso",
velocidade fora do critério) — mas é uma **escolha do dono**, não minha, e está
aqui explícita para ser contestada. `cross_speed` é parâmetro de launch: dá para
subir depois **com medição**, não no chute.

---

## 8. Riscos, em ordem de gravidade

### 8.1 🔴 A zona arma com o robô a ~0,9 m/s, e o valor foi calibrado parado

`zone_radius = 1.1` foi escolhido (06-19) para armar 0,1 m antes de um **ponto
pré-porta onde o robô chegava parado e centrado** pelo `xy_goal_tolerance = 0.15`
do Nav2. **A arena não tem esse ponto.** O robô entra na zona **em movimento**, e
a máquina vai comandar `stage_speed = 0.20 m/s` — uma **desaceleração de 0,9 para
0,2 m/s** decidida em um tick.

Isso não foi medido nunca. Modos de falha possíveis: overshoot do ponto de
preparação (e re-estágio), tranco, ou o `path_follower` e o `door_crossing`
disputando o mux na fronteira da zona. **Mitigação proposta: medir primeiro** (a
volta 6.3-4 mostra), e só então decidir entre aumentar `zone_radius`, adicionar um
waypoint pré-fresta na rota, ou ambos. **Não** ajustar os dois no mesmo passo.

### 8.2 🟠 A ré de escape hoje passa pelo `collision_monitor`

Ver 4.6. A ré foi desenhada quando `door_vel` furava o reflexo; hoje não fura. Se
o robô ficar de nariz no batente, a ré pode ser atenuada exatamente quando é
necessária — e o histórico do projeto tem **stall de motor desarmando o BMS do
hoverboard**. **Mitigação: medir o vx efetivo em `auto_vel` durante um escape
provocado**, antes de confiar. Se estiver zerado, é uma decisão nova (e do dono),
não um ajuste silencioso.

### 8.3 🟠 No robô real, o eixo da porta vem do mapa, não do mundo

No sim os batentes são exatos. No real eles são pontos no **mapa de SLAM**, e a
AMCL na arena foi medida errando **24 cm** (mediana 9, p90 16, máx 27 — item 2c
dos abertos). Um erro de pose de 12 cm é maior que `align_lat = 0.08`: a máquina
alinharia com convicção no eixo **errado**. **Este desenho vale para o SIM.**
Levar ao robô real exige antes fechar o item 2c, e o aviso de arena-no-real
(pista controlada, gente fora, E-STOP na mão) continua valendo integralmente.

### 8.4 🟡 A máquina vai atuar em voltas que já eram limpas

11 das 12 travessias tinham |desvio| entre 0,006 e 0,081 — **coladas** na
tolerância de 0,08. Metade vai entrar direto em `crossing`; a outra metade vai
girar antes. É o que 4.10-item-3 mede. Se o re-estágio for frequente, a resposta
**não** é afrouxar `align_lat` no chute (isso desfaz o conserto): é medir e
decidir com dado.

### 8.5 🟡 Um nó a mais no caminho de dado que acabou de mudar

O pipeline da arena mudou **ontem** (guard desligado, mux religado direto no
`auto_vel_raw`), e guard-off **ainda não está validado** (n = 3, com 1 contato).
Este desenho adiciona uma fonte de prioridade 20 nesse mesmo caminho. As duas
mudanças vão ficar sobrepostas nas próximas voltas — **os resultados não vão
separar o efeito de uma e de outra**, e o registro tem que dizer isso.

---

## 9. Fallback, se o desenho não fechar até 04/09

Ordem de recuo, do menos para o mais drástico:

1. **`door_crossing` só na fresta A** (já é o escopo proposto).
2. **Desarmar a porta** (`door_crossing:=false`) e correr como hoje — o
   comportamento atual, com o risco medido de contato.
3. **Fechar a fresta A para o planejador** e ir pelo contorno (`y > 4,20`), como
   `aprox2` e `latchN1` já fizeram na prática. Custa tempo, zera o risco de
   contato **nessa** passagem, e não depende de nenhum código novo.

A opção 3 é a rede de segurança da prova e deve ser **testada uma vez** de
qualquer jeito, para que exista pronta se a 1 falhar em 04/09.

---

## 10. Ordem de implementação proposta

1. `maps/arena_galpao.doors.json` gerado por `tools/gera_arena_galpao.py` (só a
   fresta A) + teste de que os batentes casam com a geometria do mundo.
2. Parâmetro `doors_file` no `door_crossing` (5.2-a) + teste.
3. Testes **vermelhos** 4.9 e 4.10 (a máquina ainda desligada no launch).
4. Fiação do launch com `door_crossing:=true` ligado pelo `--arena` (5.3) +
   teste de fiação sensível.
5. Higiene 5.4 (comentários obsoletos do collision, `_dbg_t`, medir o RTF).
6. Voltas do sim na ordem 6.3.
7. Só então decidir sobre as frestas B e D, com a conta 6.2 refeita.

**Nada disso começa antes desta spec ser revisada.**

---

## 11. O que este desenho NÃO resolve

- **Não** valida guard-off (n = 3 continua n = 3).
- **Não** resolve o `"start/goal is an obstacle"` (item 8 dos abertos), que nasce
  do `robot_radius 0.32` contra um vão de 0,90 e pode continuar custando goals.
- **Não** melhora o erro de 24 cm da AMCL (item 2c) — e depende dele para o real.
- **Não** cobre A1 (missão completa), A2 (chegar a 20 cm do cone) nem A3 (LED).
- **Não** foi ao robô real, e não deve ir sem 8.3.

---

## Fontes

- Medição e retratação: `DIARIO_ARENA.md` **§2B.10** (tabela das 12 travessias),
  §2B.9 (as 3 voltas sem guard), §5 BO 69.
- Estado do guard-off: `HANDOFF_ARENA_GUARD.md`.
- Máquina: `ros2_packages/robot_nav/robot_nav/door_crossing.py`,
  `test/test_door_crossing.py`,
  `docs/superpowers/specs/2026-06-12-zonas-de-porta-design.md`.
- Arbitragem: `config/twist_mux_auto.yaml`; fiação: `launch/nav2.launch.py`.
- Geometria da arena: `tools/gera_arena_galpao.py`.
- Trajetórias: `log/sim_ab/*/colisao.csv`; oráculo: `log/sim_ab/colisao.py`.
