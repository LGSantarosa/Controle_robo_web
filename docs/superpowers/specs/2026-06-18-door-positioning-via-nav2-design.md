# Door crossing: posicionar via nav2, cruzar via door

**Data:** 2026-06-18
**Branch:** feat/door-para-pra-pessoa
**Status:** desenho aprovado, aguardando spec review

## Problema / causa raiz

O `door_crossing` é ótimo em **cruzar reto** e péssimo em **se posicionar pra
cruzar**. Observação de campo que cravou a raiz (06-18):

- Quando o nav2 entrega o robô **centrado no eixo, perpendicular, precisando só
  do giro de 90°** → atravessa perfeito.
- Quando o nav2 entrega **torto / fora do eixo** → o `staging` tenta dirigir até
  um ponto de preparação **na diagonal, colado na porta** → raspa o batente →
  dispara ré → reposiciona reativo → ping-pong → "alucina" no meio da porta.

Todos os patches do dia (release antecipado, standoff do giro, fit 0.13) mexeram
no que acontece **depois** de já estar mal-posicionado. Por isso nenhum resolveu
a raiz: **o erro principal é a APROXIMAÇÃO** — o door tenta REPOSICIONAR o robô
na unha, pertinho da porta, em vez de chegar limpo num ponto bom ANTES.

## Objetivo

1. **Posição inicial boa SEMPRE**, ou um jeito limpo de chegar nela — sem bater,
   sem ré errada, sem ping-pong.
2. **Reduzir a complexidade.** O door ficou complexo demais pra uma tarefa que é
   pequena. O redesign tem que deixar a coisa mais simples e linear, não só
   "mais um caso tratado".

## Desenho — "posicionar via nav2, cruzar via door"

Cada componente faz o que faz bem: **nav2 posiciona** (chegar num ponto aberto e
longe, desviando de obstáculo — é o forte dele); **door alinha+cruza de um ponto
seguro** (giro longe da porta + travessia reta — é o forte dele, provado em
campo: girou limpo a 82 cm e atravessou).

### Fluxo (4 estados: `idle`, `positioning`, `rotating`, `crossing`)

1. **`idle` → arma** quando o `/plan` cruza a porta D com goal G ativo. O door:
   - **captura G** (assina `/goal_pose`),
   - **calcula W** = ponto **no eixo da porta, recuado `wp_standoff` (~1,0 m),
     centrado**, no lado onde o robô está; orientação de W = heading de
     travessia (de frente pra porta),
   - **manda W** como goal `navigate_to_pose` (cliente de action),
   - entra em **`positioning`**.

2. **`positioning`:** o door fica **de mãos quietas** — NÃO comanda velocidade,
   quem dirige é o nav2. Espera o **resultado do goal W**:
   - **W SUCCEEDED** (nav2 entregou o robô em W dentro da tolerância dele) →
     door **assume**: vai pro `rotating`.
   - **W ABORTED / `wp_timeout` estourou** → **re-manda W** (até `wp_retries`,
     ~2×). Persistiu → **desiste e avisa** (log WARN + `/door_zone='failed'`),
     volta pra `idle`. NÃO ressuscita aproximação reativa.

3. **`rotating`:** alinha NO LUGAR pro heading de travessia (point-turn limpo, o
   mesmo de hoje — confiável porque está **longe da porta**). Alinhou e parou →
   `crossing`. (Se nav2 já entregou alinhado, passa direto.)

4. **`crossing`:** anda reto pelo eixo, com micro-correção lateral/yaw, da posição
   de W **através** da porta. Mantém os fixes de hoje: zona de parada p/ pessoa
   (caminho B), `fit_lat` (0.13 = só centrado), e **solta assim que passa dos
   batentes** (`exit_margin` 0.30). Passou → **re-manda G** pro nav2 continuar →
   `idle` + `success_cooldown`. O `/plan` já não cruza a porta → não re-arma.

### Cálculo de W

Dado os batentes a, b (frame do mapa): centro = ponto médio; eixo normal n
(perpendicular à parede). `side` = lado onde o robô está (sinal de
`(pos−centro)·n`).

- **W.posição** = `centro − n·side·wp_standoff` (no eixo, recuado).
- **W.orientação** = `atan2(side·n.y, side·n.x)` (heading de travessia).

## O que SAI — redução de complexidade

Deletado (era a aproximação reativa colada na porta, fonte de todos os bugs do
dia):

- Estado **`staging`** (dirige-na-diagonal pro ponto de preparação).
- Estado **`reversing`** + toda a ré de escape de aproximação.
- **`_maybe_escape`** e parâmetros: `escape_front_gap`, `escape_substuck_time`,
  `escape_reverse_dist`, `escape_rear_margin`, `escape_rear_min`,
  `escape_max_count`, contagem `_escape_count`.
- **Substuck / âncora de progresso**: `_align_anchor`, `_align_t0`,
  `align_progress_radius`.
- O **standoff-reverse do giro** (adicionado hoje) — sem `staging`, não existe
  mais "girar colado", então o standoff e a ré-pra-ganhar-distância somem.
- `gap_min` como abort de aproximação; o `front_gap`/`rear_gap` só sobrevivem se
  ainda usados pela zona de parada do `crossing` (caminho B usa `gap` mascarado,
  não o `front_gap` do escape) — revisar e podar na implementação.

Mantido (o que funciona): `idle`, `rotating` (giro limpo longe), `crossing`
(reto + caminho B + release), geometria da porta, `fit_lat`,
`door_progress_lateral`.

Saldo: **5 estados → 4**, e some uma máquina inteira de recuperação reativa
(ré/escape/anti-stall/substuck/ping-pong). O fluxo vira **linear**: arma → nav2
leva → alinha → cruza → continua. Troca-se "recuperação reativa complexa" por
"mandar um goal e esperar", que é sequencial e testável.

## Parâmetros novos (live-tunáveis)

| Param | Default | O que é |
|---|---|---|
| `wp_standoff` | 1,0 m | distância de W antes do centro da porta (o "longe") |
| `wp_retries` | 2 | re-tentativas de mandar W antes de desistir |
| `wp_timeout` | 30 s | tempo que o nav2 tem pra chegar em W antes de re-tentar |

Reaproveitados: `rot_speed`/`rot_left_boost`/`rot_brake_*` (giro), `cross_speed`/
`cross_k_lat`/`cross_k_yaw`/`cross_wz_max` (travessia), `fit_margin` (0.13),
`exit_margin` (0.30), `stop_*` (caminho B), `success_cooldown`, `zone_radius`,
`approach_bearing`.

## Casos de borda

- **Novo destino no meio da manobra:** o `/goal_pose` muda. O door detecta (G
  mudou) → cancela a sequência atual (cancela goal W se em `positioning`; se já
  está cruzando, termina a travessia e re-manda o **novo** G), reavalia.
- **Re-arme pós-travessia:** depois de soltar e re-mandar G, o `/plan` defasado
  (~1 Hz) ainda mostra por ~1 s a rota cruzando a porta. O `success_cooldown`
  (2 s) segura o re-arme até o plano atualizar (igual hoje).
- **W do lado errado / robô já passou de W:** se ao armar o robô já está mais
  perto da porta que W (dentro de `wp_standoff`), manda W mesmo assim — o nav2
  recua/contorna pra chegar nele (é navegação normal em espaço aberto). Se isso
  causar vai-e-vem, é sinal de afrouxar a detecção de arme (armar mais cedo, mais
  longe) — tunável.
- **nav2 indisponível / sem plano:** sem `navigate_to_pose` server ou sem
  `/goal_pose`, o door não arma (fica em `idle`), não inventa manobra.
- **Goal W preempta o goal G da web:** ao mandar W, o nav2 (server único)
  abandona G. Por isso o door **captura G antes** e **re-manda fielmente** depois
  de cruzar. A métrica de nav da web verá o goal trocar (cosmético; documentar).

## Testes

- **Lógica pura (unit, como os 60 de hoje):** arma → decide mandar W (com W
  calculado certo: eixo, standoff, side, orientação); `positioning` →
  takeover só em W-SUCCEEDED; W-ABORTED → conta retry → desiste no limite;
  `crossing` → re-manda G ao soltar; novo G no meio → cancela/reavalia.
  - O envio/cancelamento de goal é injetável (interface fina: `send_goal(pose)`,
    `cancel()`, callback de resultado) pra testar a máquina sem ROS.
- **I/O (cliente de action, captura de `/goal_pose`):** smoke na bancada + campo.

## Riscos

- **Door vira cliente de action do nav2** (manda/cancela/re-manda goal). É
  responsabilidade nova; isolar atrás de uma interface fina pra não poluir a
  máquina de estados e manter testável.
- **Alinhamento de W:** apostamos que o nav2 entrega o robô na POSIÇÃO de W
  (centrado, longe); o alinhamento fino fica com o `rotating` do door (provado
  longe da porta). Não dependemos do point-turn do nav2.
- **Tolerância de goal do nav2 vs centragem:** o nav2 entrega W dentro da
  tolerância dele (~0,25 m). Como W é recuado ~1,0 m, o `crossing` tem ~1 m de
  espaço ABERTO pra convergir o lateral antes dos batentes; o `fit_lat` no
  batente é a trava de segurança (não centrou → aborta → re-tenta W). Sem
  re-centragem colada na porta.
- **Mudança grande** (memória do projeto: reescrita já mordeu antes). Mitigação:
  é redesign FOCADO na aproximação (não toca `rotating`/`crossing` que funcionam),
  vai com testes + smoke + backup do último bom (`89d08c7`) + validar em campo
  antes de main.
