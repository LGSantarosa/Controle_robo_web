# Fluidez dos giros do path_follower — rot_min 2.4 + alvo congelado no giro

**Data:** 2026-07-02 · **Aprovado pelo dono** (conversa 07-02)

## Problema (medido na run real de 07-02, `follow_debug.csv`, 260s)

O robô "para toda hora" (sensação de movimento não fluido). O dado mostrou:

- **56% do tempo em `turning`** (145s girando vs 94s andando), 44 episódios.
- Mediana do trecho reto = **1.4s** (anda ~40cm, para, corrige, repete).
- Giros pequenos são um **rastejo**: `rot_min=2.0` rad/s comandado ≈ **10°/s reais**
  (zona-morta 1.7 + resposta 0.6·(cmd−1.7) do spin_calib de 06). Corrigir 12° = 1-2s
  "parado" aos olhos.
- **6 giros de 8-19s**: durante o giro chegam 6-11 replans (~1Hz), o carrot pula e o
  erro re-abre — girou 61° pra um erro que nasceu de 14° (caça alvo móvel).
- Zero flip de sinal no wz → a histerese atual funciona; NÃO é oscilação.

## Mudanças (2 passos, medidos SEPARADOS no sim)

### Passo 1 — `rot_min` 2.0 → 2.4
Default do parâmetro no `path_follower`. Correção pequena vira ~25°/s real.
Risco: tranco no início do giro → se aparecer, recuar pra 2.2/2.3.

### Passo 2 — congelar o alvo do giro
Ao ENTRAR em `turning`, salvar o bearing-alvo (o do carrot daquele instante). Girar
até fechar (mesma histerese, `turn_exit` 3°) usando o alvo salvo. Ao voltar pra
`driving`, limpar o alvo e re-olhar o plano normalmente. O replan continua livre —
o follower só não troca de alvo NO MEIO de um giro; mudança grande é pega no ciclo
seguinte (~50ms após alinhar). `goal_turn` não muda (o yaw do goal já é fixo).
Implementação na lógica pura (`DecisiveFollower`), testável sem ROS.

## Validação (sim, rota padrão `sala.yaml`, antes de qualquer robô)

3 métricas do `follow_debug.csv`, comparando baseline → P1 → P1+P2:

| métrica | hoje (real) | alvo |
|---|---|---|
| % tempo em turning | 56% | cair |
| mediana do trecho reto | 1.4s | subir |
| giros >5s | 6 | ~0 |

Regressão em qualquer métrica, overshoot ou oscilação nova (wz flips) → reverte o passo.
Testes unitários novos pro alvo congelado; os 100+ existentes seguem passando.

## Fora do escopo
unstuck, collision_monitor, taxa de replan do nav2, `lookahead`/`turn_enter`
(subir `turn_enter` 12→18° é o passo C, só se P1+P2 não bastarem).
