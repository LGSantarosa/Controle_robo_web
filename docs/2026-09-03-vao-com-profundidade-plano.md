# Plano — vão com PROFUNDIDADE declarável + pivô limitado dentro

Origem: BO da §2H.23 do `DIARIO_ARENA.md` (giro de 180° dentro da fresta 2, dois
cones arrastados). Decisões do dono nas §2H.24 e §2H.26.

## As 3 decisões do dono (fixas, não re-discutir)

1. **Bater é o pior caso** — mata a volta inteira. Não bater vale mais que trajeto.
2. **Dentro do vão: prepara FORA, um último pivô na boca, depois RETO.** O robô não
   esterça andando (`arc_calib` ≤19%), então "corrigir andando" não existe.
3. **O tamanho da área tem que ser declarável por porta** — na competição varia de
   porta fina a túnel de 2 m. Hoje é constante global e a porta é uma LINHA.
4. Dentro do vão, **pivô é permitido até `θ_max(largura)`** — o que matou foi 180°,
   não a existência de pivô. Um pivô de 10° cabe num vão de 0,70 m.

## O número que fecha o BO

A porta da arena é feita de **2 cones de R=0,17 m** (`tools/gera_arena_galpao.py:28`),
logo ela **já tem 0,34 m de profundidade física** hoje — e o modelo a trata como
espessura zero.

No instante em que o `door_crossing` soltou o robô (`exit_margin = 0,50 m`):

| | |
|---|---|
| traseira do robô parado | 0,250 m do plano dos batentes |
| borda de trás do cone | 0,170 m |
| **folga real** | **0,080 m** |
| `s` necessário pra caber um pivô de 45° | 0,524 m (envelope 0,354 + cone 0,17) |
| **faltaram** | **2,4 cm** |
| pivô máximo que cabia ali | **24,0°** |
| pivô que o `path_follower` executou | **180°** |

## Geometria de referência (medida, não estimada)

- robô **0,50 × 0,50** roda-a-roda → meio-corpo 0,25; envelope girando θ:
  `0,25·(cos θ + sin θ)`; meia-diagonal **0,354 m** (pivô de 45° exige vão ≥ 0,81 m)
- larguras das 4 portas da arena: **0,90 / 0,70 / 0,60 / 0,80 m**
- erro de yaw na entrada, medido (§2H.4, 13 voltas): mediana **10,7°**, pior **15,8°**

`θ_max` (pivô que cabe, centrado, margem 5 cm): 0,60 m → **0°** · 0,70 → **13,1°**
· 0,80 → 36,9° · ≥0,90 → 45°

Orçamento de yaw na entrada com `wz=0` dentro: 0,70 m → 14,0° (fina) / 5,7° (d=0,5)
/ **1,4°** (d=2,0). **A porta de 0,60 m tem folga útil ZERO** — é a porta 3 da arena.

## Passos (ordem de execução; cada um testável isolado)

| # | passo | muda comportamento? |
|---|---|---|
| 1 | **Geometria pura + testes**: `depth` no `DoorGeom`; `pivot_max_yaw(largura, margem)`; `entry_yaw_budget(largura, depth)`; `exit_s_min(depth, ...)`; `will_clear` projetando até a boca de SAÍDA (`s=+depth/2`) em vez de `s=0` | **não** (depth=0 → idêntico) |
| 2 | **Schema `doors.json`**: campo `depth` opcional por porta (ausente = 0), `valida_doors` valida, gerador escreve `depth=0.34` nas portas de cone. É o "poder" que o dono pediu | não, até alguém pôr um valor |
| 3 | **Soltura derivada**: `exit_margin` deixa de ser 0,5 fixo e vira `depth/2 + meia-diagonal + margem` → **só solta quando um pivô cabe**. Mata o BO da §2H.23 | **SIM** — é o conserto |
| 4 | **Gate no `path_follower`**: assina `/door_zone`, recusa pivô > `θ_max` enquanto estiver no corredor. Defesa em profundidade (independe do passo 3) | **SIM** |
| 5 | **`crossing` sem arco**: zera `cross_k_lat`/`cross_k_yaw` (hoje comandam até 0,8 rad/s andando, que o robô não entrega) | **SIM** |
| 6 | **Pivô limitado dentro**: sub-estado para → pivota ≤ `θ_max` → anda, para o túnel fundo em que o último pivô da boca não basta | **SIM** |
| 7 | **Uma volta no sim** e medir. Nada disso é evidência até rodar | — |

`zone_radius` não precisa de campo novo: vira `zone_radius + depth/2`, então o dono
declara **um número por porta** e o resto acompanha.

## Riscos anotados

- Passo 3 empurra o ponto de soltura pra fora em toda porta; o plano do Nav2 continua
  podendo nascer torto (item 8 do §6) — o passo 3 só garante que isso não custe cones.
- Passo 6 nunca foi testado: pivô dentro do vão é comportamento novo. Sim primeiro.
- Passo 5 remove um ganho que está no código desde 06-19. A justificativa é medição
  (`arc_calib`), não gosto — mas ele nunca foi isolado num A/B.
