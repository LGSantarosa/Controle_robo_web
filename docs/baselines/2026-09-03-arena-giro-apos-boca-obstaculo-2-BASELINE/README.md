# BASELINE 2026-09-03 14:09 — giro DEPOIS da boca do obstáculo 2

Escolhido pelo dono como ponto de comparação do Fix 1:

> "faz o fix 1 ai comparamos com o baseline que será essa corrida que terminou
> agora, pois ela teve o giro depois da boca do obstaculo 2, ai depois de arrumar
> isso não deve voltar a contecer"

Estado do código nesta corrida: `exit_margin = 0,60` (commit `aed3306`), portas 1-4
ainda marcadas, `path_follower` SEM o reset de estado.

## O que tem que sumir depois do Fix 1

O robô atravessa o obstáculo 2 e, ao sair, gira sozinho. Análise na §2H.28 do
`DIARIO_ARENA.md`.

| t (epoch) | o que |
|---|---|
| 1788455313,9 | goal novo (11,43/3,38) -> (11,60/6,90); `path_follower` entra em `turning` |
| 313,9 → 326,7 | **13 s travado em `turning`** com o `door_crossing` dirigindo; o `_aim_filt` integra bearings de DOIS planos (n≈130 OESTE × n≈40-68 NORTE) |
| 320,1 → 326,7 | `door_crossing: rotating -> crossing -> idle` (atravessou) |
| 327,5 → 330,0 | solto: gira yaw 89° → 160°, `wz` +4,3 — **caçando a mira filtrada** (`herr` +67/+96/+122 com o plano já apontando pro norte) |
| 330,5 → 331,0 | EMA converge, `herr` **troca de sinal**, gira de volta (`wz` −2,98 → −2,4) |

## Critério de comparação

Na corrida com o Fix 1, entre a soltura do `door_crossing` (`crossing -> idle`) e
os ~5 s seguintes, o `follow_debug.csv` **não** pode mostrar `wz` trocando de sinal
nem `herr` acima de ~30°. Se mostrar, a causa é o plano do momento (Fix 2), não o
estado velho.
