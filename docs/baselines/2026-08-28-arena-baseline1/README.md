# Baseline Nav2 até os standoffs — arena do galpão, 2026-08-28

Evidência dos números citados em `DIARIO_ARENA.md` §2.8. Aqui ficam **só os
arquivos pequenos**; os CSVs completos (`follow_debug` 393 KB, `colisao` 192 KB,
`freeze_capture` 1,5 MB) ficam em `log/sim_ab/arena_baseline1/`, que é
`gitignore`d — some num checkout limpo.

| arquivo | o que sustenta |
|---|---|
| `result.json` | 5/5 goals, 236,4 s, tempo por goal |
| `colisao_resumo.txt` | 16 colidíveis (12 caixas + 4 cones) e as 2 colisões no `cone_3`. **Era `colisao.log` e não entrava no git** — `*.log` no `.gitignore` |
| `colisao_eventos.csv` | só as amostras com evento (COLISAO/raspao), com pose e folga |
| `samba_goal3_follow_debug.csv` | a janela do goal 3, onde a samba e os contatos acontecem |

⚠️ **Baseline até os STANDOFFS (1 m antes de cada cone). NÃO prova A1, A2 nem A3.**

Não versionado aqui, e importante: o `nav2.log` (5 recoveries do Nav2, §2.8).

Reproduzir: `DIARIO_ARENA.md` §4.5.
