# Contorno da fresta A — arena do galpão, 2026-09-01

Evidência dos números citados em `DIARIO_ARENA.md` **§2G**. Par direto do
`docs/baselines/2026-08-31-arena-sem-guard/` (mesma arena, mesmo mundo, mesma
rota, mesmo comando da §4.5, guard desligado nos dois). A **única** diferença é
o mapa que o Nav2 carrega:

| | mapa do planejador | mundo (SDF) |
|---|---|---|
| `noguard1..3` | `maps/arena_galpao.yaml` (fresta A **aberta**) | idêntico |
| `contornoA1..4` | `maps/arena_galpao_semA.yaml` (fresta A **tampada**) | idêntico |

O tampão vem de `python3 tools/gera_arena_galpao.py --mapa maps/ --fecha-fresta A`
(`2e79503`) e quebra a invariante "mapa = mundo" **de propósito e só no `.pgm`**:
o vão de 0,90 m continua fisicamente aberto no Gazebo, o robô é que deixa de ser
mandado por ele. ⚠️ O `maps/arena_galpao_semA.*` **não está versionado** (o
`.gitignore` ignora `maps/`) — regenerar com o comando acima.

Aqui ficam **só os arquivos pequenos**; os CSVs completos ficam em
`log/sim_ab/contornoA{1,2,3,4}/`, que é `gitignore`d — some num checkout limpo.

| arquivo | o que sustenta |
|---|---|
| `result_contornoA1.json` `result_contornoA2.json` `result_contornoA3.json` `result_contornoA4.json` | as 4 voltas goal a goal: tempo, distância, `parado`, `min_scan`. 5/5 goals em 3 delas, 4/5 na `contornoA3` |
| `probe_contornoA1.txt` `probe_contornoA2.txt` `probe_contornoA3.txt` `probe_contornoA4.txt` | a volta como o `ab_probe` a viu (era `probe.log`; `*.log` é `gitignore`d — BO #31) |
| `colisao_por_objeto.csv` | **zero COLISÃO e zero raspão nas 4 voltas**, e a menor folga por objeto. É onde se lê que a folga contra `A_fresta90_2` virou 0,224–0,270 m |
| `dist_final_por_goal.csv` | última amostra de cada goal: **≤ 0,07 m em 19/19 goals concluídos** |
| `unstuck_disparos.csv` | 1 disparo real em 4 voltas (`contornoA4`, `reason=timeout`, `nav_wants=1`) — as linhas `monitoring` do início são boot, não disparo |
| `pose_congelada_contornoA4.csv` | as duas janelas de pose parada da `contornoA4` (15,0 s e 10,8 s) com o vizinho mais próximo a 1,22 m — item 2e/2f, **não** o tampão |
| `contornoA3_goal1_abortado.txt` | as 3 linhas do `nav2.log` que provam a causa do goal perdido: `server_timeout` no *acknowledge* do `compute_path_to_pose`, **não** `start/goal is an obstacle` (item 2l) |
| `mapa_tampado_probes.txt` | o `mapa_passagens.py` no mapa tampado: `A_fresta90 → ✗ FECHADO`, as outras 3 frestas intactas, e as 5 pernas ainda `✓ LIGADOS` até raio 0,354 (o contorno existe) |
| `erro_pose_amcl_x_gazebo.txt` | erro de pose das 4 voltas tampadas **e** das 3 abertas, lado a lado. Produzido por `tools/sim_ab/erro_pose.py` (fora desta pasta, porque é ferramenta e não evidência) — o alinhamento é do analista, ver abaixo |


## Como ler `erro_pose_amcl_x_gazebo.txt`

`follow_debug.csv` traz a pose do **AMCL** em relógio de parede; `colisao.csv`
traz a **verdade-terreno do Gazebo** em tempo relativo. Alinhar as duas exige um
offset, e ele é **estimado**: o script varre 0–25 s em passos de 0,05 s e fica
com o offset que **minimiza o erro mediano de yaw** (residual 0,8–1,1° nas 7
voltas). Só depois calcula o erro de posição. Refazer:

```bash
python3 tools/sim_ab/erro_pose.py \
    contornoA1 contornoA2 contornoA3 contornoA4 noguard1 noguard2 noguard3
```

(precisa de `log/sim_ab/<tag>/` presente — as voltas não vão para o git).

**O que o arquivo mostra:** mediana 7,4–8,7 cm com o mapa tampado contra
7,0–8,1 cm com o mapa aberto; p90 13,6–15,5 contra 14,6–17,4. **As duas famílias
se sobrepõem** — o tampão não trouxe degrau de erro de pose. Não é o mesmo que
dizer que melhorou.

## O que esta pasta NÃO prova

- **Não** prova A1/A2/A3: a rota para nos **standoffs**, 1 m antes de cada cone.
- **n = 4**, no sim, mundo determinístico. "Zero contato em 4/4" não é taxa.
- **Não** valida nada no robô real, e o mapa tampado ainda não é o mapa oficial
  da prova — enquanto não for versionado e apontado pelo launch, não existe na Pi.
