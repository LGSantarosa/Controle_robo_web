# Volta com o LATCH da chegada — arena do galpão, 2026-08-31

Evidência dos números citados em `DIARIO_ARENA.md` §2B.4. Par direto do
`docs/baselines/2026-08-28-arena-baseline1/` (mesma arena, mesma rota, mesmo
comando da §4.5) — a **única** diferença é o commit `c85a8d8` no `path_follower`.

Aqui ficam **só os arquivos pequenos**; os CSVs completos ficam em
`log/sim_ab/arena_latch1/`, que é `gitignore`d — some num checkout limpo.

| arquivo | o que sustenta |
|---|---|
| `result.json` | 5/5 goals, 222,8 s, tempo por goal (goal 4 = 61 s, 10,8 s parado) |
| `probe.log` | a volta como o `ab_probe` a viu, goal a goal |
| `colisao_resumo.txt` | **zero evento** e a menor folga por objeto (mín. 7,4 cm na fresta A) |
| `transicoes_goal_turn.csv` | **as DUAS voltas lado a lado**: toda troca de/para `goal_turn`. É a prova do 7 → 0 |
| `goal4_parou_fora_da_tolerancia.csv` | a janela 155–176 s com AMCL **e verdade-terreno do Gazebo** na mesma linha |
| `unstuck_disparos.csv` | os 2 disparos (`reason=timeout`, `stuck_s≈5,1`, `nav_wants=1`) |

## Como ler `transicoes_goal_turn.csv` (o número que mais importa)

A samba é a linha `goal_turn -> turning` **com `dist_goal` pequeno** (mesmo goal).
`goal_turn -> turning` com `dist_goal` de metros é **goal novo** — legítimo, é o
latch soltando. Contagem: baseline **7**, latch **0**.

## Como ler `goal4_parou_fora_da_tolerancia.csv`

`yaw_gz`/`x_gz`/`y_gz` são verdade-terreno do Gazebo, alinhados ao CSV do seguidor
por um offset de **+5,8 s** achado minimizando o erro de yaw na volta inteira
(erro médio residual **1,17°**). O alinhamento é do analista, não do harness —
quem quiser refazer, o método está aqui.

O que a janela mostra: `arrived` em 160,6 com o robô **parado de verdade** (x,y do
Gazebo constantes), depois `yaw_gz` 182,5° → 199,2° entre 165,4 e 165,9 **com x,y
parados** — o robô girou de fato, não foi salto de pose. Quem girou foi o
`unstuck` (ver `unstuck_disparos.csv`).

⚠️ **Uma volta contra uma volta (n=1 de cada lado). Não é taxa de sucesso e não
fecha A4.** E zero contato aqui **não prova** que o latch foi a causa: o §2.9
lista outros dois suspeitos no `cone_3` que continuam de pé.

⚠️ **Volta até os STANDOFFS (1 m antes de cada cone). NÃO prova A1, A2 nem A3.**

Não versionado aqui: `nav2.log` (2 recoveries — 1 `No valid trajectories`, 1
`Failed to make progress`).

Reproduzir: `DIARIO_ARENA.md` §4.5, tag `arena_latch1`. **Rode `colcon build`
antes** — ver o aviso do hardlink na §2B.4.
