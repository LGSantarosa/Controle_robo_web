# Standoff 1,0 → 1,4 m — arena do galpão, 2026-09-01

Evidência dos números de `DIARIO_ARENA.md` **§2G.8 e §2G.9**. Todas as voltas
foram rodadas pelo **caminho nominal da prova** — `./launch.sh --sim --nav2
--arena`, sem mais nenhum argumento — com o oráculo de colisão e o `probe` da
rota por cima. A única diferença entre a `nominal1` e as `std14_*` é o
`STANDOFF` em `tools/gera_arena_galpao.py` (1,0 → 1,4 m) e a rota regerada.

**O que isto mede:** margem do **point-turn** que o seguidor faz ao concluir cada
goal. **O que NÃO mede:** o A2 (chegar a 20 cm do cone) — a rota morre no
standoff, o `cone_detector` nem sobe no perfil nav2, e o gargalo do A2 continua
sendo o `PolygonFront` a ~0,67 m do centro do cone, de 1,0 ou de 1,4 m.

| volta | standoff | goals | COLISÃO | raspão | folga no `cone_2` | tempo |
|---|---|---|---|---|---|---|
| `nominal1` | 1,0 m | 5/5 | 0 | **18** | **0,0000** | 236,6 s |
| `std14_1` | 1,4 m | 5/5 | 0 | 0 | 0,3628 | 295,3 s |
| `std14_2` | 1,4 m | 5/5 | 0 | 0 | 0,6255 | 232,9 s |
| `std14_3` | 1,4 m | 5/5 | 0 | 0 | 0,5657 | 244,0 s |

| arquivo | o que sustenta |
|---|---|
| `result_nominal1.json` `result_std14_1.json` `result_std14_2.json` `result_std14_3.json` | as 4 voltas goal a goal: tempo, distância, `parado`, `min_scan`. 5/5 nas quatro |
| `probe_nominal1.txt` `probe_std14_1.txt` `probe_std14_2.txt` `probe_std14_3.txt` | a volta como o `ab_probe` a viu (era `probe.log`; `*.log` é `gitignore`d — BO #31) |
| `colisao_por_objeto_nominal1.csv` | a volta que REPROVOU: 18 raspões no `cone_2`, folga 0,0000 |
| `colisao_por_objeto.csv` | as 3 voltas com 1,4 m: **zero evento**, e a folga por objeto que mostra o `cone_2` saindo de crítico (0,36–0,63 m) |
| `geometria_do_raspao.txt` | a conta que fecha os dois casos: varredura do canto (0,354) + raio do cone (0,17) contra a distância real de giro |
| `erro_pose_amcl_x_gazebo.txt` | erro de pose das 4 voltas. É onde se lê que a `std14_1` teve erro **maior** (49,1 cm) que a volta que bateu (45,1) e mesmo assim não encostou |
| `unstuck_disparos.csv` | os 3 `reason=near` da `std14_1` (as outras duas: zero) — item 2e/2f, não o standoff |
| `dist_final_por_goal.csv` | última amostra de cada goal nas 3 voltas |

## A conta que decide

```
margem do point-turn = STANDOFF − 0,354 (canto do robô) − 0,17 (raio do cone)
    1,0 m → 0,4764 m      1,4 m → 0,8764 m
```

O erro de pose do AMCL nesta arena vai a **0,49 m**. Com 1,0 a margem é menor que
o erro máximo — o contato depende de qual erro cai naquele instante. Com 1,4 o
erro cabe, e a `std14_1` prova isso com o maior erro das quatro voltas.

## O que esta pasta NÃO prova

- **n = 3** com 1,4 m, no sim, mundo determinístico. Não é taxa de contato.
- **Não corrige o item 1**: o `path_follower` continua girando no lugar sem olhar
  o anel. É mitigação por afastamento — vale enquanto a margem for maior que o
  erro de pose.
- **Não vale na fresta**: lá o orçamento lateral é ±0,20 m e o mesmo erro de
  0,49 m não cabe. Ver §2G.10.
- **3 voltas custaram 5 tentativas**, e as 2 perdas foram bug do harness (BOs
  79-81), não do robô. As tentativas perdidas não estão arquivadas aqui.
