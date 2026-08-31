# HANDOFF — `motion_guard` desligado na arena (2026-08-31)

> Estado de quem pega o bastão. **Tudo medido no SIM, nada foi ao robô real.**
> Branch: **`arena-galpao`**. Último commit: `a052c18`. **Working tree limpa.**
> Fonte completa: `DIARIO_ARENA.md` §2B.7, §2B.8, §2B.9 (este arquivo é o resumo
> navegável, o diário é a fonte).

---

## 1. O que aconteceu, em uma frase

Eu passei três sessões atribuindo as paradas longas do robô na arena ao *churn*
da mira, corrigi o churn, ele caiu 3–5× — e **o tempo parado não caiu**. A causa
dos piores casos era outro nó, que eu nunca tinha olhado: o **`motion_guard`**, o
vigia de PESSOA, ligado numa prova que **não tem pessoa nenhuma — tem cone**.

---

## 2. O que está PROVADO (e como)

3 episódios de bloqueio em 11 voltas. O guard fica `blocked` e **zera o comando**
entre `auto_vel_pre` e `auto_vel_raw`:

| volta | duração | comandos in→out | vizinho mais próximo | folga |
|---|---|---|---|---|
| `hist3` | **26,9 s** | 505 → **1** | `cone_3` | 0,312 m |
| `aprox2` | **26,2 s** | 502 → **1** | `cone_3` | 0,115 m |
| `aprox2` | **25,7 s** | 505 → **1** | `cone_4` | 0,405 m |

Três argumentos independentes:

1. **A duração é a soma dos tetos do próprio vigia:** `hold_still_max` 20 s +
   `clear_time` 5 s + `settle` (≤4 s) = 25–29 s. Os três caem em 25,7–26,9 s. É a
   vigília **rodando até o teto** em cima de coisa parada, não flicker.
2. **Pose do ground truth do Gazebo IDÊNTICA** o episódio inteiro (521 amostras
   iguais na `hist3`), e o único objeto ao alcance era um **cone**, 3/3.
   `worlds/arena_galpao.sdf` tem 23 modelos e **nenhum `<actor>`**.
3. **`bin/pause_budget.py`**, ferramenta que existe desde 07-03 e não sabe nada
   de cone nem de ground truth, atribui sozinha: `guard_hold` = 26,8 s = **52,1%
   de todo o tempo parado** da `hist3`.

**O que NÃO está provado:** o centróide da vigília (`_watch`) não é publicado por
tópico nenhum — não dá pra exibir a coordenada que o guard estava vigiando. O que
dá pra exibir é a duração, o comando zerado, a pose congelada e o vizinho.

Evidência versionada: `docs/baselines/2026-08-31-arena-histerese/`
(`guard_bloqueio_11voltas.csv` é o arquivo-chave).

---

## 3. O que mudou no código (`a052c18`)

⚠️ **Não dá pra só não lançar o nó.** O guard é um **estágio da artéria**
(`auto_vel_pre` → `auto_vel_raw`). Tirando ele, o `collision_monitor` fica **sem
publisher na entrada** e a autonomia inteira emudece. São duas metades:

| arquivo | o que faz |
|---|---|
| `ros2_packages/robot_nav/launch/nav2.launch.py` | `motion_guard:=false` → o nó **não sobe** (`condition=IfCondition`) **e** o `twist_mux_auto` publica **direto** em `auto_vel_raw` |
| `launch.sh` | `--arena` passa `motion_guard:=false` (mesmo padrão do `follow_clear_full:=1.2`) |
| `ros2_packages/robot_nav/test/test_nav2_launch_guard.py` | 5 testes; lê a LaunchDescription de verdade e resolve as substituições nos dois valores |
| `bin/pause_budget.py` | sem guard, o `auto_vel_pre` fica mudo e a ferramenta passaria a atribuir **todo** segundo parado a `mux_gap`, causa inexistente. Corrigido + aviso no relatório |

**Default é `true`.** Fora da arena nada muda: o robô de sempre segue com guard.

Verificação feita: injetei o defeito (mux fixo em `auto_vel_pre`) e **falhou
exatamente `test_sem_guard_o_mux_publica_direto_no_raw`, e mais nenhum**. Suíte
do `robot_nav`: **410 passando**.

---

## 4. As 3 voltas sem guard — ganho grande E um BO

| volta | tempo | goals | COLISÃO | raspão | parado |
|---|---|---|---|---|---|
| `hist1..3` (com guard) | 230 / 269 / 266 | 5/5 | 0 | 0 | 1,0 / 26,7 / **30,7** |
| **`noguard1`** | **227,6** | 5/5 | 0 | 0 | **0,0** |
| **`noguard2`** | **221,0** | 5/5 | 0 | 0 | **0,0** |
| `noguard3` | 245,4 | 5/5 | **9** | **48** | 3,6 |

**✅** `parado` = **0,0 s em 14 dos 15 goals**. `noguard2` é a volta mais rápida
das 14, `noguard1` a segunda. A assinatura de ~27 s sumiu.

**🔴 A `noguard3` bateu:** 9 COLISÃO + 48 raspões, folga **0,0000**
(penetração), 58 eventos entre t=60,7 e 63,6 — todos na **`A_fresta90_2`**. Pior
contato desde o `arena_baseline1`.

**Foi o guard-off que causou?** O medido vai contra, mas **não fecha**:

- a fresta A **sempre** foi passagem no fio: folga mínima **0,045–0,212 m** nas
  14 voltas, abaixo de 8 cm em 4 delas;
- nas 11 voltas com guard, o estado dele na travessia da fresta foi `idle` em
  **todas**; os únicos `slowing` do histórico somam ~6 s e foram longe (muro
  oeste 1,29 m, `C_fresta60_1` 0,95 m, `cone_4` 0,48 m). Não havia proteção ali
  pra eu ter removido;
- a `noguard3` cruzou **atrasada e torta**: t=60,9 e yaw **−5,4°**, contra
  t=35–45 s e yaw −13° a −26° nas outras 13 — depois de um `unstuck` (`near`)
  aos 50,8 s, ainda no goal 1.

> **n=3. GUARD-OFF NÃO ESTÁ VALIDADO — está medido.** Não tire taxa de contato de
> três voltas.

Evidência: `docs/baselines/2026-08-31-arena-sem-guard/`.

---

## 5. O que fazer a seguir (minha ordem)

1. **Mais 3 voltas sem guard** (~13 min, comando na §6). É a única forma de saber
   se a batida da fresta é **taxa** ou **azar**. Prazo 05/09 com meta "MUITO
   preciso": uma colisão custa mais que 25 s.
2. **Item 2k — a fresta sem correção de rumo.** A rota atravessa um vão de 90 cm
   com 4–21 cm de folga e **nada alinha o robô antes de entrar**. Um dia o rumo
   ia estar 15° fora; foi na `noguard3`.
3. **Item 2g** — 16 s em `idle` entre dois goals (`aprox3`), sem goal ativo. Não
   é o seguidor. Não investigado.
4. Os travamentos de **11–17 s sem guard nenhum** (`hist2` 2×, `aprox3`,
   `latchN3`) — a segunda causa de parada longa, ainda aberta. Curiosidade que
   não sei explicar: 3 dos 4 também estão parados ao lado do `cone_4`.

---

## 6. Como rodar (e as duas armadilhas)

```bash
AB_PARAMS=nav2_params_arena.yaml \
AB_WORLD=$PWD/worlds/arena_galpao.sdf \
AB_MAP=$PWD/maps/arena_galpao.yaml \
AB_ROTA=$PWD/maps/routes/arena_galpao.json \
AB_SX=1.0 AB_SY=1.0 \
AB_EXTRA_LAUNCH="follow_clear_full:=1.2 follow_clear_min:=0.35 motion_guard:=false" \
  bash tools/sim_ab/run_n.sh robot_nav <tag> 3
```

⚠️ **Armadilha 1 — `motion_guard:=false` é obrigatório e fácil de esquecer.**
Este harness **não passa pelo `--arena`** do `launch.sh`; ele monta os argumentos
do launch na mão. Sem essa linha a volta roda **com** o guard e traz de volta as
paradas de ~27 s, sem avisar. Conferência barata, tem que dar **0**:

```bash
grep -c motion_guard log/sim_ab/<tag>/nav2.log
```

⚠️ **Armadilha 2 — `AB_SX/AB_SY` = a largada (1.0, 1.0).** O default do harness é
(2.0, **0.0**), que na arena fica **em cima do muro sul**.

Extrair a evidência (script versionado, com autoteste, não escreve nada se
faltar bruto):

```bash
python3 tools/sim_ab/extrai_evidencia.py --autoteste
python3 tools/sim_ab/extrai_evidencia.py docs/baselines/<dir> <tag1> <tag2> <tag3>
python3 tools/sim_ab/extrai_evidencia.py --guard docs/baselines/<dir>/guard.csv <tags...>
python3 bin/pause_budget.py log/sim_ab/<tag>/freeze_capture.csv
```

⚠️ Os CSVs brutos vivem em `log/sim_ab/`, que é **`gitignore`d** — só o que está
em `docs/baselines/` sobrevive ao clone.

---

## 7. Erro meu que vale registrar (BO 65 do diário)

Atribuí as paradas longas ao churn da mira **sem ler o pipeline**. A histerese
cortou o churn 3–5× e o tempo parado não caiu. A causa era um nó que eu nunca
tinha olhado — e cuja volta ao caminho estava registrada **nos meus próprios
itens abertos** (item 7: os números de 08-27 vieram do fork *sem* `motion_guard`).

**Lição:** quando o robô não se move, ler o **pipeline inteiro** — o
`freeze_capture.csv` tem todos os estágios — antes de acusar o nó que acabei de
mexer.
