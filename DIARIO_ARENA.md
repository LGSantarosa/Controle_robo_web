# Diário da arena do galpão — prova de 2026-09-05

> **Ordem do dono (2026-08-28):** *"TUDO, TUDO o que você fizer de testes, de
> criações, de tudo, anote em um arquivo sobre, com resultados, erros, tudo."*
>
> **Como manter:** toda sessão acrescenta uma seção nova em ordem cronológica.
> Registrar o que foi **medido**, não o que se espera. Erro meu entra igual a
> erro achado — a §5 existe pra isso e não deve ser podada.
>
> Documentos irmãos: `docs/superpowers/specs/2026-08-28-arena-galpao-design.md`
> (o desenho) e `ESTADO_PROJETO.md` (estado geral do projeto).

---

## 1. Onde estamos

| | |
|---|---|
| Branch | `arena-galpao` |
| Prazo | **2026-09-05** |
| Meta | robô **sem medo de movimento e MUITO preciso** (velocidade saiu de escopo) |
| Missão | largada → 4 cones → chegada; 20 cm do cone = ponto marcado, acende LED |
| Frestas | 0,60 / 0,70 / 0,80 / 0,90 m — **passar é OPCIONAL**, sempre há contorno |
| Fora de escopo | rampa 60×60 @15° e barreira móvel oscilante (decisão do dono) |

### Critérios de aceitação e estado

| # | critério | estado |
|---|---|---|
| A1 | visita 4 cones + chegada, na ordem, sem pular após falha | ⏳ falta executor |
| A2 | para a ≤ 20 cm do cone | ⏳ falta aproximação final |
| A3 | LED acende em cada ponto | ⏳ falta nó |
| A4 | **zero contato** com bloco, cone ou parede | 🔴 **BLOQUEADO** — baseline 08-28: **2 colisões + 28 raspões, todos no `cone_3`**, pelo canto varrendo durante a samba (§2.8). Parede e bloco: **zero** |
| A5 | completa a missão **sem** atravessar fresta | ✅ no mapa; ✅ **no sim** — baseline fez 5/5 goals em 236,4 s sem depender da fresta de 0,60 |

---

## 2. Sessão 2026-08-28

### 2.1 Redirecionamento (o que mudou de rumo)

A fase anterior perseguia **velocidade** (`HANDOFF_PROVA_REAL.md`). O dono virou
o rumo: ambiente com obstáculos móveis, passagens estreitas, rampas — **sem foco
em velocidade**. Depois cortou rampa e barreira móvel do prazo
(*"esse trampo é pra meses"*).

Spec escrito e commitado: `85ee0e2`.

### 2.2 Unificação dos pacotes (`1f08a60`)

**Achado:** `launch.sh` tinha **zero** ocorrências de `nav2_trekking`. Ele
compila e lança `robot_nav` em tudo. O trabalho de 08-27 (16/16 goals) só rodava
à mão pelo `tools/sim_ab/run_one.sh`, **que recebe o pacote por argumento**.
O fork **nunca rodou pelo caminho normal nem no robô**.

Divergência real medida entre os dois pacotes:

| arquivo | linhas de diff | destino |
|---|---|---|
| `config/nav2_params_pi.yaml` | 273 | virou `nav2_params_arena.yaml` |
| `path_follower.py` | 104 | só o `speed_for_clearance` veio |
| `launch/nav2.launch.py` | 88 | só o `rear_half_width` veio |
| `unstuck_supervisor.py` | 64 | **nada** — diferia só pelo guard arrancado |
| `freeze_capture.py` | 15 | **nada** — idem |
| `trekking_runner.py` | 4 | **nada** — só comentário |

**Decisão:** dissolver o fork de volta no `robot_nav`; geometria vira perfil
`./launch.sh --nav2 --arena`.

**O que NÃO veio, e por quê:**

| deixado pra trás | motivo |
|---|---|
| velocidade 0,35 → 0,60 | era a fase VELOCIDADE, que saiu de escopo |
| remoção do `motion_guard` | o fork tirou por "não há mais pessoas nesta stack". A arena tem **obstáculo móvel**, e o guard é a cautela com objeto em movimento. **Fica.** |
| `stuck_timeout` 5,0 → 2,0 | justificativa era "esperar é tempo jogado fora numa pista". Sem limite de tempo, esperar é a atitude certa |

O perfil difere do `nav2_params_pi.yaml` por **exatamente 5 coisas**:
`robot_radius` 0.32 (×2 costmaps), inflação 0,45/0,60, e `PolygonFront` único
em modo `limit` no lugar de `PolygonSlow`+`PolygonStop`.

### 2.3 Review 1 — 6 achados, todos procedentes (`f24158b`)

| # | achado | correção |
|---|---|---|
| 1 | perfil não garante zero contato | **não corrigido em código** — parou de se chamar "modo seguro", virou "modo conservador", e virou bloqueador documentado de A4 |
| 2 | docs mandavam usar o pacote apagado | corrigido no ponto exato, não só aviso no topo |
| 3 | `--arena` falhava aberto | **falha fechada** (`exit 1`) |
| 4 | comportamento vazou pro `--nav2` normal | `speed_for_clearance` nasce desligado, entra por launch arg |
| 5 | harness não testava o perfil | `AB_PARAMS`/`AB_WORLD`/`AB_MAP`/`AB_ROTA`/`AB_SX`/`AB_SY`/`AB_EXTRA_LAUNCH` |
| 6 | comentários contradiziam o runtime | pipeline do yaml e help do `--arena` corrigidos |

### 2.4 Review 2 — o `rm -rf` (`7b6e50c`)

**`run_one.sh` com tag vazia apagava `log/sim_ab/` inteiro.** Essa pasta tem
18 entradas e **não são só voltas**: `kill_all.sh`, `colisao.py`, `probe.py` e
`consolida.py` moram lá. Um argumento esquecido apagava o harness, **inclusive o
`kill_all.sh` que o próprio script executa 30 linhas abaixo**.

**Achado meu, puxando o mesmo fio (não estava no review):** o harness **executava
as cópias, não o repo**. `run_n.sh` chamava `"$SP/run_one.sh"`; `run_one.sh`
chamava `"$SP/kill_all.sh"`, `"$SP/probe.py"`, `"$SP/colisao.py"` — tudo de
`log/sim_ab/`, que é **`gitignore`d** (`.gitignore:20`) e guarda cópias de 27/08.

Três consequências, a terceira sendo a pior:
1. o código que rodava não era o do git;
2. correção no repo não chegava na execução;
3. **a guarda que eu tinha acabado de escrever não protegeria a chamada real.**

Corrigido: scripts saem de `TOOLS` (pasta do próprio arquivo, no repo), `$SP` é
**só saída**. 9 invocações redirecionadas. Isso torna o `rm -rf` inofensivo por
construção — não há mais ferramenta dentro da pasta de saída.

### 2.5 Review 3 (`05d5188`)

- `run_n.sh`: guarda de `PREFIXO` **não era redundante** — ele nomeia
  `"$SP/${PREFIXO}_TERMINOU"` (linha 58) e o lock, então `../x` escapava do `SP`
  antes do `run_one.sh` ser chamado.
- spec: passo 2 mandava corrigir "as duas URDFs" enquanto a §3 já dizia que ela
  é única desde `1f08a60`.
- comentário invertido (erro meu, ver §5).

### 2.6 Validadores (`e440cb5`, `1cd9bda`)

**`colisao.py` — dois bugs. O primeiro reescreve o passado:**

> `sala_grande.sdf` tem **26 geometrias de colisão** (20 caixas + 4 cilindros
> isolados + as **2 pernas** do modelo `pessoa`). O oráculo de 08-27 enxergava
> **20** — só as caixas. O *"zero colisões, folga mínima 3,7 cm"* foi concluído
> com os cilindros **fora da conta.**
>
> ⚠️ **Correção do review 4:** eu primeiro escrevi "25 obstáculos, 5 cilindros".
> Errado por dois motivos: eu contava MODELOS, não geometrias, e meu parser
> corrigido ainda pegava **só a primeira `<collision>` de cada modelo, ignorando
> a pose local** — então as duas pernas da `pessoa` viravam UM cilindro no centro
> do modelo, que é o espaço **vazio entre as pernas**. Corrigido: todas as
> `<collision>`, cada uma com sua pose local girada pelo yaw do modelo. As pernas
> agora aparecem em y 3,65 e 3,35 (modelo em 3,5 ± 0,15).

Segundo bug: lia `<box>` de qualquer lugar, inclusive `<visual>`. Em
`sala_grande` não havia caso (conferido), mas na arena a plataforma amarela é
visual pura e viraria obstáculo fantasma — acusaria "colisão" toda vez que o robô
pisasse no alvo. Agora só `<collision>`.

**E uma precisão de linguagem:** contra CAIXA o **sinal** do SAT é exato (contato
é contato), mas o **valor positivo** é o maior gap entre os eixos testados — uma
**cota inferior** da distância euclidiana (com separação diagonal a real é
`hypot(gx,gy) ≥ max(gx,gy)`). Erra pro lado seguro, mas não é "folga exata".
Contra CILINDRO o valor é exato.

**`mapa_passagens.py`** media só o **maior componente conectado**: uma fresta
podia fechar e o percentual continuar 100%. Ganhou `--probe` (A→B) e `--folga`
(mede o vão em si). Para isso passou a ler o `origin` do yaml (não lia).

### 2.7 A arena (`fe475db`, `ffce067`)

`worlds/arena_galpao.sdf` (14×9 m, 23 models) e `maps/arena_galpao.{pgm,yaml}`,
os dois gerados de `tools/gera_arena_galpao.py` — **fonte única**, porque as
mesmas coordenadas aparecem no mundo, no mapa e nos probes.

| decisão | motivo |
|---|---|
| bloco com **0,60 m de espessura** | visto de topo, bloco fino mostra face de ~0,40 m, que cai na janela 0,04–0,45 do `cone_detector` e vira **cone falso**. Com 0,60 fica acima do teto da janela. De quebra a fresta vira túnel curto = caso realista pro `door_crossing` |
| plataforma **só `<visual>`** | é marca de chão; o laser não vê. Conferido: `colisao.py` ignora as 6 |
| **mapa sem os cones** | o cone é o OBJETIVO e a missão manda chegar a 20 cm. Na camada estática o goal nasceria dentro de obstáculo+inflação e o nav2 recusaria. O cone segue no MUNDO (laser vê, `obstacle_layer` marca ao vivo, `colisao.py` cobra contato). `--com-cones` gera a variante |

---

### 2.8 BASELINE Nav2 até os standoffs — primeira volta na arena (2026-08-28)

Comando: o do §4.5. Saídas arquivadas em `log/sim_ab/arena_baseline1/`
(`result.json`, `colisao.csv`, `follow_debug.csv`, `unstuck.csv`, `freeze_capture.csv`
— os três últimos copiados de `controle_web/logs/` logo após a volta, porque são
**sobrescritos a cada launch**).

⚠️ **Isto é baseline até os STANDOFFS, não missão.** Não prova A1, A2 nem A3.

#### Resultado

| | |
|---|---|
| goals | **5 de 5** |
| tempo total | **236,4 s** |
| contato com parede/bloco | **zero** |
| contato com CONE | 🔴 **2 colisões + 28 raspões, TODOS no `cone_3`** |
| unstuck | nunca disparou (2203 amostras, todas `monitoring`) |

Por goal: 43 s / 34 s / 66 s / 50 s / 43 s. O goal 3 é o dobro dos outros — e é
onde estão os dois contatos.

> **O oráculo antigo teria dito "zero colisões".** Cone é cilindro, e até
> `e440cb5` o `colisao.py` só lia caixas. O primeiro uso do oráculo corrigido já
> pegou o contato que a versão anterior não veria.

#### 🔴 A "samba" no goal — diagnosticada

O dono viu ao vivo: *"ele fica sambando tentando achar o ângulo e o ponto exato,
mas já está em cima do goal faz tempo, isso deixa ele burro"*.

**Não é limite-ciclo em torno do yaw do goal** (foi minha primeira hipótese, e o
CSV a derrubou: zero inversões de sinal DENTRO de cada bloco). São **dois
controladores brigando através de um limiar sem histerese**:

```
 4.0  driving    dist 0.166           <- se aproxima
 4.3  goal_turn  dist 0.153  wz +4.50 <- cruza 0.15, gira pro YAW DO GOAL
 6.3  goal_turn  dist 0.161  wz +2.40 <- girando no lugar, DERIVA pra 0.161
 6.6  turning    dist 0.174  wz -4.50 <- passou de 0.15: cai fora e INVERTE o giro
10.3  driving    dist 0.185           <- 4 s girando pro outro lado, deriva mais
10.6  goal_turn  dist 0.154  wz +4.50 <- volta pra baixo de 0.15... recomeça
```

- `goal_turn` gira pro **yaw do goal**; `turning` gira pra **mira do carrot**.
  Querem lados opostos — por isso o `wz` troca de sinal no instante da troca de
  estado.
- Quem arbitra é um `dist_goal <= goal_xy_tol` **pelado**, sem histerese
  (`path_follower.py:295`).
- O giro no lugar do skid **desloca o robô ±5 cm** — então **o próprio giro que o
  limiar dispara é o que faz cruzar o limiar de volta**. Se auto-alimenta.
- 13 blocos de `goal_turn`, cada um **reiniciando com um yaw novo** (26°, 115°,
  157°, 177°…), ~35 s só no goal 3.

**A doença já foi curada uma vez neste arquivo.** O docstring (linhas 16-18)
conta: *"ele girava e parava no MESMO limiar → limite-ciclo = pulinhos"*,
resolvido com histerese `turn_enter`/`turn_exit`. **O limiar de CHEGADA nunca
recebeu o mesmo remédio.**

#### 🔴 O contato no cone É a samba

| | |
|---|---|
| dist ao goal 3 | chegou a **0,04 m**, derivou até **0,59 m** |
| pose no contato | robô (12,16 · 6,99), yaw −40,6°; cone a **0,51 m** do centro |
| geometria | superfície do cone a **0,34 m** do centro do robô; o canto varre **0,354** |
| de onde | cone a **+126° do nariz** = canto **traseiro**, girando |

Ou seja: **a samba andou meio metro com o robô e encostou o canto de trás no
cone.** Não é point-turn genérico perto de parede — é a deriva do vaivém.

**Os 30 eventos são DOIS episódios de ~0,7 s, e a assinatura é inequívoca:**

```
   t      folga(cm)  pose                 yaw
 113.3      1.6      (12.16, 6.99)      -37.1
 113.4      0.2      (12.16, 6.99)      -39.6
 113.5     -0.0      (12.16, 6.99)      -40.6   <- COLISAO
 113.6      0.2      (12.16, 6.98)      -44.2
 114.0      2.0      (12.14, 6.96)      -53.7
```

**A posição não muda** (12,16 → 12,14) e **o yaw varre** −37° → −54°. A folga
mergulha até zero e volta a subir. É o **canto passando pelo cone durante o giro
no lugar** — o cone a +126° do nariz é o canto **traseiro-esquerdo** (os cantos
ficam a ±45° e ±135°).

Nenhum evento em parede ou bloco: **os 30 são no `cone_3`.**

**Consequência pra A4:** o bloqueador tem agora um caso reproduzível e medido. E
a hipótese é que travar a chegada mate os dois problemas de uma vez, porque a
deriva vinha do vaivém.

#### Correção proposta (⏳ não implementada, aguardando o dono)

Travar a chegada: quando `dist_goal` cruzar `goal_xy_tol` pela primeira vez
naquele goal, entra em fase de chegada e **não volta mais** pro carrot — só
`goal_turn` até o yaw fechar, então `arrived`. Solta a trava só com goal novo (ou
se algo empurrar o robô pra além de ~3× a tolerância).

Plano: **teste primeiro** (reproduzir o chatter com a sequência de poses do CSV,
ver falhar), depois travar, depois **repetir esta mesma volta** e comparar contra
este baseline — 236,4 s e **2 contatos**.

---

## 3. Medições

### 3.1 Geometria do robô

| grandeza | valor | fonte |
|---|---|---|
| envelope roda-a-roda | **0,50 × 0,50 m** | trena do dono, 08-28 |
| estrutura de alumínio | 0,37 × 0,35 × 0,16 | URDF / `sim_robot.sdf` |
| raio inscrito / circunscrito | 0,25 / **0,354** | extremos das rodas |
| altura do LiDAR | **NÃO MEDIDA** | URDF dá 0,465; SDF do sim dá 0,2825 |

**Bug aberto na URDF real:** `track_width` (0,50) e `wheelbase` (0,37) são
centro-a-centro, mas os valores são **medidas externas** → a URDF descreve um
robô de **0,56 × 0,54**. Valores corretos: **0,44** e **0,33**.
⏳ corrigir só após medir centro-a-centro no robô.

### 3.2 Fresta de 60 cm — orçamento de erro

Largura efetiva com yaw θ: `W(θ) = 0,50·(cos θ + sen θ)`

| yaw | largura | folga lateral restante |
|---|---|---|
| 0° | 0,500 | ±5,0 cm |
| 5° | 0,542 | ±2,9 cm |
| 10° | 0,579 | ±1,0 cm |
| **13°** | **0,600** | **0 — encosta** |

### 3.3 Arena, medido no mapa gerado (raio 0,32)

```
fresta 0,90  folga 0,450 m  -> PASSA
fresta 0,70  folga 0,350 m  -> PASSA
fresta 0,60  folga 0,300 m  -> FECHADO   <-- o esperado
fresta 0,80  folga 0,400 m  -> PASSA

4 pernas cone->cone: LIGADAS nos raios 0,25 / 0,32 / 0,354
```

**Leitura:** só a de 0,60 fecha, e a missão fecha sem ela. **A5 provado no nível
do mapa.** Falta provar no sim.

### 3.4 Isolamento do perfil (medido)

| config | folga 0,41 m | |
|---|---|---|
| `--nav2` normal (`clear_full` 0.0) | 0,300 m/s | no-op, como antes |
| perfil arena (`clear_full` 1.2) | **0,226 m/s** | freia na aperto |

---

## 4. Como reproduzir tudo

> ⚠️ **CORRIGIDO 2026-08-28 (review 4).** A versão anterior deste bloco **não
> rodava a arena**: o `launch.sh` caía nos defaults (`worlds/sala.sdf`,
> `maps/hotmilk_portas.yaml`, spawn 2.0/2.5) e o A/B usava `maps/routes/rota1.json`,
> cujos pontos têm **coordenadas negativas** — fora da arena inteira. Os comandos
> abaixo estão conferidos.

### 4.1 Autotestes (não precisam de ROS nem do robô)

```bash
python3 tools/gera_arena_galpao.py --conferir      # invariantes da arena + rota
python3 tools/mapa_passagens.py   --autoteste      # mapa sintético, vão 0,60
python3 tools/sim_ab/colisao.py   --autoteste      # 7 casos de geometria
python3 -m pytest ros2_packages/robot_nav/test/ -q # 393 testes
```

### 4.2 Regenerar mundo, mapa e rota (tudo da mesma tabela)

Os três artefatos **são versionados** (exceção explícita no `.gitignore`, que
ignora `maps/` por default): a Pi deploya por `git reset --hard` e **não roda o
gerador**. Regenerar serve pra conferir que o commitado bate com a tabela.

```bash
python3 tools/gera_arena_galpao.py --sdf  worlds/arena_galpao.sdf
python3 tools/gera_arena_galpao.py --mapa maps/
python3 tools/gera_arena_galpao.py --rota maps/routes/arena_galpao.json
```

### 4.3 Validar o mapa (argumentos gerados pela própria tabela)

```bash
python3 tools/mapa_passagens.py maps/arena_galpao.yaml \
  $(python3 tools/gera_arena_galpao.py --probes | tr '\n' ' ')
```

### 4.4 Subir a arena no sim

O `--arena` escolhe **só o perfil de params**. Mundo, mapa e spawn são
argumentos à parte — sem eles o launcher abre `sala.sdf` com `hotmilk_portas`:

```bash
./launch.sh --sim --nav2 --arena \
  --world=$PWD/worlds/arena_galpao.sdf \
  --map=$PWD/maps/arena_galpao.yaml \
  --spawn-x=1.0 --spawn-y=1.0
```

### 4.5 Uma volta A/B na arena

⚠️ **O que esta volta mede, e o que NÃO mede.** A rota para nos **standoffs, 1 m
antes de cada cone**. Ela mede **navegação, colisão e point-turn** — é o baseline
do Nav2 até os standoffs. Ela **não** prova A1 (missão completa), **nem** A2
(chegar a 20 cm), **nem** A3 (LED): esses dependem da aproximação final e do
executor, que ainda não existem. Não rotular como "missão validada".

```bash
AB_PARAMS=nav2_params_arena.yaml \
AB_WORLD=$PWD/worlds/arena_galpao.sdf \
AB_MAP=$PWD/maps/arena_galpao.yaml \
AB_ROTA=$PWD/maps/routes/arena_galpao.json \
AB_SX=1.0 AB_SY=1.0 \
AB_EXTRA_LAUNCH="follow_clear_full:=1.2 follow_clear_min:=0.35" \
  bash tools/sim_ab/run_one.sh robot_nav arena_v1
```

`AB_SX/AB_SY` = a **largada** (1.0, 1.0). O default do harness é (2.0, **0.0**),
que na arena fica **em cima do muro sul**.

## 5. Erros que EU cometi (não podar esta seção)

| # | erro | como apareceu | lição |
|---|---|---|---|
| 1 | Afirmei que a rampa cria **parede fantasma a 1,74 m** no meio da subida | Em inclinação constante o plano do laser inclina junto e fica ~paralelo à rampa. O risco está nas **transições** | Não transformar uma conta trigonométrica em conclusão física sem checar o referencial |
| 2 | Tratei a **altura do LiDAR (0,465 m)** como fato | É valor derivado da URDF; o SDF do sim dá 0,2825 e os docs de campo falam em ~0,21 | Arquivo de modelo não é medição |
| 3 | Disse que o robô do sim era **13 cm mais curto** | Olhei só a caixa do corpo; as **rodas** preenchem o envelope | Ler a geometria inteira antes de acusar |
| 4 | Inventei que o robô é **preenchido entre as rodas** | O dono corrigiu: "desde quando o robô é preenchido entre as rodas, porra". "Carcaça externa 50×50" do comentário é o **envelope**, não uma caixa sólida | Comentário de dimensão ≠ geometria de colisão |
| 5 | Chamei o perfil de **"modo seguro"** | O próprio yaml já documentava que point-turn colado na parede não é coberto | Não batizar de seguro o que eu não medi |
| 6 | Apaguei o `nav2_trekking` e deixei a **doc mandando usá-lo** | `HANDOFF_PROVA_REAL` mandava rodar um `colcon build` que falha | Apagar código é metade; a outra metade é a doc que o chama |
| 7 | `speed_for_clearance` como **default global** | Vazou pro `--nav2` normal, que ninguém pediu | Comportamento de perfil entra por parâmetro, não por default |
| 8 | Meu replace em massa **inverteu um comentário** | Trocou `"$SP/..."` → `"$TOOLS/..."` dentro do texto explicativo, que passou a dizer o contrário do que houve | Replace cego pega comentário também |
| 9 | Regra de conferência **errada**: exigi as duas pontas da ilha vedadas | Isso eliminaria o contorno, que é o requisito A5. O `--conferir` reprovou e me mostrou | O validador pegou meu erro — é pra isso que ele existe |
| 10 | `pinta_caixa` com `floor`/`ceil` + 1 **inflava o bloco** | A fresta de 0,70 media **0,60** no mapa: o validador declarava fechada uma passagem que existe. **Silencioso** | Rasterização tem que medir a mesma largura da geometria contínua, e isso virou invariante conferido |
| 11 | Mensagem de commit com **crase** foi comida pela shell | `origin: command not found`, e o campo sumiu do texto | Heredoc quoted pra mensagem de commit |
| 12 | `--probes` gerava **rótulo com espaço** | Quebrava ao expandir na shell | Saída feita pra colar tem que ser colável |
| 13 | O bloco "Como reproduzir" **não reproduzia** | `--sdf` nem existia; `launch.sh --arena` caía em `sala.sdf`+`hotmilk_portas`; o A/B usava `rota1.json`, com coordenadas **negativas**, fora da arena | Comando publicado tem que ser **executado** antes de publicado |
| 14 | Parser pegava **só a primeira `<collision>`** e ignorava a pose local | As 2 pernas da `pessoa` viravam 1 cilindro no vazio entre elas | Corrigir um parser pela metade e anunciar "ground truth completo" |
| 15 | Chamei o valor do SAT de **"folga exata"** | Para caixa é cota inferior, não distância euclidiana | Não superqualificar a métrica |
| 16 | Probes de A5 **omitiam largada→cone_1** | A conclusão era verdadeira, mas o conjunto reproduzível não a cobria | Prova automática tem que cobrir a perna toda |
| 17 | Comentários do yaml diziam que o raio **"virou 0,354"** | O valor em vigor é 0,32; 0,354 foi testado e REPROVADO | Ao mudar o valor, varrer os comentários que o citam |
| 18 | **O mapa e a rota da arena nunca foram commitados** | `maps/` inteiro é `gitignore`d (`.gitignore:15`). Só o `.sdf` entrou. Num checkout limpo — **ou na Pi, que deploya por `git reset --hard`** — existe o mundo e não existe mapa nem rota | Conferir `git ls-files` do artefato, não só `git status` limpo |
| 19 | Gerador não criava o diretório pai | `--rota` quebrava em checkout limpo; passou aqui só porque a pasta já existia | `os.makedirs(exist_ok=True)` antes de escrever |
| 20 | Autoteste do `colisao.py` cobria só a **matemática**, não o parser XML | A regressão das duas pernas não tinha teste — o bug que eu tinha acabado de corrigir podia voltar em silêncio | Bug corrigido sem teste é bug agendado |
| 21 | Escrevi que caixa girada dá folga **"subestimada"** | Não é garantido: ignorar a rotação pode inventar folga **ou perder contato**. O resultado é NÃO CONFIÁVEL, não conservador | Não vender limitação como se fosse margem de segurança |
| 23 | Diagnostiquei a samba como **limite-ciclo do yaw** antes de olhar o CSV | Zero inversões de sinal dentro de cada bloco derrubaram isso. A causa é chattering entre DOIS controladores num limiar sem histerese | A assinatura estava no dado; eu quase entreguei a causa errada |
| 25 | Reportei **"2 contatos"** olhando só o resumo do `colisao.log` | O CSV tinha **30 eventos**: 2 colisões + **28 raspões**. O resumo imprime só as colisões | Ler o CSV, não o resumo — foi a lição do "média esconde bimodal" outra vez |
| 24 | **Rodei a volta e não anotei** | O dono teve que perguntar "anotou isso tudo?". Pior: os CSVs de `controle_web/logs/` são sobrescritos a cada launch — quase perdi o `follow_debug` que sustenta o diagnóstico | Arquivar CSV e escrever o diário fazem parte da run, não são pós-jogo |
| 22 | No teste novo do parser, **minha expectativa estava errada** | Escrevi (2,0 · 1,0) — que é o resultado de **ignorar** o yaw. O teste teria passado na versão com bug | Ao testar rotação, afirmar também o valor que o bug produziria |

---

## 6. Aberto

| # | item | estado |
|---|---|---|
| 1 | **Proteção de point-turn** (anel 0,25–0,36 m no `path_follower`) | 🔴 bloqueador de A4. **Não é** no collision monitor: lá a única alavanca é escalar o `wz`, que trava o robô (deadlock já reproduzido) |
| 2 | Baseline Nav2 até os standoffs | ✅ **FEITO 08-28** — 5/5 goals, 236,4 s, 2 colisões + 28 raspões no cone (§2.8). Evidência em `docs/baselines/2026-08-28-arena-baseline1/` |
| 2b | Travar a chegada (mata a samba e, com ela, o contato no cone) | ⏳ proposto, aguardando o dono |
| 3 | Executor que não pula ponto após falha | ⏳ |
| 4 | Aproximação final ao cone (A2) | ⏳ o `PolygonFront` bloqueia o avanço a ~0,67 m do centro do cone, **antes** dos 20 cm |
| 5 | LED/relé | ⏳ interface já existe: `/light/marker` (pino 8) e `/light/cmd` (pino 7) no `mega_bridge` |
| 6 | Medir no robô: bitola, entre-eixos, altura do LiDAR | ⏳ trena |
| 7 | Refazer os números de 08-27 | ⏳ os antigos vieram do fork **sem `motion_guard`** e com oráculo cego a 5 obstáculos |
| 8 | Bug aberto `"start/goal is an obstacle"` | ⏳ anterior a esta fase |
