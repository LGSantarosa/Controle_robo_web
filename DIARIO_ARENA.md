# Diário da arena do galpão — prova de 2026-09-05

> **Ordem do dono (2026-08-28):** *"TUDO, TUDO o que você fizer de testes, de
> criações, de tudo, anote em um arquivo sobre, com resultados, erros, tudo."*
>
> **Como manter:** toda sessão acrescenta uma seção nova em ordem cronológica.
> Registrar o que foi **medido**, não o que se espera. Erro meu entra igual a
> erro achado — a §5 existe pra isso e não deve ser podada.
>
> ### ⛔ A REGRA: arquivo ANTES do chat
>
> **Ordem do dono (2026-08-28, terceira e mais dura):** *"sempre que vc trouxer
> uma conclusão aqui no chat, leve para o arquivo antes"*.
>
> **A ordem das operações é essa, literalmente:**
>
> 1. tirei uma conclusão / diagnóstico / recomendação
> 2. **escrevo AQUI**
> 3. **só então** mando no chat
>
> Não é "anoto depois", não é "anoto quando ele pedir". As duas primeiras versões
> desta regra eram *"anote tudo"* e *"anota no arquivo quando trouxer pra mim"* —
> e nas duas o dono teve que perguntar **"anotou?"**, porque eu mandava primeiro e
> registrava depois (erros 24 e 38 da §5). O chat vira o **resumo** do que já está
> no arquivo, nunca a fonte.
>
> Vale pro parágrafo inteiro que eu mando, não só pra parte tabelada — foi assim
> que o plano do point-turn escapou. E vale para conclusão que depois se mostrar
> errada: ela **fica**, com a correção do lado (é o caso de "a samba encostou no
> cone", §2.8).
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
| A4 | **zero contato** com bloco, cone ou parede | 🟡 **1 volta limpa, ainda NÃO fechado** — baseline 08-28: 2 colisões + 28 raspões, todos no `cone_3`. Com o latch (08-31): **zero evento, folga mínima +7,4 cm** (§2B.4). Mas é **n=1**: A4 pede repetição, e o point-turn sem proteção + o erro de pose seguem de pé |
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
| unstuck (nosso) | nunca disparou (2203 amostras, todas `monitoring`) |
| recoveries do **Nav2** | 1 `No valid trajectories`, 1 `start/goal is an obstacle`, 3 `Failed to make progress` |
| erro de pose AMCL × Gazebo | mediana **9,0 cm**, p90 15,8, **24,1 cm nos contatos** |

Por goal: 43 s / 34 s / 66 s / 50 s / 43 s. O goal 3 é o dobro dos outros — e é
onde estão os dois contatos.

> **O oráculo antigo teria dito "zero colisões".** Cone é cilindro, e até
> `e440cb5` o `colisao.py` só lia caixas. O primeiro uso do oráculo corrigido já
> pegou o contato que a versão anterior não veria.

#### 🔴 A "samba" no goal — mecanismo confirmado, números corrigidos

O dono viu ao vivo: *"ele fica sambando tentando achar o ângulo e o ponto exato,
mas já está em cima do goal faz tempo, isso deixa ele burro"*.

**Não é limite-ciclo em torno do yaw do goal** (foi minha primeira hipótese; o CSV
a derrubou — zero inversões de sinal DENTRO de cada bloco). São **dois
controladores brigando através de um limiar sem histerese**:

```
 4.0  driving    dist 0.166           <- se aproxima
 4.3  goal_turn  dist 0.153  wz +4.50 <- cruza 0.15, gira pro YAW DO GOAL
 6.3  goal_turn  dist 0.161  wz +2.40 <- girando no lugar, deriva pra 0.161
 6.6  turning    dist 0.174  wz -4.50 <- passou de 0.15: cai fora e INVERTE o giro
10.3  driving    dist 0.185           <- 4 s girando pro outro lado
10.6  goal_turn  dist 0.154  wz +4.50 <- volta pra baixo de 0.15... recomeça
```

- `goal_turn` gira pro **yaw do goal**; `turning` gira pra **mira do carrot**.
  Querem lados opostos — por isso o `wz` troca de sinal no instante da troca.
- Quem arbitra é um `dist_goal <= goal_xy_tol` **pelado**, sem histerese
  (`path_follower.py:295`).
- O giro no lugar do skid **desloca o robô**, então o giro que o limiar dispara
  ajuda a cruzar o limiar de volta.

**Números (corrigidos no review 7 — a versão anterior deste parágrafo exagerava):**

| | |
|---|---|
| blocos de `goal_turn` na **volta inteira** | 13, somando **13,8 s** |
| no **goal 3** | **7 blocos, 8,5 s** |
| nos outros goals | 1 / 2 / 1 / 2 blocos, ≤ 2,6 s cada |
| `dist_goal` no goal 3, **depois** de entrar na tolerância | entrou 0,139 → min **0,066** → max **0,281** m |

⚠️ A versão anterior dizia "13 blocos e ~35 s no goal 3" (13 é a volta inteira; os
35 s eram o tamanho da janela de CSV que eu arquivei) e chamava 115°/157°/177° de
"yaw do robô" — são **`herr_deg`, o ERRO de yaw**. E dizia "chegou a 0,04 m e
derivou até 0,59 m": os 0,59 são de **antes** da chegada.

**A doença já foi curada uma vez neste arquivo.** O docstring (linhas 16-18)
conta: *"ele girava e parava no MESMO limiar → limite-ciclo = pulinhos"*,
resolvido com histerese `turn_enter`/`turn_exit`. **O limiar de CHEGADA nunca
recebeu o mesmo remédio.** Isso é fato do código, independente da causa do contato.

#### 🔴 O contato no cone — assinatura clara, causa AINDA EM ABERTO

Os 30 eventos são **dois episódios de ~0,7 s**, com assinatura inequívoca
(pose do **Gazebo**, ground truth):

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
ficam a ±45° e ±135°). Nenhum evento em parede ou bloco: os 30 são no `cone_3`.

⚠️ **Mas eu escrevi que "a samba andou meio metro com o robô e encostou", e isso
é forte demais.** Existe um segundo suspeito, medido abaixo, e eu tinha misturado
duas fontes de pose: o `colisao.csv` usa o **Gazebo**, o `dist_goal` do
`follow_debug` usa o **AMCL**. A samba provavelmente participa; **provar exige
repetir a volta com o latch.**

#### 🔴🔴 ACHADO NOVO: o AMCL erra 24 cm na arena

Alinhando as duas séries pelo yaw (offset +3,45 s, erro de yaw médio 1,3°):

| erro de pose AMCL × Gazebo | |
|---|---|
| mediana da volta | **9,0 cm** |
| p90 | **15,8 cm** |
| máximo | **27,1 cm** |
| **nos 30 eventos de contato** | **24,1 cm** (21,0–26,1) |

**Isto reordena a prioridade do projeto.** A spec dizia *"no sim o erro de pose é
zero, no real não é"* — **falso nesta arena.** Consequências diretas:

- a fresta de 0,60 m exige **yaw ≤ 5° e lateral ≤ ±3 cm** (§3.2). Com 24 cm de
  erro de pose ela é **impossível**, aqui, no sim;
- o critério A2 (parar a **20 cm** do cone) é menor que o erro de pose;
- no contato, o robô "achava" que estava a 24 cm de onde estava de fato — então
  **erro de localização é co-suspeito do contato, junto com a samba.**

Suspeita da causa (⏳ não investigada): a arena é pobre em feature — muro liso e
8 blocos — e o **mapa não tem os cones** (decisão consciente, §2.7), então os 4
cones que o laser vê são obstáculos não-mapeados que o AMCL tem que ignorar.

#### ⚠️ O Nav2 FEZ recoveries (o diário anterior omitiu)

"unstuck nunca disparou" vale só pro **nosso supervisor**. O `nav2.log` mostra:

| ocorrência | onde |
|---|---|
| `No valid trajectories out of 35!` | 1× |
| `Either of the start or goal pose are an obstacle!` | 1×, de **(7,14 · 2,40)** para o goal 2 — o start está **dentro da fresta A (0,90 m)** |
| `Failed to make progress` | **3×**, todas no goal 3 (a janela da samba) |

O bug `"start/goal is an obstacle"` **foi reproduzido aqui**, então parar de
chamá-lo de "anterior a esta fase": ele está vivo nesta arena, e disparou
exatamente quando o robô estava atravessando uma fresta.

#### Correção proposta (⏳ não implementada, aguardando o dono)

Travar a chegada: quando `dist_goal` cruzar `goal_xy_tol` pela primeira vez
naquele goal, entra em fase de chegada e **não volta mais** pro carrot — só
`goal_turn` até o yaw fechar, então `arrived`. Solta a trava só com goal novo (ou
se algo empurrar o robô pra além de ~3× a tolerância).

Isso mata a briga dos dois controladores — que é um defeito real do código,
provado. Que **também** elimine o contato no cone é **HIPÓTESE**, não conclusão:
o AMCL errando 24 cm é co-suspeito e o latch não mexe nele.

Plano: **teste primeiro** (reproduzir o chatter com a sequência de poses do CSV,
ver falhar), depois travar, depois **repetir esta mesma volta** e comparar contra
este baseline — 236,4 s, 13 blocos de `goal_turn` (13,8 s), **2 colisões + 28
raspões**. ~~Se o contato sobreviver ao latch, o suspeito é a localização.~~
⚠️ **ESTA FRASE CAIU (review 8, ver §2.9):** é binária — sobra também o
**point-turn sem proteção**, que eu mesmo provei varrer o canto. Dois suspeitos
provados não viram um por eliminação de um terceiro.

**A proteção genérica de point-turn continua necessária** — o latch não a
substitui.

---

### 2.9 Conclusão da sessão e ordem recomendada (revisada 08-28, review 8)

> ⚠️ A primeira versão desta seção tinha **sete conclusões fortes demais**. O que
> caiu está marcado; o que sobreviveu está com a evidência do lado.

#### O que está PROVADO

| # | defeito | prova |
|---|---|---|
| 1 | **Samba no goal** — limiar de chegada sem histerese (`path_follower.py:295`) | troca de estado `goal_turn`↔`turning` com inversão de giro, no CSV |
| 2 | **O canto varre no giro** — 0,354 m, sem proteção nenhuma | posição parada (12,16→12,14), yaw varrendo (−37°→−54°), folga a 0,0 cm |

#### O que está MEDIDO, mas com causa em aberto

**Erro de pose AMCL × Gazebo:** mediana 9,0 cm na volta, 24,1 cm nos 30 eventos.

**E ele cresce no giro** — que é a pista mais forte:

| estado do seguidor | n | mediana | p90 |
|---|---|---|---|
| `driving` | 2854 | **8,4 cm** | 13,7 |
| `turning` | 1319 | **10,9 cm** | **22,8** |
| `goal_turn` | 268 | 9,9 cm | **17,5** |

⚠️ **Corrigido no review 9.** A primeira versão dizia 2914 / 1319 / 290 e p90 de
16,3 para o `goal_turn`. Viés meu: eu usava `searchsorted` **sem recortar o
intervalo comum**, então **82 amostras** do `colisao.csv` que caem fora da janela
do seguidor eram grudadas no primeiro/último estado. Contagem crua do CSV do
seguidor: 2607 / 1217 / 250 (+5 `arrived`). A conclusão — **erro maior girando** —
sobrevive à correção, e o p90 do `goal_turn` na verdade **piora** (16,3 → 17,5).

Isso casa com algo **já documentado e medido neste repo**: o comentário da IMU em
`sim_robot.sdf` registra que num skid-steer *"a roda patina no point-turn: medido
com verdade-terreno, a odom girava 7,8° no primeiro tick de cada giro enquanto o
robô girava 0,5°"*. **Hipótese melhor sustentada que a minha:** erro de odometria
no giro, não cone fora do mapa.

#### ❌ O que eu afirmei e NÃO se sustenta

| eu escrevi | por que cai |
|---|---|
| *"a spec dizia que no sim o erro de pose é zero"* | A frase existe, mas em **`HANDOFF_PROVA_REAL.md:42`** — citei o documento errado |
| *"com 24 cm a fresta de 0,60 é impossível"* | Os 24 cm são erro **euclidiano perto do `cone_3`**, não erro **lateral dentro de uma fresta**. E o costmap **local** roda em `odom` com o laser ao vivo — o desvio de obstáculo não depende da pose absoluta do mesmo jeito |
| *"o critério A2 é menor que o erro de pose"* | A2 é medido pelo **`cone_detector`, na distância relativa do scan** (spec §4), não pela pose absoluta do AMCL |
| *"testar `--com-cones` muda uma variável"* | O **mesmo mapa** alimenta AMCL **e** costmap global: muda localização **e** planejamento. É experimento exploratório, não teste causal |
| *"os standoffs podem cair na inflação dos cones"* | **Medido, falso:** gerando com `--com-cones`, os goals ficam com **0,85–0,90 m** de folga contra inflação de 0,60. Pior: eu me contradisse — o `STANDOFF = 1,0 m` foi escolhido justamente pra ficar fora dos 0,77 m |
| *"se o contato sobreviver ao latch, o suspeito é a localização"* | Binário demais. Sobra também o **point-turn sem proteção**, que eu mesmo provei varrer o canto |

#### Ordem recomendada (revisada)

O fio comum dos dois defeitos provados **e** do crescimento do erro de pose é o
mesmo: **girar**. Então a ordem muda:

1. **Proteção de point-turn.** Ataca o **modo de falha observado** (canto varrendo,
   mecanismo provado) e não depende de resolver a localização. ⚠️ **Não "fecha
   A4" por si:** quem fecha A4 é uma **volta completa com zero evento**. Ela é a
   candidata mais direta, não a garantia. **Detalhe na §2.10.**
2. **Latch da chegada.** Defeito provado por si só, e reduz a exposição: menos
   giro desnecessário colado no alvo. Com teste antes, e repetindo esta volta
   contra o baseline (236,4 s · 13 blocos de `goal_turn` · 2 colisões + 28 raspões).
3. **Investigar o erro de pose** — **exploratório**, sem decidir nada sozinho.
   Primeiro a hipótese barata e já documentada (odometria no point-turn); o
   `--com-cones` fica como experimento, ciente de que mexe em duas coisas.

⏳ **Aguardando o dono decidir a ordem.**

### 2.10 Proteção de point-turn — desenho (⏳ NÃO implementada, e ainda não pronta)

> Revisada no review 9. A primeira versão dizia *"olhar o anel 0,25–0,36 e
> recusar o giro, afastar ou girar pro lado livre"* — **isso não é comportamento,
> é intenção.** O que faltava está resolvido abaixo; o que continua em aberto está
> marcado ⏳.

**Onde:** no `path_follower`, antes de iniciar giro no lugar. **Não** no
`collision_monitor` — lá a única alavanca é escalar o `wz`, e giro abaixo da
zona-morta do skid (1,7) é comando morto: o robô patina e **não vira**, justo
quando girar era a saída. Deadlock já reproduzido (modo `approach` reprovado,
documentado no `nav2_params_arena.yaml`).

#### O critério não é "o anel", é o SETOR VARRIDO

Duas correções sobre a primeira versão:

1. **Retorno abaixo de 0,25 m também bloqueia.** Abaixo do raio inscrito não é
   "não varre" — é invasão do corpo. A primeira versão só olhava 0,25–0,354 e
   deixaria passar contato já em curso.
2. **Usar o anel inteiro gera falso bloqueio.** Algo a 0,30 m atrás não importa
   se o giro pedido varre só a frente. O critério tem que ser o **setor que o
   giro pedido realmente varre**.

A distância do centro à borda do quadrado na direção `α` é
`h(α) = 0,25 / max(|cos α|, |sin α|)`, e o ponto é atingido se
`∃ φ ∈ [0, Δθ] : ρ ≤ h(β−φ) + margem`.

⚠️ **A primeira versão mandava amostrar `φ` de 5 em 5° e chamava isso de "exato".
Não é.** Contraexemplo do review 10: obstáculo a **ρ = 0,35 m, β = 47,5°**, giro
de 5°. Nas pontas (φ=0 e φ=5°) `h` vale **0,339** e parece seguro; **no meio
(φ=2,5°, α=45°) `h` = 0,354 e bate.** Com passo de 5° faltariam ~1,5 cm de margem
só pra cobrir a discretização.

**Não precisa amostrar — dá pra resolver analiticamente.** `h` é máxima
(**0,354**) em `α ≡ 45° (mod 90°)` e mínima (**0,25**) em `α ≡ 0° (mod 90°)`.
Então, no intervalo `[β−Δθ, β]`:

```
se o intervalo contém algum 45° + k·90°  ->  max h = 0,354
senão                                    ->  max h = max(h(extremo1), h(extremo2))
```

Cobre os dois casos: `ρ < 0,25` é invasão (verdadeiro em qualquer φ) e o canto só
bloqueia quando o setor passa por 45°+k·90°.

⏳ **Faltava dizer (review 11):** o intervalo é **com sinal** — `Δθ < 0` inverte a
ordem — e precisa **desembrulhar o wrap de ±π** antes de procurar os 45°+k·90°.
Custo: **O(1) por feixe, O(n) pelo scan inteiro** (n ≈ 450 no LD06). Sem
amostragem de `φ`, mas não é O(1) no total.

#### ⚠️ A guarda não é um teste de entrada — é contínua, e olha até a FRENAGEM

Review 11, e é a correção mais importante do desenho. Duas coisas que eu tratava
como detalhe e são estruturais:

**1. Reavaliar a cada ciclo, não só ao iniciar o giro.** O skid desloca o centro,
o alvo do carrot muda, e obstáculo pode aparecer no meio da manobra.

**2. A margem tem que cobrir DINÂMICA, não ruído de scan.** Medido nesta volta
(1466 amostras girando):

| grandeza | mediana | p90 | max |
|---|---|---|---|
| taxa de yaw real | 25,0 °/s | 59,3 | 139,6 |
| giro **entre dois scans** (10 Hz) | 2,5° | **5,9°** | **14,0°** |
| deslocamento do centro girando (por 0,1 s) | — | **~1,6 cm** | — |

⛔ **AQUI EU ERREI FEIO (corrigido no review 12).** Eu escrevi que a frenagem
varre **17° a 2,4 rad/s e 58° a 4,5**, usando `max_angular_accel: 10.0`. Esse
parâmetro está **dentro do bloco do `RotationShim`** (`nav2_params_arena.yaml:189`,
logo abaixo de `rotate_to_heading_angular_vel`), ou seja, é do `controller_server`
— e o `path_follower` publica **`follow_vel` DIRETO no twist_mux (prio 15)**, sem
passar por ele. **Apliquei um limitador de outro nó do pipeline.** É a mesma
família do erro 46 ("o `collision_monitor` a jusante protege"): supor que um
componente atua onde ele não está.

**Medido, no lugar da conta teórica** — com duas correções do review 13 sobre a
minha primeira medição:

**Saída do modelo do atuador** (`sim_actuator_model.py`: `0,6·(|cmd|−1,7)`, satura
em 2,5, com assimetria direita):

| comando 2,4 rad/s | saída |
|---|---|
| esquerda | **0,420 rad/s = 24,1 °/s** |
| direita | **0,441 rad/s = 25,3 °/s** |

⚠️ Eu tinha escrito *"entrega do atuador: 17,3 °/s"*. **Rótulo errado:** 17,3 era
a taxa de yaw **alcançada** que eu medi da pose, não a saída do modelo. Refazendo
sem os 3 ticks de arranque: **18,0 °/s (0,314 rad/s)**. Então há **duas
grandezas**, e elas não batem: o modelo manda 0,42–0,441 e o robô entrega ~0,31.
A diferença é assunto à parte — o que não vale é chamar a segunda de "entrega do
atuador".

**Parada do giro** (zeradas vindas de `rot_min`):

| | n | mediana | p90 | max |
|---|---|---|---|---|
| todas | 85 | 1,00° | 1,30° | ~~7,00°~~ |
| **limpas** (giro NÃO recomeça em 0,15 s) | **74** | **1,00°** | **1,20°** | **1,80°** |

⚠️ **Os 7° eram contaminação minha:** em **11 das 85** o giro recomeça dentro da
janela — no caso dos 7°, o comando volta pra −4,5 rad/s **56 ms depois**. Eu media
"parada" onde havia inversão. Limpo: **máximo 1,80°**.

A guarda **continua sendo contínua e com look-ahead** — isso não muda. O tamanho,
com os números limpos:

```
Δθ_verificar = giro entre dois scans (até 14,0°)
             + parada                (até  1,8°)
             + FOLGA                 (parâmetro, NÃO zero)
```

⚠️ **A versão anterior escrevia `14 + 7 + folga ≈ 21`** — repare que 14+7 já dá
21, ou seja, **a folga sumiu da conta**. Ela é um parâmetro explícito, não um
enfeite na fórmula.

🔴 **E o mais importante: este número NÃO serve pro robô real.** O
`sim_actuator_model` é uma **curva estática** — zerou a entrada, zera a saída no
mesmo tick. Ele não modela inércia nenhuma. Logo os 1,8° medem o **simulador**,
não a frenagem física. Pro real, a margem tem que vir de **medição no robô** ou
de um **valor conservador configurável**. ⏳ pendente.

⛔ **A proposta de capar em `rot_min` CAI** — decisão do dono, review 12:

- **os dois contatos aconteceram com `wz` já em 2,4** (na janela do `cone_3`: 325
  amostras em 2,4). Capar em `rot_min` **não mudaria nada no caso que precisamos
  corrigir**;
- a justificativa dela era a conta dos 17°/58°, que acabou de cair;
- e ela adicionaria um terceiro comportamento, com limiares e chance de chatter,
  sem resolver a colisão reproduzida.

**Se depois da volta A/B aparecerem muitas paradas falsas**, aí sim: mede-se
**offline** quantas o cap evitaria, e ele entra como otimização separada.

#### Máquina de estados

| estado | condição de entrada | ação | saída |
|---|---|---|---|
| `LIVRE` | nenhum ponto atingido no setor pedido | gira normal | — |
| `INVERTE` | setor pedido bloqueado **e** arco oposto livre | gira pelo **arco complementar** | quando alinhar |
| `AFASTA` | os dois arcos bloqueados | translada pro lado que **aumenta a folga mínima** | folga > limiar + histerese, por N ciclos |

⏳ **Os dois ainda estão subdefinidos** (review 10), e isto é o que falta antes de
virar código:

- **`INVERTE`:** o outro caminho pro MESMO yaw é o **arco complementar**
  (`Δθ ∓ 2π`), **não** `−Δθ`. Girar `−Δθ` chega noutro lugar. O setor a verificar
  é o complementar inteiro, que é maior — logo bloqueia mais fácil.
- **`AFASTA`:** validar o **volume varrido pelo retângulo** na translação, não um
  "setor" — o corpo tem largura, e transladar varre uma faixa, não um raio.
  Faltam ainda: **velocidade**, **distância máxima**, **margem de frenagem** e
  **critério numérico de saída**.

| `PARADO_SEGURO` | nenhum sentido livre **e** nenhuma translação segura | publica zero **e emite estado** | intervenção |

⚠️ **`PARADO_SEGURO` NÃO é seguro enquanto o `unstuck` puder furá-lo sozinho.**
Eu tinha escrito isso como *feature* ("não é deadlock silencioso, o unstuck fura")
— é o contrário: recovery **automático** dirigindo justamente na situação que a
guarda bloqueou é o perigo. Conferido: o `unstuck_supervisor` assina
`motion_guard/state`, `door_zone`, `map`, `plan`, `scan`, `odom`, `nav_vel` — **e
nenhum estado de bloqueio do `path_follower`**. Ele só faz standdown para o
`motion_guard` e para a porta. Depois do timeout ele pode dar ré, avançar ou girar
por um canal (prio 30) que **ignora o collision monitor**.

**✅ DECIDIDO (dono, review 12): opção 1.** A tabela abaixo fica como registro do
porquê.

| | **1. `unstuck` RESPEITA o bloqueio** | **2. `unstuck` VIRA o `AFASTA`** |
|---|---|---|
| tamanho | **pequeno**: mais uma fonte de standdown, mesmo padrão já usado pro `motion_guard` e pra porta | **grande**: as manobras do unstuck passam a validar volume varrido |
| o que ganha | a guarda deixa de ser furada por recovery automático | uma coisa só sabe "como sair", sem lógica duplicada nem competindo |
| **o que perde** | **nessa situação o robô perde o resgate automático.** Se a guarda errar (bloqueio falso), ele fica parado até intervenção humana | nada de função — mas mexe num nó com história |
| risco | a guarda vira **ponto único de travamento** | o `unstuck` fura o collision monitor **de propósito** (prio 30); ensiná-lo a respeitar geometria muda o caráter dele e pode regredir os resgates que hoje funcionam |
| prazo (05/09) | **cabe** | não cabe com folga |

**Override humano consciente continua nas duas. Recovery automático independente,
não.**

**Minha recomendação:** **opção 1 agora**, opção 2 depois do prazo. Sem limite de
tempo na prova, "ficar parado esperando intervenção" é um custo aceitável; furar a
guarda e bater não é. E a opção 2 é reescrita de um nó validado em campo, o que
contraria o *"1 mudança pequena por vez"* a 8 dias da prova.
⏳ **Decisão do dono.**

**Histerese e ruído:** entra com 1 ciclo, **sai só depois de N ciclos** abaixo do
limiar — mesma lição do `turn_enter`/`turn_exit`, e agora também do latch de
chegada. Margem configurável sobre os 0,25/0,354 pra absorver ruído de scan.

**Scan velho ou ausente (> TTL): PARADA SEGURA.** ⚠️ Minha inclinação anterior
era *"permite e loga, porque o `collision_monitor` a jusante ainda protege o
avanço"*. **Raciocínio circular, e falso:** o `collision_monitor` bebe da MESMA
fonte (`source_timeout: 1.0`, `nav2_params_arena.yaml:633`). Scan mudo cega os
dois. Permitir giro ali reativa exatamente a colisão que a guarda existe pra
impedir. Para autonomia e para A4, **scan mudo → para**. Override humano
consciente segue podendo furar.

#### O teste: scan SINTÉTICO da geometria

⚠️ **A primeira versão prometia um teste que os dados arquivados não permitem.**
O `colisao_eventos.csv` tem pose verdadeira e folga; o `follow_debug.csv` tem só
`clear`, um escalar de folga frontal. **Nenhum dos dois tem o LaserScan 360°** que
a guarda consumiria.

Duas saídas, e escolho a primeira:

1. **Scan sintético calculado da geometria da arena** — o mundo é gerado por
   `tools/gera_arena_galpao.py` e o `colisao.py` já sabe ler caixas e cilindros,
   então dá pra fazer raycast exato a partir das poses gravadas. Determinístico,
   sem depender de gravação, e reusa o oráculo que já existe.
2. Arquivar `/scan_safe` nas próximas voltas — **obrigatório, não alternativa.**
   ⏳ pendente.

⚠️ **O scan sintético é teste UNITÁRIO da geometria, e só isso.** Ele não exercita
`/scan_safe` de verdade: não pega **discretização angular**, **sanitização**
(o filtro de 0,23 m), **atraso** nem **perda de mensagem**. O teste integrado
exige o scan gravado — por isso o item 2 é obrigatório.

O caso do teste está gravado: pose (12,16 · 6,99), yaw varrendo **−37° → −54°**,
`cone_3` a **0,51 m** do centro (superfície a 0,34 m), folga mergulhando a
**0,0 cm** em `colisao_eventos.csv`. A guarda tem que recusar o giro **antes** do
yaw em que a folga zera. Falha hoje por construção — não existe guarda nenhuma.

#### O que ela NÃO faz

- usa **scan ao vivo** (relativo), então **não depende do AMCL** — é por isso que
  é independente da questão do erro de pose. Pela mesma razão, **não vê o que o
  laser não vê**;
- **não** conserta a samba nem o erro de pose;
- **não fecha A4 sozinha.** A4 fecha com **volta completa e zero evento**,
  comparada ao baseline (236,4 s · 2 colisões + 28 raspões · 13 blocos de
  `goal_turn`).

#### ✂️ v1 ENXUTA (decidida no review 11, por causa do prazo)

`INVERTE` e `AFASTA` continuam subdefinidos e carregam risco alto. **Eles saem da
v1.** A primeira versão tem **dois estados**:

| estado | ação |
|---|---|
| `LIVRE` | gira normalmente |
| `PARADO_SEGURO` | **zera na hora**, publica bloqueio latched e **bloqueia o `unstuck`** |

Fechada no review 12, **sem cap e sem terceiro estado**:

- guarda **reavaliada continuamente**;
- **scan mudo → parada segura**;
- `unstuck`: **standdown + timer zerado + cauda de 2 s**.

Isso já ataca o modo de falha **observado** (canto varrendo o `cone_3`) e é uma
mudança pequena — o que o projeto manda fazer a 8 dias da prova. `INVERTE` e
`AFASTA` entram depois, sem bloquear a proteção básica.

#### Integração da opção 1 (não existe no código hoje)

Escolhida a opção 1, ela **não é só um `if`**. Precisa de quatro peças, e o
`motion_guard` já é o molde de todas:

1. o `path_follower` **publica um estado de bloqueio latched** (tópico novo);
2. o `unstuck_supervisor` **assina** esse estado (hoje ele assina
   `motion_guard/state` e `door_zone` — e **nenhum** estado do `path_follower`);
3. ao receber bloqueio: **standdown E zerar o timer de encalhe** — senão ele
   acumula "parado" durante o bloqueio e dispara no instante em que soltar;
4. **cauda ao desbloquear**, como já faz com o guard (`_guard_tail_until`, 2,0 s):
   soltar guarda e resgate no mesmo tick é convite pra thrash.

🔴 **O molde do `motion_guard` NÃO basta, e isso é um buraco real** (review 13). O
standdown dele é `if guard_blocked and self.state == _MONITORING` — e o próprio
comentário do código diz: *"Manobra JÁ em curso (state != _MONITORING) NÃO é
abortada"*. Foi decisão consciente lá (a parada era **pessoa**, e abortar ré no
meio perto de gente tem seus próprios BOs). **Aqui é o oposto:** o bloqueio existe
porque o corpo está prestes a raspar, e uma ré/avanço/giro já em curso é
exatamente o que precisa parar.

**Contrato do bloqueio de giro, então:** entra em standdown **e aborta manobra em
curso publicando zero**, em qualquer estado do unstuck — não só em `monitoring`.

⚠️ **Corrigido em 2026-08-31 (achado do dono, revisando o diário):** aqui estava
escrito *"⏳ desenho a fechar"* logo depois de enunciar o contrato — as duas
frases se contradiziam. O contrato **está fechado** (é a frase acima). O que não
existe é **código**: nem a publicação do bloqueio pelo `path_follower`, nem a
assinatura dele pelo `unstuck_supervisor`. É passo 4 da ordem da §2B.1, e o
estado certo é "decidido, não implementado".

#### ⏳ O que falta antes de virar código (lista honesta)

A versão anterior dizia que faltavam só duas coisas. **Era falso.** Falta:

| # | pendência | tipo |
|---|---|---|
| ~~1~~ | ~~`unstuck` respeita **ou** vira o `AFASTA`~~ | ✅ **DECIDIDO: respeita** (opção 1) |
| 1b | **Contrato do bloqueio**: abortar manobra JÁ EM CURSO, não só standdown | ✅ **decidido** (08-31) — falta **código**, nos dois nós |
| 2 | Arquivar `/scan_safe` (teste integrado) | tarefa |
| 3 | Valores: folga da margem, `N` ciclos de histerese, TTL do scan | calibração |
| ~~4~~ | ~~Capar o giro em `rot_min`~~ | **CAIU** (review 12) — os contatos já eram a 2,4 |
| ~~5~~ | ~~`INVERTE` / `AFASTA`~~ | **fora da v1** (review 11) |

**Decidido no review 10:** critério analítico (sem amostragem) e **scan mudo →
parada segura**.
**Decidido no review 11:** guarda **contínua** com look-ahead; v1 só `LIVRE` +
`PARADO_SEGURO`.
**Decidido no review 12 (dono):** **só bloquear na v1**, sem cap. Look-ahead
dimensionado pelo **medido** (~21°), não pela conta com o parâmetro errado.

---

## 2B. Sessão 2026-08-31

> Numerada `2B` de propósito: as seções §3–§6 são de **referência** e são citadas
> por número no corpo do texto (§2.8, §3.2, §4.5...). Renumerar pra encaixar uma
> sessão nova quebraria todas essas referências. Sessões futuras seguem 2C, 2D...

### 2B.1 O dono escolheu a ordem (e ela é o INVERSO da minha recomendação)

Na §2.9 eu recomendei **proteção de point-turn primeiro**, com o argumento de que
ela ataca o modo de falha observado. O dono decidiu o contrário, e o argumento
dele é melhor:

> *"Eu começaria pelo 2, latch de chegada: é uma correção menor, bem delimitada e
> elimina giros contraditórios que contaminam o diagnóstico. Depois repetiria o
> baseline; com a 'samba' removida, implementaria a proteção de point-turn sobre
> um comportamento mais estável."*

**Por que ele está certo e eu não estava:** os dois defeitos provados se
manifestam no MESMO lugar (giro no lugar, colado no `cone_3`) e no MESMO instante.
Enquanto a samba estiver viva, qualquer medição da proteção de point-turn mede as
duas coisas somadas — e eu já queimei uma sessão inteira (§2.9) tirando conclusão
binária de dois suspeitos sobrepostos. Tirar o **defeito menor e provado** primeiro
não é ordem de conveniência: é **desacoplar as variáveis** antes de medir a grande.

O ganho colateral que ele aponta: menos giro desnecessário colado no alvo = menos
exposição ao modo de falha do canto, então a proteção de point-turn depois é
implementada sobre uma planta mais estável.

**Ordem acordada:**

| # | passo | estado |
|---|---|---|
| 1 | teste que reproduz `goal_turn` ↔ `turning` | ⏳ |
| 2 | implementar latch/histerese da chegada | ⏳ |
| 3 | repetir a volta e medir contato no `cone_3` | ⏳ |
| 4 | fechar o contrato do bloqueio (aborto imediato do unstuck) | ⏳ |
| 5 | proteção contínua de point-turn | ⏳ |

⚠️ O passo 3 **não fecha A4 por si** — vale a mesma ressalva da §2.9: A4 fecha com
volta completa e zero evento. O passo 3 mede **quanto** da falha era samba.

### 2B.2 Desenho do latch (antes de virar código)

**Onde:** `path_follower.py:295`, o `if dist_goal <= c.goal_xy_tol:` pelado.

**O defeito, em uma frase:** o limiar de chegada arbitra entre dois controladores
que giram para **lados opostos** (`goal_turn` mira o yaw do goal; `turning` mira o
carrot), e o giro no lugar do skid **desloca** o robô o bastante pra cruzar o
limiar de volta. Sem histerese, isso é um oscilador — e a doença já foi curada uma
vez neste mesmo arquivo, pro limiar de HEADING (`turn_enter`/`turn_exit`, docstring
linhas 15-18). **O limiar de CHEGADA nunca recebeu o remédio.**

**Comportamento novo:** ao cruzar `goal_xy_tol` pela primeira vez naquele goal,
entra em fase de chegada e **não volta mais pro carrot** — só `goal_turn` até o yaw
fechar, depois `arrived`.

**O que solta a trava (as três saídas, explícitas):**

| gatilho | valor | por quê |
|---|---|---|
| goal inativo / sem plano | `goal_active=False` | já zera tudo pro `idle` hoje |
| **goal novo** | ponto final do plano move > `goal_moved_tol` | replan pro MESMO goal mantém o fim a poucos cm; goals da arena estão a metros |
| **empurrão** | `dist_goal > unlatch_dist` | unstuck/colisão tirou o robô de perto; insistir em `goal_turn` a meio metro é pior |

Valores propostos: `goal_moved_tol = 0.30 m` (2× a tolerância) e
`unlatch_dist = 3 × goal_xy_tol = 0.45 m` — o "~3× a tolerância" da §2.8.

**O que o latch NÃO faz** (mesma disciplina da §2.10): não conserta o erro de pose
do AMCL, não protege o canto no giro, e **não** garante que o contato no `cone_3`
some. Que ele elimine o contato é **hipótese** — o passo 3 é quem responde.

**Risco conhecido:** com a trava, um goal cujo yaw nunca fecha vira `goal_turn`
eterno. Hoje a samba disfarça isso saindo pro carrot. Mitigação: o gatilho de
empurrão. Se aparecer giro eterno parado no passo 3, é achado novo e entra aqui.

### 2B.3 Passos 1 e 2 ✅ — teste vermelho, depois latch (`path_follower.py`)

**Passo 1 — o teste reproduz o defeito.** `test_chegada_nao_alterna_para_turning_quando_o_giro_desloca`
monta a geometria exata do CSV: goal em (0,0), robô chegando por −x (carrot com
bearing 0) e `goal_yaw` a +90° — **lados opostos**, que é a condição que faz a
troca de estado inverter o giro. Três ticks: 0,166 m `driving` → 0,140 m
`goal_turn` → e o terceiro com o robô deslocado pra 0,161 m já com 34° rodados.

Vermelho antes do fix, com a mensagem certa:

```
E       AssertionError: saiu da chegada e voltou pro carrot
E       assert 'turning' == 'goal_turn'
```

Mais três testes fecham as saídas da trava (goal novo, empurrão, goal perdido) —
esses **passavam antes** por vacuidade; existem pra impedir que o latch vire
prisão.

**Passo 2 — o latch.** `dist_goal <= goal_xy_tol` deixou de ser o árbitro por
tick: cruzou uma vez, `_arrival_latched` fecha e a fase de chegada só devolve o
controle ao carrot por **goal novo** (fim do plano move > `goal_moved_tol` 0,30 m),
**empurrão** (`dist_goal > unlatch_dist` 0,45 m = 3× a tolerância) ou goal inativo.
Dois parâmetros ROS novos, com os mesmos nomes.

| | |
|---|---|
| testes do `path_follower` | 43 passam (39 antes + 4) |
| suíte do pacote | **397 passam** |
| linhas de lógica | ~10 (o resto é comentário e fiação de parâmetro) |

⚠️ **Isto não é evidência de campo.** É teste de unidade da lógica pura: prova que
a máquina de estados parou de alternar na geometria do CSV, **não** que o contato
no `cone_3` sumiu. Quem responde isso é o passo 3, contra o baseline (236,4 s ·
13 blocos de `goal_turn` (13,8 s) · 2 colisões + 28 raspões).

### 2B.4 Passo 3 — a volta com o latch (`log/sim_ab/arena_latch1/`)

Comando: o mesmo da §4.5, tag `arena_latch1`. ⚠️ Antes de rodar, `colcon build`:
o `install/` é egg-link pro `build/`, onde `path_follower.py` é **hardlink** do
fonte (mesmo inode). Deu certo por acaso — editei truncando no lugar, o que
preserva o inode. Ferramenta que **substitua** o arquivo (`sed -i`, editor que
escreve em temporário e renomeia) quebra o hardlink em silêncio e a volta roda
código velho **sem avisar**. Mesma família do BO do §2.2 ("o código que rodava
não era o do git").

#### Resultado, contra o baseline

| | baseline 08-28 | **latch 08-31** |
|---|---|---|
| goals | 5/5 | **5/5** |
| tempo total | 236,4 s | **222,8 s** (−5,7%) |
| **COLISÃO** | **2** | **0** |
| **raspão** | **28** | **0** |
| folga mínima | **−0,0000 m** (`cone_3`) | **+0,0741 m** (`A_fresta90_1`) |
| blocos de `goal_turn` | 13 (13,8 s) | 6 (9,6 s) |
| **`goal_turn`→`turning` no MESMO goal** | **7** | **0** |
| recoveries do Nav2 | 5 | 2 |
| unstuck (nosso) | nunca disparou | **2 disparos, 1,3 s** |

**O defeito que o latch atacava morreu:** 7 → 0 saídas da chegada de volta pro
carrot. E **zero contato** nesta volta, contra 2 colisões + 28 raspões — o
`cone_3`, que concentrava os 30 eventos, passou limpo.

⚠️ **Isso NÃO fecha A4, e não é taxa de sucesso.** É **uma** volta contra **uma**
volta (n=1 de cada lado). Este repo já tem a lição registrada de medir taxa e não
média/amostra única (2026-08-26, reverti uma correção certa por isso). A4 pede
volta completa com zero evento — e pede **repetição**.

⚠️ **Também não prova que o latch é a causa do zero contato.** A hipótese
continua hipótese: o §2.9 já registra que o `cone_3` tinha DOIS suspeitos (samba e
point-turn sem proteção) mais um co-suspeito (erro de pose). Tirei um; os outros
dois seguem lá e podem simplesmente não ter se alinhado nesta volta.

#### 🔴 ACHADO NOVO: com a samba morta, o robô PARA — e parar acorda o unstuck

O goal 4 custou **61 s** (os outros: 41/34/46/41) com **10,8 s parado**. O CSV
conta a história inteira, e ela **não** é o limiar de yaw:

| t (s) | o que aconteceu |
|---|---|
| 157,0 | trava a chegada, gira 116° → 6,2° de erro de yaw |
| 160,6 | `arrived`, publica zero — **e fica parado, correto** |
| ~165,4 | **`unstuck` dispara** (`reason=timeout`, `stuck_s=5,06`, `ang=4,2`) e **gira o robô ~17°** |
| 165,9 | erro de yaw estourou a tolerância → `goal_turn` **desfaz** o giro do unstuck |
| 167,0 | `arrived` de novo. Para. |
| ~173,3 | **`unstuck` dispara de novo** (`stuck_s=5,1`), mesma dança |
| 174,1 | goal 4 enfim completa; entra o goal 5 |

Confirmado com **verdade-terreno do Gazebo** (alinhamento por yaw, erro médio
1,17°): entre 165,4 e 165,9 o yaw do Gazebo vai de **182,5° a 199,2°** com x,y
parados — **o robô girou de verdade**, não foi salto de pose. E o `unstuck.csv`
mostra `nav_wants=1` nos dois disparos: o Nav2 ainda queria movimento.

**A causa raiz é outra, e é geométrica:** o follower travou a chegada com
`dist_goal = 0,147`, girou pro yaw do goal, e o giro do skid o deixou em
**0,166 m** — onde ele **para**. O `xy_goal_tolerance` do Nav2
(`nav2_params_arena.yaml:151`) é **0,15**. O robô estaciona **fora da tolerância
de quem julga a chegada**, a ação nunca completa, e 5 s parado é exatamente o
gatilho do `unstuck`.

Pior: pela verdade-terreno o robô estava a **27,7 cm** do goal (AMCL dizia 16,6) —
o erro de pose da §2.8 aparecendo de novo, agora decidindo se um goal completa.

**Antes o defeito existia igual, escondido:** a samba mantinha o robô se mexendo, e
o vaivém acabava caindo dentro dos 15 cm. Matar a samba **expôs** isso — é
regressão de sintoma no goal 4 (10,8 s parados), não regressão do latch.

#### 🧹 Furo de higiene no harness (achado pelo revisor, corrigido)

O `colisao.py` abre o subscriber de pose do Gazebo como **processo filho**. O
`kill_all.sh` mata o `colisao.py` com `kill -9` — o que **não** mata o filho — e o
padrão dele não casava com o binário real
(`.../gz_transport_vendor/libexec/gz/transport13/gz-transport-topic`): tinha
`gz sim` e `ruby.*gz`, nenhum dos dois pega. Resultado: o subscriber ficou
**17 min** vivo depois da volta, assinando a pose de um mundo que não existe mais.

Corrigido no padrão do `kill_all.sh` (`gz-transport-topic|gz topic`), com a
verificação de que o padrão **antigo não pegava** e o novo pega. Isso importa
além da limpeza: o comentário do próprio `kill_all.sh` registra que sobra de
processo **já contaminou uma medição inteira** (2026-08-27).

⏳ **Não corrigi.** As duas saídas óbvias (apertar o `goal_xy_tol` do follower pra
**abaixo** do checker do Nav2, ou continuar aproximando enquanto o Nav2 ainda quer
movimento) mudam comportamento de chegada e o dono decide. **Registrado, não
implementado.**

#### ✅ Decisões do dono (2026-08-31), sobre as duas perguntas em aberto

| pergunta | decisão |
|---|---|
| como consertar a chegada | **seguir aproximando**: enquanto o Nav2 ainda quiser movimento, o follower volta a avançar em direção ao **ponto do goal** (reto, sem carrot) e refecha o yaw. Descartado apertar o `goal_xy_tol` — é margem fixa contra uma deriva medida **uma** vez (0,02 m) |
| medir antes de seguir | **3 voltas agora**, com o latch **sozinho**, antes de mexer na chegada |

A ordem importa: as 3 voltas medem **o latch**, não o latch+chegada. Se eu
consertasse a chegada antes, nunca saberia o que o latch valia sozinho — e este
repo já registra (08-26) uma correção certa revertida por medir amostra única.

⚠️ Nota sobre a solução escolhida: ela reintroduz movimento perto do goal, que é
de onde veio a samba. **Não é a mesma coisa** — a samba era carrot × yaw-do-goal
brigando; aqui o avanço é dirigido **ao goal**. Mas é exatamente o tipo de coisa
que o `transicoes_goal_turn.csv` tem que reprovar depois: a checagem do conserto é
"saídas da chegada pro carrot continuam **0**".

**Isto reforça o passo 4 e muda o alvo dele:** o contrato do unstuck não é só
"não atrapalhar o point-turn" — é **não empurrar um robô que já chegou**. O
`unstuck` não tem como saber que o follower está em `arrived`, porque
[o supervisor não assina nenhum estado do `path_follower`](#) (§2.10, peça 2).
O mesmo tópico de bloqueio resolve os dois casos.


### 2B.5 Passo 3b — 3 voltas de repetição (`latchN1..3`). **Duas coisas que eu disse caem.**

Decisão do dono da §2B.4: medir o latch **sozinho** antes de mexer na chegada.
Comando: `tools/sim_ab/run_n.sh robot_nav latchN 3`, mesmas env vars da §4.5.

| volta | tempo | goals | COLISÃO | raspão | folga mín | samba | unstuck |
|---|---|---|---|---|---|---|---|
| baseline 08-28 | 236,4 s | 5/5 | **2** | **28** | −0,0000 | **7** | 0,0 s |
| `arena_latch1` | 222,8 s | 5/5 | 0 | 0 | 0,0741 | 0 | 1,3 s |
| `latchN1` | 219,8 s | **4/5** | 0 | 0 | 0,0620 | 0 | 2,9 s |
| `latchN2` | 251,2 s | 5/5 | 0 | 0 | 0,0749 | 0 | 5,8 s |
| `latchN3` | 254,7 s | 5/5 | 0 | 0 | 0,1014 | 0 | 6,2 s |

#### ✅ O que a repetição CONFIRMA

- **Samba: 0 em 4 de 4 voltas** (baseline 7). O defeito que o latch atacava está
  morto, agora com taxa e não com amostra.
- **Contato: 0 colisão e 0 raspão em 4 de 4** — contra 2 + 28 do baseline. A folga
  mínima ficou entre **6,2 e 10,1 cm**, sempre positiva.

#### ❌ O que a repetição DERRUBA (eu afirmei, e não se sustenta)

**1. "222,8 s contra 236,4 s: −5,7%".** Não se sustenta.

⚠️ **Corrigido no review do dono (08-31).** A primeira versão deste parágrafo
dizia *"com n=4 a média é 237,1 s ... o ganho era ruído"* — e tinha **um erro de
método dentro da correção**: os 237,1 s **incluíam a `latchN1`, que fez 4/5
goals**. Uma volta que perde um goal **não percorre o mesmo caminho**, então o
tempo dela não entra na mesma média (a `latchN1` perdeu justamente o goal 2, o
mais distante do trecho inicial).

**Só as voltas COMPLETAS (5/5):** 222,8 / 251,2 / 254,7 → média **242,9 s**,
contra **236,4 s** do baseline.

E o que dá pra afirmar é menos do que eu afirmei: **o baseline continua sendo
n=1**, então "era ruído" é conclusão estatística que estes dados não pagam. O
defensável é: **não há evidência de que o latch tenha acelerado a missão** — a
faixa das voltas completas (222,8–254,7) contém o baseline. O que o latch fez,
com taxa, foi parar de bater.

**2. "o goal 4 travou".** Não é o goal 4 — e também não é "um goal por volta",
como a primeira versão desta linha dizia (o revisor pegou): a `latchN3` travou em
**dois** (g3 com 6,3 s e g5 com 19,1 s).

| volta | onde travou |
|---|---|
| `arena_latch1` | g4 (10,8 s) |
| `latchN1` | g4 (16,4 s) |
| `latchN2` | **g5** (16,5 s) |
| `latchN3` | **g3** (6,3 s) **e g5** (19,1 s) |

O defeito de estacionar fora da tolerância (§6 item 2e) é **sistemático: aparece
nas 4 voltas**, em goals diferentes, e o pior caso por volta **cresce**
(10,8 → 19,1 s), como cresce o unstuck (1,3 → 6,2 s). Não é peculiaridade de um
ponto da rota.

#### 🔴 `latchN1` perdeu o goal 2 — falha direta do PLANNER, em modo já observado antes do latch

O probe registrou `CANCELADO em 25s`. **O rótulo estava errado** (ver abaixo): é
**ABORTADO** — o Nav2 desistiu. O `nav2.log` dá a causa, com 0,4 s entre as duas
linhas e 0,05 s até o probe registrar:

```
1788183683.62 [planner_server] GridBased plugin failed to plan
              from (7.90, 2.12) to (10.50, 1.76):
              "Either of the start or goal pose are an obstacle! "
1788183684.05 [controller_server] Could not find a legal trajectory:
              No valid trajectories out of 35!
1788183684.10 [ab_probe] goal 2: ABORTADO em 25s
```

É o **bug já aberto** (§6 itens 2d e 8), no **mesmo lugar da §2.8**: partindo de
dentro da **fresta A (0,90 m)**, indo pro goal 2. O que mudou é a gravidade —
antes aparecia como recovery, agora **custou um goal**. Taxa de goals com o latch:
**19 de 20**.

⚠️ **Não escrever "não é do latch"** (a primeira versão escrevia; o revisor
pegou). O que está demonstrado é o **mecanismo imediato**: o planner recusou o
par start/goal, num modo de falha **já observado antes do latch existir**. O que
**não** está demonstrado é que a pose em que o latch deixa o robô ao fim de cada
goal não influencia a chance de o próximo plano nascer com o start em obstáculo —
ninguém isolou isso. A formulação honesta é a do título: **falha direta do
planner, em modo já observado antes do latch**.

#### 🐞 BO do harness: o probe trocava ABORTADO por CANCELADO

`tools/sim_ab/probe.py` mapeava `{4:'OK', 5:'ABORTADO', 6:'CANCELADO'}`. O
`action_msgs/GoalStatus` do jazzy (conferido em runtime) é **4=SUCCEEDED,
5=CANCELED, 6=ABORTED** — invertido. E o probe **só** cancela no timeout, caso em
que o status vira `PRESO` sem passar por esse mapa: então, na prática, **todo
`CANCELADO` que esse harness já imprimiu era o Nav2 ABORTANDO**. A diferença não é
cosmética — "cancelado" se lê como *o harness desistiu*, "abortado" é *o robô
falhou*. Corrigido.

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
python3 tools/confere_evidencia.py --autoteste    # o conferidor de evidência
python3 tools/confere_evidencia.py                # docs/baselines/: git + CRLF + README
python3 tools/sim_ab/extrai_evidencia.py --autoteste  # critério da samba
python3 tools/sim_ab/colisao.py   --autoteste      # 7 casos de geometria
python3 -m pytest ros2_packages/robot_nav/test/ -q # 397 testes (08-31: +4 do latch)
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
| 26 | Contei **13 blocos de `goal_turn` "no goal 3"** | São 13 na volta **inteira**; o goal 3 tem **7 (8,5 s)**. E os "~35 s" eram o tamanho da janela que eu arquivei, não a duração da samba | Não confundir o recorte da análise com o fenômeno |
| 27 | Chamei `herr_deg` de **"yaw do robô"** | 115°/157°/177° são **erro** de yaw, não pose | Ler o nome da coluna |
| 28 | "Chegou a 0,04 m e derivou até 0,59 m" | Os 0,59 são de **antes** da chegada. Depois de entrar na tolerância: min 0,066, max 0,281 | Ancorar "antes/depois" num evento, não na janela inteira |
| 29 | **"A samba encostou no cone"** dito como conclusão | É hipótese: eu misturei pose do Gazebo (`colisao.csv`) com `dist_goal` do AMCL, e o AMCL erra **24 cm** ali. Co-suspeito não investigado | Não cruzar duas fontes de pose sem medir a diferença entre elas |
| 30 | Escrevi **"unstuck nunca disparou"** sem qualificar | Vale só pro NOSSO supervisor. O **Nav2** fez 5 recoveries, incluindo o bug que eu chamava de "anterior a esta fase" | Ler o log do nav2, não só os CSVs dos nossos nós |
| 57 | Chamei de **"entrega do atuador"** a taxa de yaw **alcançada** | O modelo manda **0,420–0,441 rad/s** (24–25 °/s) pra comando 2,4; os 17,3 (refeito: 18,0) eram a taxa medida da pose. São **duas** grandezas, e não batem | Rotular a medida pelo que ela é |
| 58 | Medi "parada" incluindo **11 casos em que o giro recomeça** | Num deles o comando volta pra −4,5 em **56 ms** — era dali que saía meu "7°". Limpo (74 casos): **max 1,80°** | Definir a janela pelo fenômeno, e excluir quem sai dele |
| 59 | `14 + 7 + folga ≈ 21` | 14+7 já é 21: **a folga sumiu da conta** | Se a parcela não muda o total, ela não está na conta |
| 60 | Usei o número do **sim** como margem de segurança | O `sim_actuator_model` é curva **estática**: zerou a entrada, zera a saída. Não modela inércia — os 1,8° medem o simulador, não a frenagem física | Margem de segurança do robô real se mede no robô real |
| 61 | Disse "copiar o molde do `motion_guard`" pro contrato do bloqueio | O standdown dele só vale em `monitoring`, e o comentário do código **diz explicitamente** que manobra em curso não é abortada. Aqui tem que abortar | Ler o que o molde faz antes de chamá-lo de molde |
| 55 | Calculei a frenagem com **`max_angular_accel`, que é do `RotationShim`** | O `path_follower` publica `follow_vel` **direto no twist_mux**, sem passar pelo `controller_server`. Medido: o giro para em **~1°**, não 17°. E o comando 2,4 rad/s entrega **0,30** — curva estática, não rampa | Mesma família do erro 46: supor que um componente atua onde ele não está. **Conferir o caminho do dado antes de usar o parâmetro** |
| 56 | Propus **capar em `rot_min`** como melhoria | Os dois contatos foram com `wz` **já em 2,4**: o cap não mudaria nada no caso a corrigir. E a justificativa dele era a conta que caiu | Otimização se prova contra o caso reproduzido, não contra uma conta |
| 52 | Desenhei a guarda como **teste de entrada** do giro | Tem que ser **contínua**: o skid desloca o centro, o carrot muda, obstáculo aparece no meio da manobra | Guarda que roda uma vez protege um instante, não uma manobra |
| 53 | Pensei a margem como **ruído de scan** | É **dinâmica**: o robô gira até **14°** entre dois scans, então o teste geométrico pode ser exato para uma pose já vencida. ⚠️ Este mesmo erro dizia "varre 17° parando de 2,4 e 58° de 4,5" — **números falsos, ver erro 55** | Margem de guarda se dimensiona pela dinâmica, não pelo sensor |
| 54 | Chamei o critério de **O(1)** | O(1) **por feixe**; O(n) pelo scan (~450 feixes). E faltava intervalo **com sinal** e **wrap de ±π** | Dizer a complexidade do que roda, não de um pedaço |
| 51 | Listei as duas saídas do `unstuck` **sem a consequência de cada uma** | O dono perguntou "anotou isso tudo?" pela terceira vez; o trade-off que eu descrevi no chat (pequena mas perde resgate × maior mas concentra num nó) não estava no arquivo | Opção sem consequência escrita não é decisão apresentada, é lista |
| 45 | Chamei de **"exato"** um critério **amostrado** | Com passo de 5°, um obstáculo a 0,35 m e 47,5° passa nas pontas (h=0,339) e bate no meio (h=0,354). Faltava ~1,5 cm só de margem de discretização — e nem precisava amostrar: `h` tem máximo analítico em 45°+k·90° | "Exato" é palavra que se prova, não se escreve |
| 46 | Ia **permitir giro com scan mudo**, alegando que o `collision_monitor` protege | **Circular:** ele bebe da mesma fonte (`source_timeout: 1.0`). Scan mudo cega os dois, e permitir ali reativa a colisão que a guarda existe pra impedir | Antes de contar com proteção a jusante, conferir se ela sobrevive à mesma falha |
| 47 | Vendi como **feature** o `unstuck` poder furar o `PARADO_SEGURO` | É o perigo: recovery **automático** dirigindo na situação que a guarda bloqueou. E ele **não assina** estado nenhum do `path_follower` — só faz standdown pra guard e porta | Distinguir override **humano** de recovery **automático** |
| 48 | `INVERTE` como `−Δθ` | O outro caminho pro mesmo yaw é o **arco complementar** (`Δθ ∓ 2π`); `−Δθ` chega noutro lugar | Ângulo tem topologia; não tratar como número |
| 49 | `AFASTA` validando um **"setor"** | O corpo tem largura: transladar varre um **volume**, não um raio | Usar a geometria do corpo, não a do sensor |
| 50 | Escrevi que faltavam **duas** coisas pra implementar | Faltam cinco, sendo uma decisão de segurança do dono | Não subdimensionar o que falta |
| 39 | Prometi um teste que **os dados não permitem** | A guarda consome LaserScan 360°; o arquivado só tem pose, folga e o escalar `clear`. Nunca arquivei `/scan_safe` | Antes de prometer teste, conferir se o dado de entrada dele existe |
| 40 | *"Recusar o giro, afastar ou girar pro lado livre"* como se fosse desenho | É **intenção**, não comportamento: faltava direção do afastamento, checagem da translação, histerese, saída, scan mudo, ruído e o caso "nenhum lado livre" | Se não dá pra escrever a tabela de estados, não é um plano |
| 41 | Critério só no **anel 0,25–0,354** | Retorno **abaixo de 0,25** é invasão do corpo e também tem que bloquear; e o anel inteiro gera falso bloqueio — o certo é o **setor varrido** pelo giro pedido | Definir a região pelo movimento pedido, não por um raio fixo |
| 42 | Tabela de erro por estado **enviesada** | `searchsorted` sem recortar o intervalo comum: **82 amostras** fora da janela do seguidor grudavam no primeiro/último estado. p90 do `goal_turn` saía 16,3 quando é **17,5** | Recortar ao intervalo comum antes de fatiar por estado |
| 43 | *"É a única das três que fecha A4"* | Ela ataca o **modo de falha observado**; quem fecha A4 é **volta completa com zero evento** — eu mesmo escrevi isso duas linhas depois | Não promover candidata a garantia |
| 44 | Deixei em §2.8 uma conclusão que a §2.9 já derrubou | *"Se o contato sobreviver ao latch, o suspeito é a localização"* ficou lá sem aviso | Ao derrubar conclusão, marcar **no lugar onde ela foi escrita** |
| 38 | Levei a conclusão pro chat e **deixei o plano de fora do arquivo** | O dono teve que perguntar "anotou?" pela **segunda vez**. A §2.9 tinha a ordem, mas o *como* do passo 1 (onde vai, o que testa, o que não resolve) só existia na conversa | A regra vale pro parágrafo inteiro que eu mando, não só pra parte tabelada |
| 32 | Atribuí à **spec** uma frase que é do `HANDOFF_PROVA_REAL.md:42` | Citei documento errado; a frase existe | Conferir a origem antes de citar |
| 33 | Liguei os **24 cm** direto à fresta e ao A2 | Erro euclidiano perto do cone ≠ erro lateral na fresta; e A2 é medido no **scan relativo**, não na pose absoluta | Não transportar uma métrica pra um contexto onde ela não é a métrica |
| 34 | Chamei `--com-cones` de **"uma variável"** | O mesmo mapa alimenta AMCL **e** costmap global | "Uma variável" é sobre o que o experimento MEXE, não sobre quantos flags eu passo |
| 35 | Avisei que os standoffs cairiam na **inflação dos cones** | **Medido falso**: 0,85–0,90 m contra inflação 0,60. E contradizia meu próprio `STANDOFF = 1,0` | Medir antes de avisar — inclusive contra o que eu mesmo já tinha calculado |
| 36 | *"Se sobreviver ao latch, o suspeito é a localização"* | Binário: sobra o point-turn sem proteção, que eu mesmo provei | Dois suspeitos provados não viram um por eliminação de um terceiro |
| 37 | Ignorei a hipótese mais sustentada pelo repo | O erro de pose **cresce no giro** (8,2→10,9 cm; p90 13,5→22,8) e `sim_robot.sdf` já documenta odom patinando 7,8° por tick no point-turn | Procurar no repo se o fenômeno já foi medido antes de inventar hipótese nova |
| 31 | A evidência versionada tinha **`colisao.log` fora do git** (`*.log`) e **CSV com CRLF** | O README listava um arquivo que sumia em checkout limpo; `git diff --check` reprovava toda linha | Conferir `git ls-files` e `git diff --check` do que se declara versionado |
| 25 | Reportei **"2 contatos"** olhando só o resumo do `colisao.log` | O CSV tinha **30 eventos**: 2 colisões + **28 raspões**. O resumo imprime só as colisões | Ler o CSV, não o resumo — foi a lição do "média esconde bimodal" outra vez |
| 24 | **Rodei a volta e não anotei** | O dono teve que perguntar "anotou isso tudo?". Pior: os CSVs de `controle_web/logs/` são sobrescritos a cada launch — quase perdi o `follow_debug` que sustenta o diagnóstico | Arquivar CSV e escrever o diário fazem parte da run, não são pós-jogo |
| 45 | **REINCIDÊNCIA do #31**: o README da evidência de 08-31 listava `probe.log`, que o `.gitignore:11` (`*.log`) mantém **fora do git** | Achado pelo revisor, não por mim. Os outros 6 arquivos estavam certos; esse existia só no meu checkout e sumiria num clone limpo — o revisor leria a linha e não acharia o arquivo. Virou `probe_volta.txt` | O #31 já tinha a lição escrita (**"conferir `git ls-files` do que se declara versionado"**) e eu li a seção inteira sem aplicá-la. Lição que não vira **passo executado** não protege nada: conferir `git ls-files` **do diretório** é parte de arquivar evidência, não uma boa intenção |
| 46 | Declarei **"sim parado, nada órfão"** com um `ps` que não cobria o que sobrou | Grepei `gz sim`, mas o subscriber de pose do `colisao.py` é um **processo filho** cujo binário é `.../transport13/gz-transport-topic`. Sobreviveu ao `kill -9` do pai e ficou **17 min** assinando a pose de um mundo morto. Achado pelo revisor. O `kill_all.sh` tinha o mesmo furo no padrão | Higiene de sim se confere com `ps` **amplo** (`grep -i gz`), não com a lista do que eu **espero** que esteja rodando. E `kill -9` no pai não mata o filho |
| 47 | Publiquei **"−5,7% de tempo"** com n=1 | Com 4 voltas: 219,8 / 222,8 / 251,2 / 254,7 — média **237,1** contra 236,4 do baseline. **Era ruído.** Eu tinha escrito a ressalva do n=1 duas linhas acima e mesmo assim pus o número na tabela | Ressalva **ao lado** de um número não impede o número de ser lido. Se n=1, o campo não recebe percentual — recebe "medido 1×" |
| 48 | Disse **"o goal 4 trava"**, e depois **"um goal por volta"** | As 4 voltas travam em goals **diferentes**: g4, g4, g5, e a `latchN3` em **DOIS** (g3 com 6,3 s **e** g5 com 19,1 s) — então nem "o goal 4", nem "um por volta". ⚠️ Esta linha ficou com o resumo errado (`g4,g4,g5,g5`) mesmo depois de eu corrigir a §2B.5; o revisor pegou **a linha do registro de erros** | Não batizar um defeito com o nome da primeira amostra. E corrigir um fato **em todos os lugares onde ele aparece** — inclusive na §5, que é justamente onde ele fica registrado |
| 49 | `probe.py` mapeava **`5:'ABORTADO', 6:'CANCELADO'`** — invertido | `GoalStatus` é 5=CANCELED, 6=ABORTED. Como o probe só cancela via timeout (→ `PRESO`), **todo "CANCELADO" já impresso por este harness era ABORT do Nav2**. "Cancelado" se lê como *o harness desistiu*; "abortado" é *o robô falhou* | Constante de mensagem ROS se confere no runtime, não de memória — foi o que fiz só depois de estranhar o rótulo |
| 50 | **REINCIDÊNCIA do #31 (a outra metade)**: os 4 CSVs de evidência que escrevi hoje saíram com **CRLF** | `csv.writer` termina linha com `\r\n` por **default**, e o `newline=''` que a doc do módulo manda usar **preserva** isso. `git diff --check` reprovava toda linha. Os CSVs do baseline (sessão anterior, pós-#31) estão LF; os meus, não. Achado pela **pergunta** do dono ("anotou tudo pro codex?"), não por mim | O #31 tinha DUAS metades (arquivo fora do git **e** CRLF). No BO #45 eu repeti a primeira; aqui repeti a segunda — **na mesma sessão em que escrevi que a lição vira passo executado**. Virou `tools/confere_evidencia.py`, com autoteste. Lição que eu preciso lembrar não é controle |
| 51 | Misturei uma volta **incompleta** na média de tempo | Os "237,1 s" incluíam a `latchN1`, que fez **4/5** goals — ela não percorre o mesmo caminho. Só as completas: **242,9 s**. Pior: o erro estava **dentro de uma correção** que eu escrevi justamente pra consertar rigor de amostra | Antes de tirar média, conferir que as amostras são da **mesma coisa**. Corrigir um viés não imuniza o parágrafo contra outro |
| 52 | Chamei de **"ponta-a-ponta"** um autoteste que só cobria a extração de nomes | O `--autoteste` do `confere_evidencia.py` testava o `citados()` e **nada** do `confere()` — nem arquivo ausente, nem fora do git, nem CRLF. A prova ponta-a-ponta que eu rodei foi **manual, no shell, e não ficou no repo**. É o BO 20 de novo | Se a prova não está no autoteste, ela não existe pro próximo. Agora são 7 casos que montam pasta de mentira e rodam o `confere()` |
| 53 | *"A perda do goal NÃO é do latch"* | Demonstrei o **mecanismo imediato** (planner recusou start/goal), não a **independência**: ninguém isolou se a pose onde o latch para o robô afeta o start do plano seguinte | Mecanismo observado ≠ causa isolada. Escrever "falha direta do X, em modo já observado antes de Y" |
| 54 | O gerador de evidência **truncava o destino antes de validar a entrada** | Ele abria `colisao_3voltas.csv` em `'w'` e só falhava depois, dentro do `_ler()`. Num clone limpo, o comando que **o próprio README manda rodar** apagava a evidência boa pra descobrir que não podia gerar a nova; falha no meio deixava extratos pela metade. Eu tinha escrito no docstring que ele "falha sem inventar dado" — não inventava, **apagava**. Achado pelo revisor | Validar TUDO antes de abrir qualquer destino; gerar em temporário e trocar com `os.replace` só no fim. E: uma promessa no docstring que nenhum teste cobre é decoração |
| 55 | O `finally` de limpeza **não pegava o temporário do gerador que falha** | O `tmps.append()` vinha **depois** do `fn(tmp, tags)`, então o arquivo que explodia no meio vazava na pasta. ⚠️ Quem pegou foi **o autoteste que eu tinha acabado de escrever** pro BO 54 — na primeira execução | Registro o acerto junto: o teste do BO anterior pegou o BO seguinte. É o argumento inteiro a favor de teste em vez de lição escrita |
| 22 | No teste novo do parser, **minha expectativa estava errada** | Escrevi (2,0 · 1,0) — que é o resultado de **ignorar** o yaw. O teste teria passado na versão com bug | Ao testar rotação, afirmar também o valor que o bug produziria |

---

## 6. Aberto

| # | item | estado |
|---|---|---|
| 1 | **Proteção de point-turn** (anel 0,25–0,36 m no `path_follower`) | 🔴 bloqueador de A4, **desenho fechado** (§2.10), zero código. **Não é** no collision monitor: lá a única alavanca é escalar o `wz`, que trava o robô (deadlock já reproduzido) |
| 2 | Baseline Nav2 até os standoffs | ✅ **FEITO 08-28** — 5/5 goals, 236,4 s, 2 colisões + 28 raspões no cone (§2.8). Evidência em `docs/baselines/2026-08-28-arena-baseline1/` |
| 2b | Travar a chegada (mata a samba — defeito provado) | ✅ **FEITO 08-31** (`c85a8d8`) — saídas da chegada pro carrot **7 → 0**; volta em `docs/baselines/2026-08-31-arena-latch1/` (§2B.4) |
| 2e | 🔴 **O robô ESTACIONA fora do `xy_goal_tolerance` do Nav2** | ⏳ **novo, 08-31. SISTEMÁTICO: 4 de 4 voltas** (§2B.5), em goals diferentes, com o tempo parado crescendo 10,8 → 19,1 s.| Trava a chegada a 0,147, gira pro yaw do goal, para em **0,166** — o checker do Nav2 é **0,15** (`nav2_params_arena.yaml:151`). A ação não completa, 5 s parado acorda o `unstuck`, que gira o robô 17°, e o seguidor desfaz. **Custou 14 s no goal 4.** Existia antes, escondido pela samba. Duas saídas propostas na §2B.4, **nenhuma decidida** |
| 2f | **`unstuck` empurra robô que JÁ CHEGOU** | ⏳ **novo, 08-31.** Caso irmão do item 1: o supervisor não assina estado nenhum do `path_follower`, então não sabe distinguir "encalhado" de "chegou e parou". O mesmo tópico de bloqueio do passo 4 resolve os dois |
| 2c | 🔴🔴 **AMCL erra 24 cm na arena** (mediana 9, p90 16, max 27) | ⏳ **novo, 08-28.** Maior que a tolerância de A2 (20 cm) e muito maior que os ±3 cm da fresta de 0,60. Suspeita: arena pobre em feature + os 4 cones não estão no mapa |
| 2d | Bug `start/goal is an obstacle` **reproduzido nesta arena** | ⏳ disparou dentro da fresta A (0,90 m), indo pro goal 2 |
| 3 | Executor que não pula ponto após falha | ⏳ |
| 4 | Aproximação final ao cone (A2) | ⏳ o `PolygonFront` bloqueia o avanço a ~0,67 m do centro do cone, **antes** dos 20 cm |
| 5 | LED/relé | ⏳ interface já existe: `/light/marker` (pino 8) e `/light/cmd` (pino 7) no `mega_bridge` |
| 6 | Medir no robô: bitola, entre-eixos, altura do LiDAR | ⏳ trena |
| 7 | Refazer os números de 08-27 | ⏳ os antigos vieram do fork **sem `motion_guard`** e com oráculo cego a 5 obstáculos |
| 8 | Bug `"start/goal is an obstacle"` | 🔴 **AGRAVADO 08-31**: em `latchN1` ele **custou o goal 2** (Nav2 abortou), no mesmo ponto da §2.8 — partindo de dentro da fresta A. Saiu de "recovery" para "goal perdido". É a **única** perda em 20 goals com o latch (§2B.5) |
