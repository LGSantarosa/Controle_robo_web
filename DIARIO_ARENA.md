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
| A4 | **zero contato** com bloco, cone ou parede | 🔴 **NÃO fechado, e a causa está identificada.** Baseline 08-28: 2 colisões + 28 raspões. Com o latch, **7 voltas: 6 limpas, 1 com 4 raspões** (`aprox2`, folga 8 mm) — e os 4 estão em `turning` a **6,15 m do goal**, ou seja **point-turn de rota sem guarda** (§2B.6), não chegada. Fecha com o passo 5 + volta completa sem evento |
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
| 1 | teste que reproduz `goal_turn` ↔ `turning` | ✅ `c85a8d8` (§2B.3) |
| 2 | implementar latch/histerese da chegada | ✅ `c85a8d8` — **samba 8 → 0, em 7 voltas** |
| 3 | repetir a volta e medir contato no `cone_3` | ✅ §2B.4 (1 volta) + §2B.5 (3 voltas) |
| 2b | *(surgiu no caminho)* aproximação final — o robô parava fora da tolerância do Nav2 | 🟡 `4da6eb4` (§2B.6): goals completam a **3–9 cm** (era 11–15), mas com churn de mira. **Item 2e não fechado** |
| 4 | fechar o contrato do bloqueio (aborto imediato do unstuck) | ⏳ — menos urgente: a interferência do unstuck caiu pra 0,0–3,0 s com a aproximação |
| 5 | proteção contínua de point-turn | ⏳ 🔴 **virou o bloqueador medido de A4** (§2B.6: 4 raspões no `cone_3` a 6,15 m do goal) |

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
| **saídas da chegada pro carrot** (samba) | **8** | **0** |
| recoveries do Nav2 | 5 | 2 |
| unstuck (nosso) | nunca disparou | **2 disparos, 1,3 s** |

**O defeito que o latch atacava morreu:** 8 → 0 saídas da chegada de volta pro
carrot.

⚠️ **Era "7 → 0" até 08-31 (review do dono).** O critério original só contava
`goal_turn → turning`; quando a aproximação criou o `goal_approach` eu ampliei o
`conta_samba()` pra `{goal_approach, goal_turn} → {turning, driving}` — e o
baseline passou a somar **8**, porque tem uma saída `goal_turn → driving` a
`dist_goal 0,15` (t=122,2 s, dentro da janela da samba) que a métrica estreita
não via. **O número certo do baseline é 8**, e o diário ficou três commits
dizendo 7 enquanto a ferramenta dizia 8. E **zero contato** nesta volta, contra 2 colisões + 28 raspões — o
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

**A geometria, medida:** o follower travou a chegada com `dist_goal = 0,147`,
girou pro yaw do goal, e o giro do skid o deixou em **0,166 m** — onde ele
**para**. O `xy_goal_tolerance` do Nav2 (`nav2_params_arena.yaml:151`) é **0,15**.

🔴 **RETRATAÇÃO (review do dono, 08-31).** Eu escrevi aqui que *"o robô estaciona
fora da tolerância de quem julga a chegada, **a ação nunca completa**"* — e isso
**não está demonstrado**. O mesmo bloco do yaml tem **`stateful: true`**
(linha 153): o `SimpleGoalChecker` do Nav2, uma vez satisfeito o XY, **para de
reconferir XY** e passa a olhar só o yaw. Como o robô ENTROU a 0,147 (dentro dos
0,15), o XY provavelmente já estava travado — então "parou a 0,166, logo o Nav2
recusa" é **inferência minha, não medição**.

**O que continua medido e de pé:** o robô ficou parado em `arrived`, o
`unstuck.csv` mostra `nav_wants=1` (o Nav2 ainda queria movimento) e o resgate
disparou por `timeout` aos ~5 s. **O que não sei é POR QUE o Nav2 ainda queria
movimento.** Hipótese alternativa que o próprio log sugere: os
`Failed to make progress` (3 no goal da samba) disparam recovery, e recovery
**reseta** o goal checker — o que zeraria o latch do XY e voltaria a exigir os
0,15. ⏳ Não investigado; item 2i.

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

- **Samba: 0 em 4 de 4 voltas** (baseline 8). O defeito que o latch atacava está
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

### 2B.6 Passo 2b — aproximação final (`4da6eb4`). ⏳ 3 voltas rodando

Executa a decisão da §2B.4: **enquanto o Nav2 ainda quiser movimento, aproximar**.

**Onde:** dentro do bloco do latch, **antes** do yaw. A ordem é o desenho:
**posição primeiro, yaw depois**. Entrou fora do que o Nav2 aceita → volta a
avançar pro **ponto do goal**, reto, **sem carrot** — o carrot é justamente quem
brigava com o yaw do goal e produzia a samba.

| knob | valor | por quê |
|---|---|---|
| `approach_enter` | 0,10 m | acima disso volta a aproximar |
| `approach_exit` | 0,06 m | abaixo disso para. **Dois limiares diferentes de propósito**: foi um limiar pelado que criou a samba |
| folga até o Nav2 | 0,15 − 0,06 = **9 cm** | cabe a deriva de ~2 cm do giro seguinte |

**Goal pra trás:** avançar afastaria. Gira no lugar pra encarar o goal — este robô
não faz arco (`arc_calib`), então é point-turn ou nada. ⚠️ Isso **soma giro perto
do goal**, que é onde o canto varre; é mais um motivo pra guarda de point-turn
(passo 5) e um item a olhar nos CSVs destas voltas.

#### Os 5 testes, e o que quase passou vazio

O que protege a correção é o **microsim de laço fechado**: aproxima, fecha o yaw
(que **desloca** o robô, como o skid faz), e exige terminar em `arrived`
**dentro dos 0,15 do Nav2**, sem nunca voltar pro carrot.

⚠️ **A primeira versão dele passava sem a correção.** Eu modelei a deriva do
point-turn empurrando o robô **em direção** ao goal; o medido é o contrário
(0,147 → 0,166, **afastando**). Com o sinal certo ele falha com a mensagem
exata — *"parou fora do checker do Nav2"*. **Teste que não falha não prova nada**,
e este passou perto de ir pro repo como prova de coisa nenhuma (BO 56).

Também errei a geometria de um teste (BO 22 de novo): afirmei `vx > 0` numa pose
em que o goal está a **90° do nariz** — ali girar pra encarar é o certo, e o
código estava certo. Trocado por uma pose com o goal à frente.

**402 testes passam** (397 + 5).

#### O que esta correção NÃO faz

- **não** conserta o `unstuck` empurrando quem chegou (§6 item 2f): ela remove a
  *causa* mais provável (ficar 5 s parado fora da tolerância), não o mecanismo. O
  contrato do passo 4 continua de pé;
- **não** protege o canto no giro (passo 5) — e ainda **acrescenta** giro perto do
  goal no caso "goal pra trás";
- **não** mexe no erro de pose do AMCL, que é quem faz o `dist_goal` do seguidor
  discordar da verdade-terreno (16,6 contra 27,7 cm, §2B.4).

#### 📊 As 3 voltas (`aprox1..3`) — resultado MISTO, item 2e **não fechado**

| volta | tempo | goals | COLISÃO | raspão | folga mín | samba | unstuck | parado |
|---|---|---|---|---|---|---|---|---|
| `latchN2` (antes) | 251,2 | 5/5 | 0 | 0 | 0,0749 | 0 | 5,8 s | 16,5 s |
| `latchN3` (antes) | 254,7 | 5/5 | 0 | 0 | 0,1014 | 0 | 6,2 s | 25,4 s |
| **`aprox1`** | 250,4 | 5/5 | 0 | 0 | 0,0483 | 0 | **0,0 s** | **0,1 s** |
| **`aprox2`** | **328,6** | 5/5 | 0 | **4** | **0,0083** | 0 | 3,0 s | **56,6 s** |
| **`aprox3`** | 254,3 | 5/5 | 0 | 0 | 0,0893 | 0 | 1,4 s | 15,4 s |

**✅ A correção FAZ o que foi desenhada pra fazer.** A medida é a distância do goal
no **último tick antes de o plano apontar pro goal seguinte**:

⚠️ **Duas ressalvas do review, ambas procedentes.** (a) Isso **não é** a pose no
instante exato da conclusão: a 20 Hz e 0,22 m/s cabem **~1,1 cm entre amostras**,
então um valor de 0,156 pode ter cruzado os 0,15 entre ticks. (b) Com
`stateful: true` **não é verdade em geral** que "a distância no fim é o que o
checker julgou" — ele pode ter travado o XY bem antes. O que a tabela mede
honestamente é **onde o robô estava quando o goal acabou**, que é o número que
importa pro defeito, com resolução de ~1 cm:

| | distâncias finais, por goal | pior |
|---|---|---|
| `arena_latch1` | 0,135 · 0,153 · 0,143 · 0,144 · 0,131 | 0,153 |
| `latchN2` | 0,144 · 0,131 · 0,148 · 0,122 · 0,109 | 0,148 |
| `latchN3` | 0,145 · 0,144 · **0,365** · 0,128 · 0,112 | 0,365 |
| **`aprox1`** | **0,040** · 0,156 · **0,038** · **0,042** · **0,034** | 0,156 |
| **`aprox2`** | 0,060 · 0,160 · 0,124 · 0,065 · 0,092 | 0,160 |
| **`aprox3`** | 0,145 · 0,160 · 0,065 · 0,043 · 0,069 | 0,160 |

**Ele chega a 3–9 cm onde antes parava a 11–15.** E a interferência do `unstuck`
caiu (0,0 / 3,0 / 1,4 s contra 1,3 a 6,2). ⚠️ Sobra um teimoso: **o goal 2 termina
a 0,156–0,160 nas TRÊS voltas** — o único que a aproximação não puxa pra dentro,
e sempre o mesmo. **Não é prova de que ele termina fora da tolerância** (ver as
duas ressalvas acima: ~1 cm de resolução e o checker stateful). Não investigado;
item 2h.

**❌ Mas o item 2e não fechou, e um critério meu estava mal escolhido.**

O `parado` do probe é **deslocamento nulo** — e **point-turn conta como parado**.
A aproximação ADICIONA giro (aim-turn pro ponto do goal), então esse número mede
duas coisas juntas. Na `aprox2` foram 780 ticks de `goal_approach` + 310 de
`goal_turn` ≈ 54 s de giro perto do goal, e só **2 janelas de 3,7 s** de robô
genuinamente parado. Não é encalhe: é **churn** — mira, anda, fecha o yaw, a
deriva tira, re-aproxima. Bounded (3 re-entradas), mas caro: 328,6 s de volta.

Contagem de re-entradas (`goal_turn → goal_approach`): **1 / 3 / 2**. Convergiu
sempre; nunca virou laço infinito.

**A `aprox3` parou 16 s em `idle`** — sem goal ativo — entre dois goals. Isso não
é o seguidor: é buraco entre a conclusão de um goal e o próximo ser aceito. Item
novo (§6, 2g), não investigado.

#### 🔴 Os 4 raspões da `aprox2`: contato durante point-turn de ROTA, modo já observado antes

Alinhando os CSVs (offset +6,8 s), os quatro eventos estão em:

```
t=116.84  raspao  cone_3  folga 0.0190 | state=turning  dist_goal=6.148  wz=-4.11
t=116.90  raspao  cone_3  folga 0.0106 | state=turning  dist_goal=6.149  wz=-3.78
t=116.95  raspao  cone_3  folga 0.0083 | state=turning  dist_goal=6.149  wz=-3.63
t=117.00  raspao  cone_3  folga 0.0120 | state=turning  dist_goal=6.150  wz=-3.36
```

`dist_goal = 6,15 m`: **longe de qualquer chegada**. É o carrot girando no meio do
caminho e o **canto varrendo o `cone_3`** — o item 1 dos abertos, o mesmo
mecanismo provado na §2.9.

⚠️ **Não escrever "a aproximação não tem parte nisso"** (a primeira versão
escrevia; é o BO 53 outra vez). O demonstrado é o **mecanismo imediato**: o
contato aconteceu **durante um point-turn de rota, modo de falha já observado
antes da aproximação existir**. O que **não** foi isolado é se a aproximação do
goal anterior mudou pose ou rumo de um jeito que tornou aquele giro mais
provável.

⚠️ **E isso derruba a leitura otimista do "zero contato em 4/4"**: com 3 voltas a
mais, apareceu 1 volta com contato. **7 voltas com latch: 6 limpas, 1 com 4
raspões.** O ponto não é a taxa — é que a causa **nunca foi corrigida**, só não
tinha se alinhado. **A guarda de point-turn (passo 5) é o que falta pra A4**, e
isso agora está medido, não suposto.

#### 🐞 Meu critério de samba estava CEGO pro estado novo

Escrevi o aceite como *"`goal_turn → turning` continua 0"* — e criei o
`goal_approach` no mesmo commit. Uma samba pela porta nova não seria contada. As
3 voltas **não** tiveram nenhuma (as 2 saídas de `goal_approach` foram com
`dist_goal > 6 m` = goal novo), mas **a métrica estava cega, o que é diferente de
estar certa**. `conta_samba()` agora cobre `{goal_approach, goal_turn} →
{turning, driving}`, com 5 casos de autoteste.

### 2B.7 3 voltas com a histerese (`hist1..3`) — e o achado que muda o quadro

Rodadas depois de `6f707a3`, porque as `aprox1..3` validavam `4da6eb4` (o revisor
apontou; o README delas já diz isso).

| volta | tempo | goals | COLISÃO | raspão | folga mín | samba | unstuck | parado |
|---|---|---|---|---|---|---|---|---|
| `aprox1` | 250,4 | 5/5 | 0 | 0 | 0,0483 | 0 | 0,0 | 0,1 |
| `aprox2` | 328,6 | 5/5 | 0 | 4 | 0,0083 | 0 | 3,0 | 56,6 |
| `aprox3` | 254,3 | 5/5 | 0 | 0 | 0,0893 | 0 | 1,4 | 15,4 |
| **`hist1`** | **230,3** | 5/5 | 0 | 0 | 0,0776 | 0 | 0,0 | **1,0** |
| **`hist2`** | 269,1 | 5/5 | 0 | 0 | 0,0630 | 0 | 1,5 | 26,7 |
| **`hist3`** | 265,6 | 5/5 | 0 | 0 | 0,0451 | 0 | 3,2 | 30,7 |

**✅ A histerese fez o que foi desenhada pra fazer, na métrica dela.** Alternâncias
mira↔avanço dentro de `goal_approach` (a mesma conta do teste unitário):

| | `aprox` | `hist` |
|---|---|---|
| alternâncias | 28 / 90 / 37 | **7 / 17 / 11** |
| ticks em `goal_approach` | 379 / 780 / 401 | 264 / 498 / 849 |

Corte de **3 a 5×**. Contato **0 em 3/3**, samba 0.

**❌ Mas o tempo parado NÃO caiu** (26,7 e 30,7 s). Então o chattering da mira
**não era** a causa das paradas longas — eu tinha atribuído a ele.

#### 🔴🔴 A causa dos piores casos: o `motion_guard` bloqueia o robô na arena

Na `hist3`, 26 s com o seguidor mandando `wz = 2,4` e a **pose congelada em
(11.961, 6.520)**, `herr` travado em 31,0°. O `freeze_capture` mostra o pipeline:

```
t=116.0  follow_vel    wz=2.4000     <- o seguidor manda girar
t=116.0  auto_vel_pre  wz=2.4000     <- passou pelo twist_mux
t=116.0  auto_vel_raw  wz=0.0000     <- ZERADO aqui
```

O estágio entre `auto_vel_pre` e `auto_vel_raw` é o **`motion_guard`**
(`nav2_params_arena.yaml:616` documenta a cadeia). E o estado dele:

```
t=  0.0  guard_state: idle
t=115.7  guard_state: blocked      <- 0,1 s depois de a aproximação começar
t=142.6  guard_state: idle
```

**26,9 s bloqueado.** Na janela, **505** comandos entraram no `motion_guard`
(`auto_vel_pre`, todos `wz = 2,4`) e **1** saiu. A pose do seguidor andou
**1,7 cm** em 26,9 s; `herr` foi de 29,5° a 31,0° — ou seja, **piorou**.

| volta | tempo com `guard` BLOCKED |
|---|---|
| baseline, `arena_latch1`, `latchN1..3`, `aprox1`, `aprox3`, `hist1` | **0,0 s** |
| `hist2` | 0,1 s |
| **`hist3`** | **26,9 s** (1 episódio) |
| **`aprox2`** | **52,1 s** (2 episódios) |

(O extrato `guard_bloqueio_11voltas.csv` lista episódios **≥ 1 s** — o de 0,1 s
da `hist2` fica de fora de propósito: é transitório, não parada.)

**O `motion_guard` é o vigia de PESSOA** — a §2.7 do `project_motion_guard`
descreve: detector de movimento semeia "gente aqui", vulto tamanho-de-gente
carrega parada. **A arena não tem pessoa nenhuma: tem CONE.** E ele só disparou
nas voltas com aproximação, que é o que adicionou point-turn perto de cone.

#### A vigília fechou em cima de um CONE — isto agora está MEDIDO

Os 3 episódios de bloqueio das 11 voltas têm a **mesma assinatura de tempo** e o
**mesmo tipo de vizinho** (pose e folga do ground truth do Gazebo, `colisao.csv`):

| volta | episódio | duração | comandos in→out | objeto mais próximo | folga |
|---|---|---|---|---|---|
| `hist3` | 120,1 → 147,0 | **26,9 s** | 505 → **1** | `cone_3` | 0,312 m |
| `aprox2` | 120,3 → 146,5 | **26,2 s** | 502 → **1** | `cone_3` | 0,115 m |
| `aprox2` | 201,0 → 226,7 | **25,7 s** | 505 → **1** | `cone_4` | 0,405 m |

(A pose é a do **ground truth** do Gazebo e fica **idêntica** o episódio inteiro
— 521 amostras iguais na `hist3`. Na `aprox2` o comando zerado era `vx = 0,30`
reto, não giro: o guard zera os dois.)

Duas coisas fecham o caso:

1. **A duração é a soma dos tetos do próprio vigia:** `hold_still_max` **20 s**
   (vigília segurando presença **parada**) + `clear_time` **5 s** (decaimento) +
   `settle` (≤ 4 s) → 25–29 s. Os três episódios caem em **25,7–26,9 s**. Não é
   flicker de detecção: é a vigília **rodando até o teto** em cima de algo que
   não sai do lugar.
2. **Não há o que confundir com gente:** `worlds/arena_galpao.sdf` tem 23 modelos
   e **nenhum `<actor>`** — muros, frestas, plataformas e 4 cones, tudo estático.
   Nos três episódios o único objeto ao alcance era um cone.

O elo que **continua sem log**: o centróide da vigília (`_watch`) não é
publicado, então não posso exibir a coordenada vigiada. O que posso exibir é que
o vizinho era cone em **3/3** e que pessoa não existe nesse mundo.

**Por que só nas voltas com aproximação** (leitura de código
`motion_guard.py:265-275`, ⚠️ não medida): o `wz_gate` faz o `observe()` **não
avaliar nem snapshotar** enquanto o robô gira; a referência "old" logo após o
point-turn é o último snapshot **pré-giro**, de outra pose. E a aproximação é
exatamente o que passou a fazer point-turn a ~0,3 m de um cone.

**Isto explica os piores casos, não todos.** A mesma extração acha travamentos
de 11–17 s **sem guard nenhum**: `hist2` (14,9 s e 11,3 s), `aprox3` (14,9 s),
`latchN3` (17,1 s). Ou seja, há pelo menos **duas** causas de parada longa — o
guard é a nova e a maior. ⚠️ Detalhe que eu não sei explicar e não investiguei:
**três dos quatro** travamentos sem guard também estão parados ao lado do
`cone_4` (folga 0,30–0,48 m).

**Contexto que eu devia ter olhado antes:** o item 7 dos abertos diz que os
números de 08-27 vieram do fork **sem `motion_guard`** — a §2.2 unificou os
pacotes e o guard voltou pro caminho. Um nó feito pra não atropelar gente está
ligado numa prova onde não há gente, e ele **zera o giro**, que é exatamente o
comando de que o robô precisa pra sair.

### 2B.8 Guard DESLIGADO na arena — o que mudou no código

**Decisão do dono (08-31), depois da §2B.7:** desligar o `motion_guard` na
arena. Ele é o vigia de PESSOA; a prova não tem pessoa.

⚠️ **Não dá pra só não lançar o nó.** O guard é um estágio da artéria
(`auto_vel_pre` → `auto_vel_raw`): tirando ele, o `collision_monitor` fica **sem
publisher na entrada** e a autonomia inteira emudece. Então a mudança tem duas
metades, e as duas moram no `nav2.launch.py`:

| | o que faz |
|---|---|
| `motion_guard:=false` | o nó **não sobe** (`condition=IfCondition`) |
| `auto_mux_out` | o `twist_mux_auto` publica **direto** em `auto_vel_raw` |

Default é `true` — fora da arena **nada muda**, o guard segue ligado no robô de
sempre.

🔴 **Mas `--arena` também vale no ROBÔ REAL** (achado do review): tudo o que
mediu isto foi **sim**, e a ausência de `<actor>` prova só que o **mundo
simulado** não tem gente. `./launch.sh --nav2 --arena` **sem** `--sim` sobe o
robô físico **sem o vigia de pessoa**. Só é aceitável com **pista controlada,
gente fora da área e E-STOP humano na mão**. O `collision_monitor` segue ligado,
mas ele é reflexo **geométrico** de obstáculo — não substitui vigia de coisa que
se **move**. O `launch.sh` agora **avisa na tela** quando é real + arena. Quem desliga é o `--arena` do `launch.sh` (junto do
`follow_clear_full:=1.2`, mesmo padrão), e o harness do sim pelo
`AB_EXTRA_LAUNCH` (a §4.5 abaixo já traz o comando atualizado).

**Uma ferramenta minha ia passar a mentir.** O `pause_budget.py` classifica
segundo parado por causa, e uma delas é `mux_gap` = *"o seguidor comandava e o
`auto_vel_pre` estava zerado"*. Sem guard o `auto_vel_pre` fica **mudo a run
inteira** — todo segundo parado viraria `mux_gap`, uma causa que não existe.
Corrigido: sem `auto_vel_pre`, a saída do mux lida passa a ser o `auto_vel_raw`,
e o relatório diz na cara que a stack está sem guard.

De brinde, o `pause_budget` é uma **terceira fonte independente** confirmando a
§2B.7 na `hist3` — ele não sabe nada de cone nem de ground truth:

```
== ORÇAMENTO (quem segura o robô) ==
  guard_hold                     26.8s  (52.1%)
  follower_off[idle]             21.6s  (42.0%)
```

**Metade do tempo parado da `hist3` era o guard.**

### 2B.9 3 voltas SEM guard (`noguard1..3`) — ganho grande e **um BO que não posso esconder**

Comando da §4.5 com `motion_guard:=false`. Conferido em 3/3:
`grep -c motion_guard log/sim_ab/noguard*/nav2.log` = **0**.

| volta | tempo | goals | COLISÃO | raspão | folga mín | samba | unstuck | parado |
|---|---|---|---|---|---|---|---|---|
| `hist1` | 230,3 | 5/5 | 0 | 0 | 0,0776 | 0 | 0,0 | 1,0 |
| `hist2` | 269,1 | 5/5 | 0 | 0 | 0,0630 | 0 | 1,5 | 26,7 |
| `hist3` | 265,6 | 5/5 | 0 | 0 | 0,0451 | 0 | 3,2 | 30,7 |
| **`noguard1`** | **227,6** | 5/5 | 0 | 0 | 0,0827 | 0 | 0,0 | **0,0** |
| **`noguard2`** | **221,0** | 5/5 | 0 | 0 | 0,0667 | 0 | 0,0 | **0,0** |
| **`noguard3`** | 245,4 | 5/5 | **9** | **48** | **0,0000** | 0 | 1,5 | 3,6 |

**✅ O que melhorou:** `parado` = **0,0 s em 14 dos 15 goals**. A `noguard2`
(221,0 s) é a **volta COMPLETA mais rápida das 14**, e a `noguard1` a segunda.
⚠️ A `latchN1` marcou 219,8 s — mais rápido em relógio, mas com **4/5 goals**
(achado do review): tempo de volta incompleta não compete com tempo de volta
inteira. Nenhuma
parada longa em nenhuma das três — a assinatura de ~27 s sumiu, como esperado.

**🔴 O que piorou: a `noguard3` bateu.** 9 COLISÃO + 48 raspões, folga
**0,0000** (penetração), tudo na **`A_fresta90_2`** — 58 eventos entre t=60,7 e
t=63,6. É o pior contato desde o `arena_baseline1`, e o **segundo** contato em 14
voltas.

#### Isso foi eu que causei tirando o guard?

**O que está medido, e vai contra essa hipótese:**

1. **A fresta A sempre foi a passagem no fio.** Folga mínima nela, nas 14 voltas:
   0,045 a 0,212 m. O robô **sempre** passou a menos de 21 cm, e em 4 voltas a
   menos de 8 cm. A `noguard3` não estreou o risco — cobrou ele.
2. **O guard nunca atuou ali.** Nas 11 voltas com guard, o estado dele na
   travessia da fresta foi `idle` em **todas**. Os únicos `slowing` do histórico
   somam ~6 s e aconteceram longe: muro oeste (1,29 m), `C_fresta60_1` (0,95 m),
   `cone_4` (0,48 m). Não havia proteção ali pra eu ter removido.
3. ~~**A `noguard3` chegou na fresta ERRADA, e atrasada.**~~ ⚠️ **RETRATADO
   09-01 — ver §2B.10.** O yaw de −5,4° era leitura de UMA amostra solta; medindo
   a travessia inteira, a `noguard3` cruzou o plano dos blocos a **−10,7°**,
   **dentro** da faixa das 13 voltas boas (−8° a −16°). **Yaw não é o
   discriminante.** O que continua valendo: ela cruzou **atrasada** (t=60,9
   contra 35–45 s), depois de um `unstuck` (`reason=near`) aos 50,8 s no goal 1.

**⚠️ O que isso NÃO prova:** são **3 voltas**. Não dá pra tirar taxa de contato
de n=3, e "o guard não atuava ali" não é o mesmo que "tirar o guard não muda
nada em lugar nenhum" — o guard estava na artéria, e a religação do mux é
mudança de caminho de dado. **Não vou registrar guard-off como validado.**

**O que a `noguard3` provavelmente escancarou é um defeito que já estava lá:** a
rota passa por um vão de 90 cm com folga de 4–21 cm, sem nada que corrija o rumo
antes de entrar. Um dia o rumo ia estar 15° fora. Item novo nos abertos.

### 2B.10 O que REALMENTE separa a volta que bateu (medido 09-01)

Fui desenhar a correção da fresta A e a primeira coisa que fiz foi conferir a
minha própria explicação contra a trajetória (`colisao.csv`, x/y/yaw/folga a
20 Hz). **Ela não sobreviveu.**

Geometria: blocos em **x = 7,5** com **0,60 m de espessura** → o túnel é
**x 7,2–7,8**; o vão vai de **y 1,80 a 2,70**, eixo em **y = 2,25**. O robô é um
retângulo 0,5×0,5 (o oráculo usa o retângulo exato girado pelo yaw).

Pose de cada volta no plano dos blocos (x = 7,5):

| volta | t | y | desvio do eixo | yaw | folga mín na A |
|---|---|---|---|---|---|
| `hist2` | 48,2 | 2,169 | −0,081 | −9,8° | 0,0630 |
| `noguard1` | 40,4 | 2,187 | −0,063 | −9,4° | 0,0827 |
| `arena_latch1` | 47,3 | 2,187 | −0,063 | −10,4° | 0,0741 |
| `noguard2` | 41,0 | 2,191 | −0,059 | −12,6° | 0,0667 |
| `aprox3` | 39,8 | 2,188 | −0,062 | −8,1° | 0,0907 |
| `latchN2` | 41,1 | 2,202 | −0,048 | −12,9° | 0,0749 |
| `hist1` | 43,2 | 2,216 | −0,034 | −12,0° | 0,0776 |
| `latchN3` | 41,8 | 2,244 | −0,006 | −15,1° | 0,1014 |
| `arena_baseline1` | 39,1 | 2,293 | +0,043 | −14,7° | 0,0687 |
| `aprox1` | 50,5 | 2,306 | +0,056 | −15,8° | 0,0483 |
| `hist3` | 42,5 | 2,328 | +0,078 | −13,1° | 0,0451 |
| **`noguard3`** | **64,0** | **2,370** | **+0,120** | **−10,7°** | **0,0000** |

**O yaw da `noguard3` está no meio do pelotão.** O que ela tem de único é o
**desvio lateral**: +0,120 m, contra ±0,081 de todas as 11 outras. A minha frase
de "−5,4° quase de frente pro batente" veio de amostra solta e **está retratada**
(BO 69).

**E o contato não foi dentro do túnel — foi ANTES dele.** As 9 COLISÃO estão em
**x 6,90–6,96**, ou seja **~25 cm antes da boca** (túnel começa em 7,2), com o
robô a **y ≈ 2,50** (desvio +0,25): ele encostou na **face frontal do bloco de
cima** (o que vai de y 2,70 a 4,20) enquanto ainda vinha chegando. Depois disso
**raspou por 3 s** com folga travada em 0,018 e yaw **congelado em −10,7°**,
convergindo o desvio de +0,26 até +0,02 — ele **se espremeu pra dentro do vão
raspando**, sem nada reagir ao contato.

**A volta boa fazia o oposto:** a `noguard2` chegou em x=6,90 com desvio +0,170
e yaw **−24,3°** — apontada pra *cortar* em direção ao eixo — e ainda deu um
giro no lugar na boca (−24,3° → −12,6° com x parado em 7,33), entrando com
desvio −0,02. **O yaw grande das voltas boas era a CORREÇÃO, não o defeito.**

**Mecanismo, então:** a perna `cone1→cone2` chega pela esquerda-de-cima e nada
controla o **erro lateral** contra o eixo do vão antes da boca. Cada volta
converge o quanto o plano por acaso cortou. Quem chega com desvio grande **e**
ângulo de convergência pequeno bate na face do batente antes de entrar.

**Margem real que existe pra errar:** vão 0,90; largura varrida por um 0,5×0,5 a
−12° é 0,5·(cos+sen) = **0,60 m** → sobram **0,15 m repartidos nos dois lados**.
Para o **Nav2** é pior: com `robot_radius` 0,32 ele enxerga um círculo de 0,64 →
**±0,13 m** de erro de eixo antes de o plano ficar inviável. É por isso que o
`"start/goal is an obstacle"` mora justamente aqui.

**Nota de escopo:** 2 das 14 voltas (`aprox2`, `latchN1`) **não passaram pela
fresta A** — foram pelo contorno (`y > 4,20`). A fresta é **atalho opcional**
(`tools/gera_arena_galpao.py`), não obrigação da prova.

## 2C. Sessão 2026-09-01 — conferência do review (nada mudou no código)

O dono trouxe um review que aponta 4 problemas. **Os 4 descrevem o estado do
commit `dd4d0cc`** — o `7ff5b5f` (HEAD) já é a correção deles (BO 66/67/68 +
alcance real do `--arena`). Em vez de confiar na mensagem do commit, **re-rodei
a evidência no HEAD**:

| o que o review apontou | estado no HEAD | como conferi |
|---|---|---|
| 1. conferidor achou `guard_bloqueio.csv` não citado | ✅ limpo | `tools/confere_evidencia.py` → **6 pastas, 0 problemas** |
| 2. `test_collision_monitor_le_sempre_o_raw` é asserção vazia | ✅ sensível | injetei `cmd_vel_in_topic: auto_vel_pre` no `nav2_params_arena.yaml` → **o teste falhou**; restaurei → 6/6 passam |
| 3. "volta mais rápida das 14" (a `latchN1` fez 4/5 goals) | ✅ corrigido | texto agora diz "volta **completa** mais rápida"; `latchN1` com a ressalva |
| 4. `--arena` desliga o vigia **no robô real** | ✅ escrito + aviso na tela | `launch.sh:586-591` imprime o alerta quando arena **sem** `--sim`; condição na §2B.8 e no item 2j |

E a ordem que o review propõe (fresta A **antes** de repetir voltas) já é a que
está registrada: **item 2k antes de novas voltas**. Uma config que fez 9 colisões
+ 48 raspões não vira validada por mais n.

**Achado lateral (pequeno, não mexido):** `nav2_params_legacy.yaml` ainda existe
no `config/`, mas o `ESTADO_PROJETO.md:37` e o `HANDOFF_NAV2_TREKKING.md:95`
dizem que ele foi **apagado**. Nenhum caminho do `launch.sh` o carrega (o ramo
sem `--pi/--sim/--arena` não passa `params_file` nenhum), então é arquivo morto
+ doc desencontrada, não risco de subir com a geometria errada.

Suíte completa depois de restaurar o yaml: **411 passam** (`python3 -m pytest ros2_packages/robot_nav/test -q`).

**Segunda rodada do review (mesmo dia):** sobrou uma inconsistência — o
cabeçalho do `HANDOFF_ARENA_GUARD.md` dizia *"Último commit: `a052c18`"* com o
HEAD já em `cddeb3b`. O **hash estava certo, o rótulo é que não**: `a052c18` é o
commit **da mudança** (guard-off), e um cabeçalho que nomeia a ponta do ramo
envelhece a cada doc que eu commito. Virou "commit da mudança" nos dois
handoffs — o `HANDOFF_NAV2_TREKKING.md` tinha o **mesmo defeito não apontado**
(dizia `e03555b`, ponta real `f78ffc1`). Só rótulo; nenhum código tocado
(`4b44819`).

Working tree limpo, `origin/arena-galpao` = HEAD (nada parado no dev).

## 2D. Sessão 2026-09-01 — review do Codex ao desenho da fresta A (nada mudou no código)

O dono trouxe o review do Codex ao
`docs/superpowers/specs/2026-09-01-fresta-a-door-crossing-design.md`. **Confiro
cada ponto no código antes de aceitar** — e o resultado é que **4 dos 5 procedem,
e um deles derruba o argumento central do desenho.**

| # | apontamento | veredito | como conferi |
|---|---|---|---|
| 1 | O nó **não arma** na rota da arena: `_pick_door` exige a porta em `_cleared`, e `_cleared` só recebe id quando um goal termina **SUCCEEDED com o robô na zona** | ✅ **procede, e é decisivo** | `door_crossing.py:330` (`if d['id'] not in self._cleared: continue`) e `:359-367`. `maps/routes/arena_galpao.json` tem 5 waypoints (4 cones + chegada), **nenhum** na frente da fresta. Contrato coberto por teste: `test_door_crossing.py:529-572` |
| 2 | O argumento *"0,120 > 0,080 → não atravessaria"* **está errado**: `align_lat` não decide nada | ✅ **procede** | `grep cfg.align_lat` → **uma** ocorrência, `:613`, dentro de uma **string de log**. O arme vai `idle → rotating` DIRETO (`:381`, comentário de 19/06 diz explicitamente *"NÃO faz staging"*), e `rotating` só exige `abs(yaw_err) <= align_yaw` **e** `will_clear()` |
| 3 | Os testes propostos em §4.9 **não são vermelhos** | ✅ **procede** | Sem `_cleared`, o estado fica `idle` para sempre → *"não pode estar em crossing"* passa com a máquina desligada. É o BO 56/63/66 outra vez, agora **no desenho** |
| 4 | O fallback é **fail-open** e o *"pior caso = comportamento atual"* é impreciso | 🟡 **parcialmente** | `_abort` (`:275`) devolve ao Nav2 — e é literalmente o estado de hoje. Mas **antes** de abortar a máquina pode ter girado ou dado ré: ela entrega ao Nav2 uma **pose diferente**, não a mesma. Essa metade procede. Ver a ressalva abaixo |
| 5 | A garantia do point-turn (18,7 cm) está incompleta | ✅ **procede — e é pior do que o review diz** | O Codex desconta `stage_tol = 0.10` (`:183`) e chega a ~8,7 cm. Mas com o arme **direto em `rotating`** (ponto 2) o giro inicial **não acontece no ponto de preparação**: acontece onde o robô entrar na zona (raio `1,1 m`, `bearing ≤ 70°`). Ex.: armando em (7,00; 1,99) a distância ao canto (7,20; 1,80) é **0,276 m < 0,354 m** de raio circunscrito → o círculo varrido **invade o bloco**. A conta da §4.4-(a) descreve um lugar que a máquina não usa |

### O que sobra do desenho (e o que não sobra)

**Não sobra:** o §3 (diagrama `IDLE → STAGING`), a linha *"entra em CROSSING
|lat| < 0,08"* da tabela §4.3, o "argumento decisivo" do §2, o critério 2 do §4.10
e a justificativa do teste do §4.9. **Todos descrevem o desenho de 12/06, não o
código em vigor depois de 19/06.** Eu li o `door_crossing.py` inteiro e mesmo
assim copiei a docstring do topo do arquivo (`:9-14`) — que também está
desatualizada (é o BO da docstring `|yaw|<5°` que eu **já tinha anotado** na
§5.4, sem perceber que o diagrama ao lado tinha o mesmo defeito).

**Sobra, com a mecânica corrigida:** o discriminante real é `will_clear()`
(`:72-91`), com `fit = 0,45 − 0,25 − 0,05 = 0,15 m`, e ele **é** sensível à pose
ruim. Refazendo a conta com a amostra do 1º raspão da `noguard3`
(`d = +0,259`, `yaw_err = −7,7°`, `s ≈ −0,6`, `side = −1`):

```
lat = d + s·side·tan(yaw_err) = 0,259 + (−0,6)(−1)(−0,135) = 0,178 m  >  0,15  → REPROVA → re-estagia
```

E com o `d = +0,120` que eu usei no §2:

```
lat = 0,120 − 0,081 = 0,039 m  ≤  0,15  → APROVA → entra em crossing
```

Ou seja: **a máquina provavelmente pega a pose do raspão, mas pela trava
`will_clear`, não pelo `align_lat`; e a pose de 0,120 que eu apresentei como "a
que ela teria barrado" ela deixa passar.** A conclusão do desenho pode continuar
de pé — o argumento que a sustentava, não.

### Ressalva ao ponto 4 (onde eu discordo do remédio, não do diagnóstico)

O Codex propõe *"manter zero/standdown até condição explícita de liberação"* em
scan velho / vão bloqueado. **Isso é um freeze com prioridade 20 no mux** — e o
projeto já reverteu exatamente essa troca: `align_timeout` 15 → 600 virou *"um
FREEZE de 10 min"* e foi desfeito (`door_crossing.py:189-193`). Numa prova
cronometrada, robô parado para sempre é volta perdida, não é o lado seguro.
O que aceito e corrijo é a **frase**: o pior caso não é "o comportamento atual
mais tempo", é *"o comportamento atual, a partir de uma pose que a máquina pode
ter mexido"*.

**Nada foi implementado, nada foi decidido.** O desenho leva um aviso de
retratação no topo apontando para cá; a direção do conserto é decisão do dono.

## 2E. Sessão 2026-09-01 — 2ª rodada do Codex: ele achou 2 erros MEUS na retratação

A retratação da §2D introduziu **duas imprecisões novas**. As duas procedem; a
terceira observação é uma distinção que eu apaguei indevidamente.

### 2E.1 ✅ A conta do `will_clear` usava o yaw da CHEGADA, não o da decisão

`will_clear()` só é avaliado **depois** que o `rotating` fechou `|yaw_err| ≤ 3°`
por `align_stable = 5` ticks (`door_crossing.py:439-446`) — e o point-turn **não
muda `x`/`y`** (`Cmd('rotating', 0.0, wz, ...)`, `:466`), logo **`s` e `d` são os
mesmos**. Usar `yaw = −7,7°` (a pose de chegada) na conta é usar um ângulo que a
máquina já corrigiu antes de decidir.

Refazendo com o yaw que existe no instante da chamada (`|yaw_err| ≤ 3°`,
`tan 3° = 0,0524`, braço `s·side = +0,6` → `±0,031 m`):

| pose | projeção no plano dos batentes | contra `fit = 0,15` |
|---|---|---|
| `d = +0,259` (1º raspão da `noguard3`) | **0,228 … 0,290 m** | **reprova sempre** |
| `d = +0,120` | **0,089 … 0,151 m** | aprova — **exceto** colada em +3°, onde 0,151 > 0,150 |

**A conclusão sobre a pose do raspão sobrevive e fica mais forte** (reprova em
toda a janela, não por 0,028 de folga). Mas **eu não podia ter afirmado que
`d = 0,120` "passa"**: passa em quase toda a janela e reprova na borda. Os 0,178
que publiquei não são o número de decisão nenhuma.

### 2E.2 ✅ "O giro acontece onde o robô entra na zona" está errado

Entrar no círculo de 1,1 m **não basta**. O arme exige, em ordem:
`_cleared` populado (goal SUCCEEDED na zona) → **outro** goal ativo →
`nav_forward` → aí sim `_pick_door` e `rotating`. Meu exemplo (7,00; 1,99) prova
que **o conjunto de poses de arme permitidas contém posições perigosas** — não
prova que a rota giraria ali. O correto:

- **sem waypoint pré-fresta:** não gira **nunca** (é a §2D.1);
- **com pré-fresta em (6,90; 2,25):** gira **a partir dali**;
- **se alguém remover o `_cleared`:** aí sim o giro nasce perto da entrada da
  zona, e o lugar tem que sair da **trajetória medida**, não de um ponto que eu
  escolhi para ilustrar.

> ⏳ **Detalhe meu, ainda por medir (é inferência, não medida):** "a partir dali"
> não é "exatamente ali". O pulso de `goal_succeeded` é consumido em 1 tick
> (`:683-684`), mas o arme também exige `goal_active` — e entre dois goals existe
> um buraco medido de até **16 s em `idle`** (item 2g). Quando o goal seguinte
> entra, o robô pode já ter andado. A janela real de arme (e a folga do círculo
> varrido nela) tem que sair do CSV de uma volta, não desta linha.

### 2E.3 🟡 Fail-closed limitado ≠ freeze — a distinção é dele, e é justa

Eu colei "manter zero até liberação" no `align_timeout` 15 → 600 revertido. **São
coisas diferentes:** fail-closed pode ser **limitado** (zero até o scan voltar ou
até um timeout curto). A analogia foi preguiçosa e o título da §4.8 do desenho —
*"Aborto e fallback **seguro**"* — não se sustenta com A4 exigindo zero contato:
devolver ao mesmo plano que aponta para um vão que eu **acabei de medir como
bloqueado** é avanço sem percepção, não segurança.

**O que eu acrescento, e que muda o custo do remédio:** os desfechos que ele
propõe — *"cancelar a passagem e mandar o executor pelo contorno"*, *"solicitar
nova rota"* — **não existem hoje, em lugar nenhum**. Conferido:

- o `door_crossing` **não tem `ActionClient` nem publisher de goal**: os únicos
  publishers são `door_vel` e `door_zone` (`:593-594`). Ele não cancela nem
  substitui goal — está escrito na §4.6 do desenho e é verdade.
- **não há keepout/filter_mask** em `config/` nem em `robot_nav/` (o `grep` por
  `contorno` só acha plano-de-desvio de **pessoa**). O `probe.py` manda os
  waypoints em sequência e não replaneja.

Então "fail-closed limitado" hoje se decompõe em duas opções honestas, e a
escolha é do dono:

| | o que é | custo |
|---|---|---|
| **(a)** zero limitado **e depois abortar** | atraso antes do mesmo desfecho de hoje | ~nada; **não fecha** a objeção — só adia o retorno cego |
| **(b)** desistir da passagem de verdade | exige **máquina nova**: marcar a fresta como intransponível e/ou o executor pular/replanejar | mudança no executor + no planner a 4 dias da prova; é **maior** que reativar o `door_crossing` |

**Não escolho sozinho.** O que eu não posso é continuar chamando (a) de "fallback
seguro".

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
AB_EXTRA_LAUNCH="follow_clear_full:=1.2 follow_clear_min:=0.35 motion_guard:=false" \
  bash tools/sim_ab/run_one.sh robot_nav arena_v1
```

`AB_SX/AB_SY` = a **largada** (1.0, 1.0). O default do harness é (2.0, **0.0**),
que na arena fica **em cima do muro sul**.

**Gerou pasta em `docs/baselines/`? Roda o conferidor ANTES do commit** (BO 67 —
eu não rodei, e ele acha em 0,2 s o arquivo arquivado que o README esqueceu de
citar):

```bash
python3 tools/confere_evidencia.py     # sai 0 se está tudo certo
```

⚠️ **`motion_guard:=false` é obrigatório aqui, e é fácil de esquecer.** Este
harness **não passa pelo `--arena`** do `launch.sh` — ele monta os argumentos do
launch na mão. O `--arena` desliga o guard sozinho; o `AB_EXTRA_LAUNCH`, não. Um
comando sem essa linha volta a rodar com o guard e traz de volta as paradas de
~27 s da §2B.7, sem avisar. Conferência barata: `grep -c motion_guard
log/sim_ab/<tag>/nav2.log` tem que dar **0**.

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
| 56 | O microsim da aproximação **passava sem a correção** | Modelei a deriva do point-turn empurrando o robô **em direção** ao goal; o medido é o contrário (0,147 → 0,166, afastando). Com o sinal errado o laço convergia sozinho e o teste aprovava o código quebrado. Percebi porque **estranhei 2 dos 5 testes novos passarem de primeira** | Teste novo que passa antes da correção é suspeito, não alívio: rodar a suíte ANTES de implementar e exigir a falha com a mensagem certa |
| 57 | Escrevi o critério de aceite **cego pro estado que eu tinha acabado de criar** | O aceite era "`goal_turn → turning` continua 0", e o `goal_approach` nasceu no mesmo commit: samba pela porta nova passaria batido. Não houve nenhuma, mas a métrica **estava cega** | Criou estado novo, revise as métricas que classificam estado — no mesmo commit |
| 58 | Usei **"tempo parado"** pra julgar uma correção que ADICIONA giro | O `parado` do probe é deslocamento nulo, e point-turn conta como parado. A `aprox2` marcou 56,6 s "parada" com só 7,4 s de robô genuinamente imóvel; o resto era churn de mira | Escolher a métrica pelo mecanismo que ela precisa distinguir. A boa aqui era a **distância final no goal**, que mede o defeito direto |
| 59 | **Limiar pelado dentro do código escrito pra curar limiar pelado** | Na aproximação, o giro de mira usava `turn_enter` dos **dois** lados: ≥16° gira, <16° avança. Chattering possível em torno de 16°, e **o microsim não pegava porque começa apontado pro goal**. Achado no review | Ao criar uma decisão liga/desliga, perguntar qual é o **par** de limiares. E teste de laço fechado tem que começar na condição RUIM, não na boa |
| 60 | Afirmei que **"a ação nunca completa"** por causa dos 0,15 | O `goal_checker` é **`stateful: true`**: satisfeito o XY uma vez, ele só reconfere yaw — e o robô ENTROU dentro dos 0,15. A cadeia causal do item 2e era **inferência minha**, não medição. O que está medido é robô parado + `nav_wants=1` + unstuck por timeout | Ler os parâmetros do componente que eu estou acusando **antes** de escrever a cadeia causal. `stateful` estava a 2 linhas do `xy_goal_tolerance` que eu citei |
| 61 | Tratei **"último tick antes da troca de goal"** como "pose no instante da conclusão" | A 20 Hz e 0,22 m/s há **~1,1 cm entre amostras**; os 0,156 do goal 2 podem ter cruzado 0,15 entre ticks. O 2h não está provado | Declarar a **resolução** da medida junto com o número, sempre que ela for da ordem do efeito |
| 62 | Deixei o diário dizendo **7** enquanto minha própria ferramenta dizia **8** | Ampliei o `conta_samba()` pra cobrir `goal_approach` e `driving`, o baseline passou a somar 8 (tem uma saída `goal_turn → driving` a 0,15 que a métrica estreita não via) — e não varri o texto. Ficou 3 commits assim | Mudou o critério de uma métrica: **regerar e reconciliar todo número já publicado com ela**, no mesmo commit |
| 63 | Asserção **VAZIA** no microsim novo — no teste escrito pra responder o review anterior | `trocas = sum(... if p == q == 'goal_approach')` comparado com `< 600`, num laço de no máximo 599 pares: **não podia falhar**. E contava pares de estados IGUAIS, não alternâncias; o nome do estado nem distingue mirar de avançar — quem distingue é o `vx`. É o BO 56 outra vez, dois commits depois | Toda asserção numérica nova tem que ser **provada sensível**: agora há um teste que roda o microsim com `turn_exit == turn_enter` (o defeito) e exige alternância MAIOR — 1 com histerese, 5 sem |
| 64 | Anunciei retratações e **não as propaguei** | Retratei no corpo da §2B.6 e deixei: o título absoluto *"NÃO são da aproximação"*, o README repetindo as duas conclusões retiradas, a coluna `dentro_do_checker_0.15` afirmando o julgamento do checker, e os itens 2e/2f/2h repetindo a causalidade caída | Retratação não é parágrafo, é **varredura**: título, README, nome de coluna, itens abertos. O mesmo erro do BO 48, agora com retratação em vez de fato |
| 68 | Chamei a `noguard2` de "**volta mais rápida das 14**" comparando só o relógio | A `latchN1` fez **219,8 s**, mais rápido — com **4/5 goals**. Comparei tempo de volta incompleta com tempo de volta inteira, o que infla o ganho que eu estava anunciando | Tempo só compara entre voltas com o **mesmo número de goals cumpridos**; a coluna `goals_ok` entra na frase, não no rodapé |
| 69 | 🔁 **Diagnostiquei a batida da `noguard3` por UMA amostra de yaw** — e publiquei no diário, no handoff e no chat | Escrevi que ela entrou na fresta a **−5,4°** contra −13°/−26° das boas, e construí em cima disso o item 2k ("entra torta, nada corrige o rumo"). Medindo a travessia inteira: **−10,7°, no meio do pelotão**. O discriminante é o **desvio lateral** (+0,120 m contra ±0,081), e o contato foi **25 cm ANTES da boca**, na face do batente — não dentro do vão. É o BO 65 outra vez: conclusão sobre uma volta inteira tirada de um ponto | Antes de virar item de backlog, **plotar a trajetória inteira** do evento (a `colisao.csv` tem x/y/yaw/folga a 20 Hz). Uma amostra não descreve uma travessia — e foi o desenho da correção que quase saiu errado por causa disso |
| 67 | Não rodei `tools/confere_evidencia.py` — a ferramenta que existe **exatamente** pra isso | Deixei `guard_bloqueio.csv` arquivado e **não citado** no README da pasta nova. O conferidor pega isso em 0,2 s, foi escrito depois dos BOs 31/45/50 (a mesma falha 3×) e eu não o executei nas duas pastas que criei hoje | Gerou pasta em `docs/baselines/` → **roda o conferidor no mesmo comando**, antes do commit. Está na §4.5 agora |
| 66 | 🔁 **Asserção VAZIA outra vez** — no commit em que eu comemorava ter provado outro teste sensível | `test_collision_monitor_le_sempre_o_raw` achava o nó e depois só afirmava que a saída do mux era **um dos dois valores possíveis**: nunca olhava o `collision_monitor`. Passava com o pipeline quebrado. É o BO 63 **na íntegra**, dois commits depois, no mesmo arquivo em que eu tinha acabado de provar sensibilidade injetando defeito | Provar sensível **teste a teste**, não uma vez por arquivo: o teste que eu injetei defeito pra ver falhar era o *outro*. Agora o do collision lê o `cmd_vel_in_topic` dos dois YAMLs, e falha se apontarem pro `auto_vel_pre` |
| 65 | Atribuí as paradas longas ao **churn da mira** sem medir o pipeline | A histerese cortou o chattering 3–5× e o tempo parado **não caiu**. A causa dos piores casos era o `motion_guard` zerando o giro — um nó que eu nem tinha olhado, e cuja volta ao caminho está registrada no item 7 dos meus próprios abertos | Quando o robô não se move, ler o **pipeline inteiro** (`freeze_capture` tem todos os estágios) antes de acusar o nó que eu acabei de mexer |
| 22 | No teste novo do parser, **minha expectativa estava errada** | Escrevi (2,0 · 1,0) — que é o resultado de **ignorar** o yaw. O teste teria passado na versão com bug | Ao testar rotação, afirmar também o valor que o bug produziria |
| 74 | Calculei o `will_clear` com o yaw da **chegada**, quando ele só roda depois do alinhamento | Usei `−7,7°` e publiquei `0,178`. A trava só é chamada com `|yaw_err| ≤ 3°` (`:439-446`) e o point-turn não muda `s`/`d` — a projeção real é uma **janela** (0,228–0,290 para `d=0,259`; 0,089–0,151 para `d=0,120`). Achado pelo revisor, **na correção que eu tinha acabado de escrever para outro erro do mesmo tipo** (BO 71) | Ao avaliar uma guarda, usar o estado **no ponto de chamada**, não o estado de entrada. E se a entrada é um intervalo (±3°), o resultado é **intervalo**, não ponto — "passa" e "reprova" só valem se a janela inteira concordar |
| 75 | Disse que o point-turn acontece **"onde o robô entra na zona"** | Entrar no raio de 1,1 m não arma: falta `_cleared` + goal ativo + `nav_forward`. Meu exemplo (7,00; 1,99) mostra que o **conjunto de poses permitidas** contém posições perigosas — não que a rota giraria ali. Eu tinha **acabado de documentar** o gate `_cleared` no item ao lado (BO 73) e mesmo assim escrevi a frase ignorando ele | Corrigir um gate esquecido e depois raciocinar como se ele não existisse é o mesmo erro duas vezes no mesmo parágrafo. Depois de mapear as pré-condições, **reescrever as conclusões que foram tiradas sem elas** — inclusive as da mesma página |
| 70 | 🔁 **Copiei o diagrama de estados da docstring em vez de ler a máquina** — e ele está desatualizado desde 19/06 | O desenho da fresta A afirma `IDLE → STAGING → ROTATING(|lat|<8cm E |yaw|<3°)`. O código arma **direto em `rotating`** (`door_crossing.py:381`) e o gate é `yaw` + `will_clear()`; `align_lat` **não é lido por decisão nenhuma** (`grep` → 1 ocorrência, numa string de log, `:613`). Achado pelo revisor. Pior: eu **já tinha anotado** na §5.4 do próprio desenho que a docstring estava errada em outro número (`|yaw|<5°`) e não desconfiei do diagrama 4 linhas acima | Docstring é comentário datado, igual ao caso da odom do Gazebo. O diagrama de estados se lê **do `update()`**, e um parâmetro só existe se `grep cfg.<nome>` achar um **uso**, não uma declaração |
| 71 | Construí o **argumento decisivo** do desenho em cima de uma comparação que a máquina não faz | *"0,120 > 0,080 → ela não teria deixado atravessar"*. O discriminante real é `will_clear` com `fit = 0,15 m`, e ele **aprova** 0,120 com aquele yaw (projeção 0,039). A conclusão pode sobreviver — via a outra trava, com a pose de 0,259 — mas o número que eu publiquei como prova prova o contrário | Antes de chamar um número de "argumento decisivo", achar a **linha que compara esse número**. Sem uso, é declaração |
| 72 | 🔁 **Teste vermelho que não é vermelho** — no documento escrito para ser revisado | O teste do §4.9 (*"com a pose ruim, não pode estar em `crossing`"*) passa com o nó em `idle` para sempre, que é justamente o estado real (o contrato `_cleared` nunca é cumprido na rota da arena). É o BO 56/63/66 pela **quarta** vez, agora antes de existir código | Um teste de "não faz X" só é sensível se o **par positivo** ("faz X nesta condição") rodar no mesmo caminho de arme. Escrever os dois juntos, sempre |
| 73 | Chamei o `/doors` de **"o bloqueador de integração"** (singular) | Havia um segundo, decisivo e testado: `_pick_door` exige `_cleared`, populado só por um goal **SUCCEEDED dentro da zona** (`:330`, `:359-367`, `test_door_crossing.py:529`). Com `doors_file` e o launch ligado, o nó ainda ficaria `idle` a volta inteira — e eu poderia ler a volta como "a máquina não achou necessário atuar". Achado pelo revisor | Antes de declarar UM bloqueador, percorrer **todos** os `continue`/`return` do caminho de arme e checar cada pré-condição contra a rota real |

---

## 6. Aberto

| # | item | estado |
|---|---|---|
| 1 | **Proteção de point-turn** (anel 0,25–0,36 m no `path_follower`) | 🔴🔴 **bloqueador de A4, agora MEDIDO como o que falta**: a `aprox2` raspou o `cone_3` 4× com o seguidor em `turning` a **6,15 m do goal** (§2B.6). 7 voltas com latch: 6 limpas, 1 com contato — a causa nunca foi corrigida, só não tinha se alinhado. **Desenho fechado** (§2.10), zero código. **Não é** no collision monitor: lá a única alavanca é escalar o `wz`, que trava o robô (deadlock já reproduzido) |
| 2 | Baseline Nav2 até os standoffs | ✅ **FEITO 08-28** — 5/5 goals, 236,4 s, 2 colisões + 28 raspões no cone (§2.8). Evidência em `docs/baselines/2026-08-28-arena-baseline1/` |
| 2b | Travar a chegada (mata a samba — defeito provado) | ✅ **FEITO 08-31** (`c85a8d8`) — saídas da chegada pro carrot **8 → 0**; volta em `docs/baselines/2026-08-31-arena-latch1/` (§2B.4) |
| 2e | 🟡 **O robô PARA longe do goal e o Nav2 não conclui** | **ATACADO 08-31, não fechado** (`4da6eb4`, §2B.6). ⚠️ O título era *"estaciona fora do `xy_goal_tolerance`"* — **a explicação pelos 0,15 caiu** (checker é `stateful`, item 2i). O que é medido: robô parado + `nav_wants=1` + unstuck por timeout, em 4/4 voltas. Com a aproximação a última amostra cai pra **3–9 cm** (era 11–15) e o `unstuck` some (0,0–3,0 s) — sobra **churn de mira** (~54 s na `aprox2`) |
| 2f | **`unstuck` empurra robô que JÁ CHEGOU** | ⏳ **novo 08-31, menos urgente depois de `4da6eb4`**: os disparos caíram de 1,3–6,2 s para **0,0 / 3,0 / 1,4 s** — a aproximação reduziu o *tempo parado* que aciona o resgate (⚠️ **não** "removeu a causa": por que o Nav2 não concluía continua sendo o item 2i), **não o mecanismo**. O supervisor segue sem assinar estado nenhum do `path_follower`, então continua sem distinguir "encalhado" de "chegou e parou". Passo 4 |
| 2c | 🔴🔴 **AMCL erra 24 cm na arena** (mediana 9, p90 16, max 27) | ⏳ **novo, 08-28.** Maior que a tolerância de A2 (20 cm) e muito maior que os ±3 cm da fresta de 0,60. Suspeita: arena pobre em feature + os 4 cones não estão no mapa |
| 2d | Bug `start/goal is an obstacle` **reproduzido nesta arena** | ⏳ disparou dentro da fresta A (0,90 m), indo pro goal 2 |
| 2g | **16 s em `idle`** entre dois goals (`aprox3`) | ⏳ **novo, 08-31.** Sem goal ativo — não é o seguidor. Buraco entre a conclusão de um goal e o próximo ser aceito; não investigado |
| 2h | **Goal 2: última amostra a 0,156–0,160 nas 3 voltas** | ⏳ **novo, 08-31.** Único goal que a aproximação não puxa pra dentro, e sempre o mesmo. ⚠️ **NÃO é prova de que termina fora da tolerância** (BO 61: ~1,1 cm entre amostras a 20 Hz, e o checker é `stateful`). Suspeita não verificada: algo bloqueia o avanço ali (é o goal cujo caminho passa pela fresta A) |
| 2i | **Por que o Nav2 ainda queria movimento com o robô já chegado?** | ⏳ **novo, 08-31.** A explicação que eu tinha dado (parou fora dos 0,15) **caiu**: o checker é `stateful` e o robô entrou dentro. Hipótese do próprio log: `Failed to make progress` → recovery → **reset do goal checker** → o XY volta a ser exigido. Medir antes de afirmar |
| 2k | 🔴 **Fresta A: nada controla o ERRO LATERAL antes da boca** (título corrigido — BO 69) | ⏳ **novo, 08-31 (§2B.9).** Nas 14 voltas a folga mínima na `A_fresta90_2` foi 0,045–0,212 m; a `noguard3` entrou com yaw **−5,4°** (as outras 13: −13° a −26°) e **bateu** (9 COLISÃO + 48 raspões). Nada alinha o robô antes do vão. ⚠️ n=3 sem guard: **não** está provado que tirar o guard causou — o guard estava `idle` na fresta em 11/11 voltas — mas também não está provado que não. **DESENHO 09-01 escrito, aguardando revisão do Codex:** `docs/superpowers/specs/2026-09-01-fresta-a-door-crossing-design.md` — reativar o `door_crossing` (decisão do dono); medição em §2B.10 |
| 2j | 🔴🔴 **`motion_guard` bloqueia o robô na arena** | ⏳ **novo, 08-31 (§2B.7).** Vigia de PESSOA ligado numa prova **sem pessoa**; zera o giro entre `auto_vel_pre` e `auto_vel_raw`. Medido: **26,9 s** (`hist3`) e **52,1 s** (`aprox2`); nos episódios, ~505 comandos entraram e **1** saiu. Os 3 episódios duram 25,7–26,9 s = `hold_still_max` 20 + `clear_time` 5 + settle, sempre com um **cone** como único vizinho — a vigília roda até o teto em cima do cone. Só dispara nas voltas com aproximação (que adiciona point-turn perto de cone). ✅ **DESLIGADO na arena por decisão do dono 08-31** (`motion_guard:=false`, §2B.8); 3 voltas sem ele na §2B.9 — parado 0,0 s em 14/15 goals, mas **1 das 3 bateu na fresta A** (item 2k), então **não** está validado. 🔴 **`--arena` vale no REAL também**: sem `--sim` o robô físico sobe sem vigia de pessoa — exige pista controlada + E-STOP na mão |
| 3 | Executor que não pula ponto após falha | ⏳ |
| 4 | Aproximação final ao cone (A2) | ⏳ o `PolygonFront` bloqueia o avanço a ~0,67 m do centro do cone, **antes** dos 20 cm |
| 5 | LED/relé | ⏳ interface já existe: `/light/marker` (pino 8) e `/light/cmd` (pino 7) no `mega_bridge` |
| 6 | Medir no robô: bitola, entre-eixos, altura do LiDAR | ⏳ trena |
| 7 | Refazer os números de 08-27 | ⏳ os antigos vieram do fork **sem `motion_guard`** e com oráculo cego a 5 obstáculos |
| 8 | Bug `"start/goal is an obstacle"` | 🔴 **AGRAVADO 08-31**: em `latchN1` ele **custou o goal 2** (Nav2 abortou), no mesmo ponto da §2.8 — partindo de dentro da fresta A. Saiu de "recovery" para "goal perdido". É a **única** perda em 20 goals com o latch (§2B.5) |
| 9 | `nav2_params_legacy.yaml` existe, mas os docs dizem que foi apagado | ⏳ **novo, 09-01.** `ESTADO_PROJETO.md:37` e `HANDOFF_NAV2_TREKKING.md:95` afirmam que o arquivo foi **apagado**; ele está no `config/`. Nenhum ramo do `launch.sh` o carrega (sem `--pi/--sim/--arena` não passa `params_file`), então é arquivo morto + doc desencontrada — **não** risco de geometria errada. Apagar o arquivo ou corrigir os dois docs |
