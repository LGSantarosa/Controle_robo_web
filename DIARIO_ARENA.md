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
| A4 | **zero contato** com bloco, cone ou parede | 🔴 **BLOQUEADO** — sem proteção de point-turn |
| A5 | completa a missão **sem** atravessar fresta | ✅ provado **no mapa**; falta no sim |

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

> `sala_grande.sdf`, o mundo de **todas** as medições de 08-27, tem **25
> obstáculos**. O oráculo enxergava **20**. Os 5 cilindros eram invisíveis
> (o parser só procurava `<box><size>`). O *"zero colisões, folga mínima
> 3,7 cm"* foi concluído com **20% do mundo fora da conta.**

Segundo bug: lia `<box>` de qualquer lugar, inclusive `<visual>`. Em
`sala_grande` não havia caso (conferido), mas na arena a plataforma amarela é
visual pura e viraria obstáculo fantasma — acusaria "colisão" toda vez que o robô
pisasse no alvo. Agora só `<collision>`.

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

```bash
# autotestes (nenhum precisa de ROS nem do robô)
python3 tools/gera_arena_galpao.py --conferir      # invariantes da arena
python3 tools/mapa_passagens.py   --autoteste      # mapa sintético, vão 0,60
python3 tools/sim_ab/colisao.py   --autoteste      # 7 casos de geometria
python3 -m pytest ros2_packages/robot_nav/test/ -q # 393 testes

# regenerar mundo e mapa a partir da tabela
python3 tools/gera_arena_galpao.py --mapa maps/

# validar o mapa com os argumentos gerados pela própria tabela
python3 tools/mapa_passagens.py maps/arena_galpao.yaml \
  $(python3 tools/gera_arena_galpao.py --probes | tr '\n' ' ')

# rodar o perfil da arena
./launch.sh --sim --nav2 --arena

# uma volta A/B no perfil da arena
AB_PARAMS=nav2_params_arena.yaml \
AB_WORLD=$PWD/worlds/arena_galpao.sdf \
AB_MAP=$PWD/maps/arena_galpao.yaml \
AB_EXTRA_LAUNCH="follow_clear_full:=1.2 follow_clear_min:=0.35" \
  bash tools/sim_ab/run_one.sh robot_nav arena_v1
```

---

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

---

## 6. Aberto

| # | item | estado |
|---|---|---|
| 1 | **Proteção de point-turn** (anel 0,25–0,36 m no `path_follower`) | 🔴 bloqueador de A4. **Não é** no collision monitor: lá a única alavanca é escalar o `wz`, que trava o robô (deadlock já reproduzido) |
| 2 | Baseline da missão no sim, medido | ⏳ próximo passo |
| 3 | Executor que não pula ponto após falha | ⏳ |
| 4 | Aproximação final ao cone (A2) | ⏳ o `PolygonFront` bloqueia o avanço a ~0,67 m do centro do cone, **antes** dos 20 cm |
| 5 | LED/relé | ⏳ interface já existe: `/light/marker` (pino 8) e `/light/cmd` (pino 7) no `mega_bridge` |
| 6 | Medir no robô: bitola, entre-eixos, altura do LiDAR | ⏳ trena |
| 7 | Refazer os números de 08-27 | ⏳ os antigos vieram do fork **sem `motion_guard`** e com oráculo cego a 5 obstáculos |
| 8 | Bug aberto `"start/goal is an obstacle"` | ⏳ anterior a esta fase |
