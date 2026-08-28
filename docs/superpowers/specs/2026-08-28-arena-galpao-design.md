# Design — Arena do galpão (prova de 2026-09-05)

> Sessão de 2026-08-28. Substitui o objetivo da fase anterior (`HANDOFF_PROVA_REAL.md`),
> que era provar velocidade e margem. **A meta mudou: nada de velocidade — o robô
> tem que ser destemido no movimento e MUITO preciso.**

---

## 1. A missão (palavras do dono)

Numa arena de galpão:

```
LARGADA → cone 1 → cone 2 → cone 3 → cone 4 → CHEGADA
```

- Cada cone fica em cima de uma **plataforma amarela grande** no chão.
- O **cone é a âncora de detecção** (o LiDAR o enxerga); a **plataforma é o alvo**.
- **Chegar a 20 cm do cone = já está na plataforma = ponto marcado.**
- Ao marcar o ponto, **um LED acende** (relé a ser remontado no robô).
- Entre os pontos há **blocos com frestas de 60 / 70 / 80 / 90 cm**.
  **Passar pela fresta é OPCIONAL** — é atalho, ganha tempo. Contornar sempre é possível.
- **Sem limite de tempo.** O critério é chegar em todos os pontos **sem bater em nada**.

---

## 2. Critérios de aceitação

| # | critério | como se mede |
|---|---|---|
| A1 | Visita os 4 cones + chegada, na ordem, sem pular ponto após falha | log da missão |
| A2 | Para a ≤ 20 cm do cone (borda do robô → superfície do cone) | pose vs cone no sim; trena no real |
| A3 | LED acende nos 4 cones e na chegada, somente após o ponto ser validado | observação + CSV |
| A4 | **Zero contato** com bloco, cone ou parede | `tools/sim_ab/colisao.py`, depois de suportar cilindros, no sim; inspeção no real |
| A5 | Completa a missão **sem atravessar nenhuma fresta** | atalho desligado + rota planejada fora das regiões das frestas |

**A5 é a rede de segurança do prazo:** se o módulo de fresta não ficar pronto, a missão
ainda fecha. O risco fica isolado no bônus, não na entrega.

---

## 3. Geometria — números do repositório e medições declaradas

O robô **não é uma caixa cheia**: é a estrutura de alumínio com as rodas para fora.
Quem define o envelope são **as rodas**.

| grandeza | valor | fonte |
|---|---|---|
| Envelope, roda a roda (largura e comprimento) | **0,50 × 0,50 m** | trena do dono, 2026-08-28 |
| Estrutura de alumínio (o corpo de fato) | 0,37 × 0,35 × 0,16 m | `robot.urdf.xacro:8`, `sim_robot.sdf:38` |
| Roda | raio 0,085 · largura 0,06 | `robot.urdf.xacro:14-15` |
| Raio inscrito / circunscrito | 0,25 / **0,354** | extremos das rodas, em (±0,25, ±0,25) |
| Altura do LiDAR | **medir no robô antes do deploy** | a URDF resulta em 0,465 m, o SDF em 0,2825 m e o README de campo registra LD06/objetos abaixo de ~0,21 m |
| Cone real | mais alto que o LiDAR | dono |

O valor de 0,465 m é, portanto, **um valor da URDF, não uma medição confirmada**. Também
há divergência de modelo (LD06 nos arquivos operacionais e LD20 citado nesta sessão).
Nenhuma decisão física sobre cone ou rampa pode depender desse número antes da trena.

### 🐛 Inconsistência: a URDF real descreve um robô 6 cm mais largo e 4 cm mais longo

`track_width` e `wheelbase` significam **centro a centro dos eixos**, mas os valores que
estão lá são **medidas externas** — o robô da URDF mede **0,56 × 0,54**.

| propriedade | na URDF | alvo geométrico, se a medida externa for confirmada | conta |
|---|---|---|---|
| `track_width` (esq↔dir) | 0,50 | **0,44** | 0,44 + 2 × 0,03 (meio pneu) = 0,50 |
| `wheelbase` (frente↔trás) | 0,37 | **0,33** | 0,33 + 2 × 0,085 (raio) = 0,50 |

Dá pra ver de onde saiu cada um, no comentário do próprio arquivo (`robot.urdf.xacro:5-11`):
o `0,50` da "bitola" é a medida **externa** usada como se fosse centro-a-centro, e o `0,37`
do "entre-eixos" é o mesmo 37 da linha de cima, o comprimento da **estrutura interna**.

O `sim_robot.sdf` usa rodas em ±0,22 e ±0,165, resultando em envelope 0,50 × 0,50 e
coincidindo com a medida externa declarada. A alteração da URDF deve ser feita nas duas
cópias hoje existentes (`robot_nav` e `nav2_trekking`) **somente após confirmar no robô as
distâncias centro-a-centro**.

Isso é uma correção da geometria visual/TF. **Não trocar automaticamente o
`wheel_base = 0.50` usado por `cmd_vel_to_wheels.py` e pelo launch:** ali o valor é a
bitola efetiva de um skid-steer e precisa ser calibrado pelo movimento, não deduzido da
malha visual.

⚠️ De quebra, o `base_link` da URDF real é um `<box>` de 0,50 × 0,50 × 0,26 — uma
super-aproximação: trata como cheio o espaço vazio entre as rodas. Não afeta o costmap
(que usa o parâmetro `robot_radius`, à parte), então fica anotado, não vira tarefa.

### A conta da fresta de 60 cm

Robô 0,50 m numa fresta de 0,60 m: **5 cm por lado, e só se entrar reto.**

Largura efetiva com yaw θ: `W(θ) = 0,50·(cos θ + sen θ)`

| yaw | largura efetiva | folga lateral restante |
|---|---|---|
| 0° | 0,500 m | ±5,0 cm |
| 5° | 0,542 m | ±2,9 cm |
| 10° | 0,579 m | ±1,0 cm |
| **13°** | **0,600 m** | **0 — encosta** |

Yaw e erro lateral **não têm orçamentos independentes**. A condição de entrada deve ser
avaliada em conjunto, com uma margem configurável:

`|erro_lateral| + 0,25·(|cos(yaw)| + |sen(yaw)|) + margem ≤ largura_da_fresta/2`

Por exemplo, a 5° sobram apenas 2,9 cm de erro lateral absoluto **antes da margem**;
portanto não é
seguro aceitar simultaneamente yaw de 5° e erro lateral de 3 cm. No código atual,
`align_lat = 0.08` é declarado mas não governa a transição de travessia, e o yaw padrão é
3°, não 5°. Para uma abertura exatamente de 0,60 m, o `fit_margin = 0.05` atual deixa
tolerância lateral zero quando o robô está reto. Esses limites precisam ser testados em
conjunto; não basta apertar um parâmetro.

---

## 4. Arquitetura — modo seguro obrigatório e bônus isolado

### Nível 1 — nav2 com `robot_radius: 0.32` = MODO SEGURO

O parâmetro `robot_radius: 0.32` existe em
`ros2_packages/nav2_trekking/config/nav2_params_pi.yaml`. Porém, o fluxo normal de
`launch.sh --nav2` ainda compila e lança **`robot_nav`**, cujo footprint é o quadrado de
±0,25 m. Antes de usar esta arquitetura, `nav2_trekking` precisa ser definido como fonte
oficial e integrado ao launcher; misturar os dois pacotes invalida as conclusões abaixo.

Na geometria contínua ideal, o raio 0,32 exige vão ≥ 0,64 m:

| fresta | nav2 puro |
|---|---|
| 90 cm | candidato a passar — validar no mapa rasterizado |
| 80 cm | candidato a passar — validar no mapa rasterizado |
| 70 cm | candidato a passar — validar no mapa rasterizado |
| **60 cm** | deve fechar na geometria ideal — validar localmente no mapa |

Isso não garante que o Nav2 contorne sozinho nem que o robô nunca raspe: inflação é custo,
o raster do mapa altera a largura observada e um círculo de 0,32 m ainda não contém os
cantos do envelope quadrado (raio circunscrito 0,354 m). O modo seguro deve ter pontos de
contorno explícitos e um preflight que rejeite qualquer plano que entre nas regiões das
quatro frestas. O resultado de 70/80/90 cm só pode ser marcado como aprovado depois dos
testes de mapa e colisão.

### Nível 2 — o atalho de 60 cm (`door_crossing` generalizado)

O `door_crossing` existente pode servir de base, mas **não entrega esse comportamento hoje**:

- está comentado/desabilitado em `nav2_trekking/launch/nav2.launch.py`;
- só trabalha com portas marcadas manualmente;
- exige que o goal anterior à porta seja concluído;
- o `MapBridge` só insere esse goal quando o plano do Nav2 já cruza uma porta marcada —
  justamente o que não ocorre se a passagem de 60 cm estiver fechada no costmap;
- a máquina possui estados de rotação, staging, ré e travessia; não é apenas "two-phase".

O bônus deve ser acionado explicitamente pela missão para **uma fresta configurada**, com
staging antes e depois independentes do plano que a cruza. Pode reaproveitar a geometria e
os controles existentes, mas só entra na passagem quando a desigualdade conjunta de yaw,
lateral e margem acima for satisfeita por várias leituras. Depois de entrar, segue reto,
sem point-turn dentro da fresta. O bônus permanece desligado por padrão.

### Aproximação final ao cone — NÃO é trabalho do nav2

⚠️ **Conflito previsto:** o cone é obstáculo no costmap. A inflação aumenta o custo, mas
**não transforma automaticamente toda a região inflada em obstáculo letal**. O erro
`"Either of the start or goal pose are an obstacle!"` observado anteriormente ocorreu a
42–72 cm de uma parede e foi registrado como possível obstáculo fantasma; não prova que um
goal junto ao cone será recusado por causa da inflação.

**Desenho:** nav2 leva até **~1,0 m** do cone (fora da inflação). Dali, uma **aproximação
final** guiada pelo `cone_detector` dirige reto até a distância-alvo e para. Mesmo padrão
do atalho de fresta: nav2 para o grosso, comportamento dedicado para o fino.

O `cone_detector` atual calcula a média dos **pontos visíveis da superfície** e publica em
`odom`; isso não é o centro geométrico do cilindro. Portanto `dist_alvo_cone = 0,60 m`
centro-a-centro não pode usar diretamente esse `PoseArray`.

Além disso, `nav2.launch.py` não inicia esse detector, e o fluxo de trekking que o inicia
depende de `/trekking/pose`. O launch da missão precisa declarar a dependência explicitamente
ou extrair a clusterização do scan para uma rotina que não dependa desse `PoseArray`.

A aproximação deve medir no scan bruto a distância da frente do robô à superfície do cone,
com alvo nominal parametrizado (por exemplo 0,15 m, mantendo o aceite ≤ 0,20 m), associação
única ao cone esperado e estabilidade por várias leituras. O robô faz point-turn, para,
avança reto e volta a alinhar se o bearing sair da tolerância — sem arco apertado.

Há ainda um bloqueio concreto: o `PolygonFront` do collision monitor ocupa x = 0,25…0,50,
y = ±0,22, com `linear_limit: 0`. Um cone de raio 0,17 entra nessa área quando seu centro
está por volta de 0,67 m, deixando cerca de 0,25 m entre superfícies, antes do A2. Para a
aproximação final, o scan sanitizado deve mascarar **somente o cluster do cone selecionado**
enquanto houver autorização fresca (TTL); o controlador continua usando o scan bruto para
parar. Perda de scan, TF, autorização ou associação única manda velocidade zero. Todos os
outros obstáculos continuam visíveis ao collision monitor.

### LED / relé

A interface já existe: `mega_bridge.py` assina `/light/marker` e `/light/cmd`, e o firmware
da MEGA trata ambos. O pino 8 é o LED marcador; o pino 7 é o relé. A missão deve escolher
**um** conforme a fiação (`/light/marker` por padrão), publicar um pulso não bloqueante e
registrar liga/desliga no CSV. Só há mudança de firmware se a montagem exigir outra
polaridade ou outro pino; `project_mega_pinout` não existe no repositório.

### Executor da missão

Não usar diretamente a política atual do runner web, que tenta duas vezes e depois pula o
waypoint abortado. O executor da arena deve manter a ordem Largada → 1 → 2 → 3 → 4 →
Chegada e **nunca avançar nem acender o LED após falha**. Depois de tentativas configuradas,
entra em estado parado/falha e espera retomada explícita. Sem limite de tempo não significa
loop descontrolado. Também não depender do `waypoint_follower` com
`stop_on_failure: false`; usar o executor próprio ou mudar essa política para a arena.

No modo seguro, cada perna inclui pontos de contorno explícitos. Antes de iniciar, o executor
pede o plano de cada trecho ao Nav2 e recusa a missão se algum plano intersectar a região
configurada de qualquer fresta. A aproximação final só marca o ponto depois de distância
estável, robô parado e A2 satisfeito.

---

## 5. O mundo do sim — `worlds/arena_galpao.sdf`

O world, o mapa e as rotas `arena_galpao` **ainda não existem no repositório**; esta seção
é a especificação dos arquivos a criar, não evidência de que a arena já foi validada.

Galpão de **14 × 9 m** com paredes (o dono confirmou galpão fechado → AMCL tem estrutura).
Espelha o estilo de `sala_grande.sdf`: caixas SDF escritas à mão, `<static>true</static>`.

### Pontos

| ponto | posição (x, y) |
|---|---|
| Largada | 1,0 · 1,0 (rumo +x) |
| Cone 1 | 4,5 · 1,5 |
| Cone 2 | 11,5 · 1,8 |
| Cone 3 | 12,2 · 7,5 |
| Cone 4 | 5,0 · 7,8 |
| Chegada | 1,5 · 2,5 |

Cone = cilindro r 0,17 × h 0,70 (o diâmetro ideal de 0,34 m passa no filtro 0,04–0,45 m
do `cone_detector`; confirmar o cluster no scan simulado).
Plataforma = placa amarela 1,2 × 1,2 × 0,01, **sem colisão** (é marca de chão, invisível
ao laser — de propósito).

### Obstáculos (cada um com contorno viável)

| obst | perna | eixo | fresta | contorno |
|---|---|---|---|---|
| A | cone 1 → 2 | x = 7,5 | **90 cm** (y 1,80–2,70) | por y > 4,20 |
| B | cone 2 → 3 | y = 4,6 | **70 cm** (x 11,05–11,75) | por x < 9,55 |
| C | cone 3 → 4 | x = 8,2 | **60 cm** (y 7,20–7,80) | por y < 5,40 |
| D | cone 4 → chegada | y = 4,6 | **80 cm** (x 2,20–3,00) | por x > 4,60 |

Blocos com 0,80 m de altura — acima das duas alturas de LiDAR hoje descritas no repo;
confirmar a altura física antes do deploy.
Cada par de blocos é uma **ilha**: a ponta encostada no muro deixa < 0,30 m (vedada),
a outra ponta fica aberta e é o desvio.

Mapa `.pgm`/`.yaml` gerado do mundo e conferido antes de qualquer teste. O
`tools/mapa_passagens.py` atual mede apenas o maior componente conectado após erosão; ele
pode retornar 100% mesmo quando uma fresta local fechou. Estender a ferramenta para receber
as regiões/probes dos quatro corredores e testar a conectividade **local** de cada um.

Também estender `tools/sim_ab/colisao.py`: hoje ele lê somente caixas estáticas, então não
detecta colisão com os cones cilíndricos. O aceite A4 exige distância assinada entre o OBB
do robô e caixas, paredes **e círculos/cilindros**. Só depois dessas duas correções a arena
pode ser declarada conferida.

---

## 6. Fora de escopo até 05/09 (decisão do dono)

> *"acho que não conseguimos nem a rampa e nem o obstáculo movél a tempo... esse trampo é pra meses"*

### 🚫 Rampa 60×60 @ 15° (sobe → plataforma → desce)

Levantado nesta sessão, registrado para não se perder:

- Se 60 cm for a projeção horizontal, a subida é 60·tan(15°) = **16,1 cm**; se for o
  comprimento da rampa, é 60·sen(15°) = **15,5 cm**. Confirmar qual medida foi dada.
- A altura do LiDAR ainda precisa ser medida; não usar 46,5 cm como fato físico.
- Numa inclinação constante, o plano do laser inclina junto com o robô e permanece
  aproximadamente paralelo à própria rampa. A conta `altura/tan(15°) = 1,74 m` não prova
  que ele baterá no chão no meio da subida. Os pontos críticos são as transições de entrada,
  topo e saída, que exigem simulação e leitura de pitch.

Tratamento futuro: medir a geometria, simular as transições e então decidir se é necessário
filtrar scan pelo pitch da IMU e/ou marcar a região como transponível. **Não assumir a
solução antes desses dados.**

### 🚫 Barreira móvel oscilante

Barreira de ~60 cm deslizando numa porta de ~1,2 m; no extremo, sobra janela de 60 cm.
Exige rastrear a barreira, prever a janela, **esperar** e commitar a travessia.
Depende do atalho de fresta estar pronto (a janela é a mesma fresta de 60, porém móvel).

---

## 7. Ordem de trabalho

| # | passo | entrega |
|---|---|---|
| 1 | Definir `nav2_trekking` como fonte oficial e integrar `launch.sh`; medir bitola, entre-eixos e altura do LiDAR | premissas coerentes de runtime e geometria |
| 2 | Corrigir as duas URDFs visuais conforme a medição, sem alterar a bitola efetiva do drive sem calibração | TF/modelo coerentes |
| 3 | Estender `mapa_passagens.py` para probes locais e `colisao.py` para cilindros | validadores capazes de medir A4/A5 |
| 4 | `worlds/arena_galpao.sdf` + mapa + arquivo de missão com frestas, regiões, standoffs e contornos seguros | o campo de prova reproduzível |
| 5 | Baseline Nav2 no sim: validar planos fora de todas as frestas e navegar até os standoffs | prova somente navegação segura; ainda não A1–A5 |
| 6 | Executor que não pula pontos + aproximação pela superfície do cone + autorização TTL no scan | garante A1/A2 sem desativar proteção dos demais obstáculos |
| 7 | Pulso pela interface de LED já existente + CSV | garante A3 |
| 8 | Rodar a missão segura completa, com atalho desligado e colisão zero | gate A1–A5 |
| 9 | Atalho explícito da fresta de 60 cm, condicionado ao teste conjunto de fit | bônus opcional |
| 10 | Deploy na Pi + SLAM e preflight do galpão real | a prova |

Os passos 1–8 entregam a missão. O passo 9 é o bônus e não pode bloquear o 10.

### Gates de conclusão

- Não reutilizar `tools/sim_ab/run_one.sh` com os caminhos fixos de `sala_grande`; aceitar
  world, mapa, rota e diretório de log da arena por parâmetro.
- O mapa só passa quando os quatro probes locais retornarem o estado esperado e houver uma
  rota segura externa a todos eles. Não inferir isso pelo percentual do maior componente.
- A aproximação do cone só passa com distância física entre superfícies ≤ 0,20 m, sem
  contato, em várias execuções e com perda de scan/TF testada.
- A missão segura só passa após A1–A5 completos; a baseline do passo 5 não pode receber
  esse rótulo antes de aproximação e LED existirem.
- O bônus de 60 cm exige repetição nos dois sentidos, com perturbações de pose/yaw/lateral,
  zero contato e nenhum bypass ativo após TTL. Se falhar, continua desligado.
- No galpão metálico, iniciar com `use_imu2_heading:=false`; só habilitar rumo do BNO055
  depois de validar a bússola no local.

---

## 8. Riscos

| risco | mitigação |
|---|---|
| Erros combinados de AMCL, yaw e lateral matam a fresta de 60 | Avaliar a desigualdade conjunta com margem e medir no real. A missão não depende disso (A5). |
| Bug aberto `"goal/start is an obstacle"` gera recovery | Não atribuir automaticamente à inflação. Manter o goal Nav2 no standoff, inspecionar costmap/obstáculo fantasma e usar comportamento dedicado só na aproximação final. |
| Plataforma amarela invisível ao laser | Por desenho: o cone é a âncora, não a plataforma. |
| Hardware do LED ainda não montado | A interface já existe; escolher pino 8 (`/light/marker`) ou relé no pino 7 (`/light/cmd`) conforme a fiação e testar o pulso. |
| `robot_nav` e `nav2_trekking` divergem | Um único pacote deve ser compilado e lançado em sim e no robô; smoke test do launcher antes da arena. |
| Validador ignora cone cilíndrico | Implementar círculo/cilindro vs OBB antes de usar A4 como evidência. |
| SLAM do galpão sai ruim | Conferir as passagens com probes locais e validar os planos seguros **no dia**, antes de rodar. |
| Magnetômetro deriva no galpão metálico | Manter heading magnético desligado até validação no local. |
