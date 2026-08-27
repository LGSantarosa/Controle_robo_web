# HANDOFF — nav2_trekking (2026-08-27)

> Estado de quem pega o bastão. Tudo aqui foi **medido no sim**, nada foi ao robô real.
> Branch: **`nav2-trekking`**. Último commit: `e03555b`.

---

## 1. O que é isto

O dono decidiu: **o trekking parou; quem substitui é o nav2.** Contexto dele:

> "a pista terá vários obstáculos, será em um galpão fechado, o lidar consegue
> criar o mapa pro nav2 seguir e o nav2 está ÓTIMO em movimentação, OTIMO, então
> só precisamos ajeitar o medo dele e a velocidade"

E, crucialmente (define o critério de sucesso):

> "ele não pode desistir dos goals, pois serão goals que ele DEVE chegar pra
> tirar o tempo da competição"

Logo: **8/8 goals sempre, zero colisão, e o tempo é a nota.**

`ros2_packages/nav2_trekking/` é uma **cópia integral** do `robot_nav`, renomeada
por dentro. Os dois convivem (colcon acha os dois); **nenhuma linha do robot_nav
foi tocada**. `wheel_msgs` é compartilhado de propósito.

---

## 2. ESTADO DO CÓDIGO AGORA

**Tudo commitado. Working tree limpo.** Nada pendente de deploy.

| commit | o que entrou |
|---|---|
| `6a365d0` | o pacote `nav2_trekking` + fase 1 "sem medo" (0,30 m/s) |
| `80d9f35` | fase 2 velocidade: `forward_speed` 0.60, `speed_for_clearance`, `PolygonFront` 0.50, harness `tools/sim_ab/`, CSVs |

O `path_follower` está no **último estado validado** (`lookahead` 0.6). **Três**
tentativas de melhorar as curvas foram testadas e revertidas (`turn_enter` 24°,
`aim_tau` 1.6, `lookahead` 1.0 — ver §5); os comentários no código guardam o
porquê e os números.

⚠️ **Nada disto foi ao robô real ainda.**

---

## 3. Resultados medidos (rota1 + `sala_grande`, 8 goals)

| configuração | voltas | goals | tempo médio | v média | colisões |
|---|---|---|---|---|---|
| baseline `robot_nav` (medroso) | 1 | 7/8 | 791,6 s | 0,177 | — |
| caixa fixa `stop` | 1 | 8/8 | 800,4 s | 0,175 | — |
| `approach` 0.3 | 1 | 6/8 | 926,7 s | 0,164 | — |
| `limit` + inflation 0.35 | 1 | 6/8 | 752,2 s | 0,182 | — |
| **`limit` + inflation 0.45 (fase 1 final)** | **4** | **32/32** | 787,7 s | 0,184 | — |
| **0,60 m/s (fase 2 atual)** | **2** | **16/16** | **621,6 s** | **0,237** | 1 toque |
| 0,60 + `aim_tau` 1.6 (revertido) | 1 | 8/8 | 685,2 s | 0,220 | 1 toque |
| 0,60 + `lookahead` 1.0 (revertido) | 1 | 8/8 | 597,9 s | 0,236 | **11 colisões** |
| 0,60 + `rot_min` 3.0 (revertido) | 1 | 8/8 | 649,1 s | 0,241 | 1 toque |
| **0,60 + raio 0,32 + inflation 0,60 (ATUAL)** | **2** | **16/16** | **654,3 s** | **0,233** | **ZERO** |

**Ganho acumulado vs BASELINE (nav2 padrão, `robot_nav`): 791,6 s → 654,3 s =
17% mais rápido, 7/8 → 16/16 goals, e ZERO contato com parede em 2 voltas.**

⚠️ **"baseline" = o nav2 PADRÃO (791,6 s, 7/8).** Não usar a palavra pros degraus
intermediários nossos — confundir os dois inverte o sinal da notícia (a fase de
velocidade, 621,6 s, é 5% mais rápida no relógio mas é a que RASPA a 1 mm).

⚠️ **Comparar v média, não relógio bruto.** O planner sorteia rota diferente a
cada volta: 140,1 a 156,5 m entre as runs (11% de spread). O relógio bruto é
contaminado pela rota; `dist/tempo` é a métrica honesta.

CSVs em `log/nav2_trekking_velocidade/` (um por configuração + os `*_colisao.csv`
brutos, que são a prova ponto a ponto da folga contra as paredes).

---

## 4. A configuração que funciona hoje

- **`collision_monitor`**: UMA zona `PolygonFront`, modo **`limit`**,
  caixa `x 0.25..0.50 × |y| <= 0.22`, `linear_limit 0.0`, `angular_limit 4.0`.
- **`costmap`**: `robot_radius` **0.32** (era footprint quadrado ±0.25 = inscrito
  0.25, e o canto em diagonal alcança 0.354 → passava colado por desenho).
- **`inflation`**: global **0.60**, local **0.45**.
- **`unstuck`**: `stuck_timeout` 2.0 / `mapped` 1.0 / `rear_half_width` **0.26**.
- **`path_follower`**: `forward_speed` **0.60**, `lookahead` 0.6,
  `turn_enter` 16°, `turn_exit` 3°, `aim_tau_short` 0.8,
  **`clear_full` 1.2 / `clear_min` 0.35** (velocidade por folga).
- `motion_guard` **removido**; `nav2_params_legacy.yaml` **apagado**.

---

## 5. ⭐ O que já foi testado e NÃO funciona (não repetir)

**Reflexo de colisão** — os 3 modos têm defeitos opostos:
- **caixa fixa `stop`**: 38 paradas / 60,3 s, TODAS com a frente livre
  (`min_front` >= 0.41 m). Quem entrava na caixa era a **parede do lado** a
  0.275 m. Caixa fixa não gira com o robô: não distingue "passo reto ao lado"
  (seguro) de "giro colado" (o canto varre 0.354 e bate).
- **`approach`**: escala o twist INTEIRO, `wz` junto → `follow_vel` 2.40 saía
  0.00/-0.96/-1.12, **abaixo da zona-morta 1.7** → rodas patinam, não vira.
  É o deadlock do point-turn, reproduzido mesmo com `time_before_collision` 0.3.
  O unstuck manobrou 217 s e a volta perdeu 2 goals.
- **`limit`** (o que ficou): corta **por eixo**, giro nunca capado.

**Medo ≠ folga**: baixar a `inflation` junto com o reflexo fez o robô passar a
0.245 m das paredes (< raio inscrito 0.25) → a célula dele vira custo letal →
`"Either of the start or goal pose are an obstacle"` **19x numa volta** → o
planner para de replanejar e o BT afunda em recovery. Medo no reflexo **para** o
robô (custa tempo); folga no plano faz ele **contornar andando** (não custa).

**Curvas / minicurvinhas** (pedido do dono: "se ele girar menos vezes vai mais
rápido"). Medido a 0,60 m/s: **39% da volta girando parado**, 126 giros em 3 min,
giro médio 13,1°, e **72% do giro se cancela** (1275° brutos → 362° líquidos).
Duas tentativas, ambas revertidas:
- `turn_enter` 16° → 24°: giros/min −30%, mas cada giro proporcionalmente maior
  → **tempo girando igual** (39% → 42%), volta mais lenta, robô mais torto.
- `aim_tau_short` 0.8 → 1.6: **piorou** — cancelamento 72% → **87,8%**, volta
  +9%. EMA forte em malha fechada = atraso de fase: gira por mira defasada,
  passa do ponto, volta.

**`lookahead` 0.6 → 1.0** (era a hipótese #1 aberta) — **testado 2026-08-27 com
medição, REFUTADO**. Ver `log/nav2_trekking_velocidade/v060_lookahead10.csv`:
- **Não é mais rápido.** 597,9 s contra 621,6 s, mas a volta ENCURTOU (141,2 m
  contra 147,7): a v média é a **mesma** (0,236 vs 0,237). Ele não andou mais
  rápido, andou menos — **cortando canto, colado na parede**.
- **Preço**: **11 colisões + 17 raspões numa volta** (o 0.6 fez 1 e 3 em DUAS),
  21 s em contato, folga mínima **−1 mm**. Bateu em `wall_7` e `wall_10`.
- **Efeito colateral no planner**: 16 `failed to plan` + 3 recoveries (o 0.6 fez
  0,5 e 0) — colado na parede a célula do robô vira custo letal, o mesmo
  mecanismo do "medo ≠ folga" acima.
- **E não curou o que motivou o teste**: cancelamento de giro **72% → 87,5%**
  (mesma janela de 3 min), giro bruto igual (1275° → 1230°), líquido 362° → 153°.
  **Mesma assinatura do `aim_tau` 1.6** (87,8%).

⭐ **A conclusão que sobra**: alongar a mira e filtrar a mira falham do MESMO
jeito, logo o problema **não é ganho de laço nem ruído** — é o **plano**. Vai
para o item 1 dos próximos passos.

Do histórico antigo do arquivo (também não repetir): `turn_exit` 3° → 7° gera
dente-de-serra (sair torto → anda em diagonal → estoura o enter → mais giros);
`lookahead` 0.4 e "mira longe" pioraram em 06-27/28.

---

## 6. ⏳ Próximos passos (em ordem)

1. **AFROUXAR O `speed_for_clearance`** — é onde está o tempo. Medido na volta:
   de 613 s de seguidor, **403 s dirigindo** e **211 s parado girando (34%)**; e
   das 403 s dirigindo, só **51% a 0,55-0,60** — **28,6% rastejando a 0,25-0,40**
   (média comandada 0,493 contra `forward_speed` 0,60). Esse freio foi criado
   pra compensar o robô passar COLADO. Agora o plano garante 3-8 cm de folga:
   ele cobra duas vezes pela mesma segurança. Subir `clear_min` / baixar
   `clear_full` e medir — a folga real (colisao.py) diz na hora se passou do
   ponto. É o único lugar onde "menos cuidadoso = mais rápido" ainda tem espaço.
2. **BUG ABERTO: "start pose is an obstacle" a 42-72 cm de parede.** Longe do
   raio 0.32 E da inflação (que ali dá custo ~93, contra 253 de letal). Não é
   regressão do fix de margem: acontecia na config velha e PIOR (16 na volta
   `la10_1`, 0 na `rot30_1`, 5 na `r32_1`) e acompanha o robô andar colado.
   Suspeita: `obstacle_layer` marcando fantasma ou falhando em limpar marcação
   velha. É QUEM GERA OS RECOVERIES — e recovery é tempo perdido puro.
3. **SUAVIZAR O PLANO.** O `nav2_smoother` não está no launch e o BT não chama
   `SmoothPath`: o Theta* entrega o caminho quebrado e o carrot persegue os
   cantos. As QUATRO tentativas de resolver as curvas no SEGUIDOR falharam
   (`turn_enter`, `aim_tau`, `lookahead`, `rot_min` — ver §5); o que resta é a
   fonte. Ataca os 34% da volta girando parado.
4. **Degrau 0,90 m/s.** ⚠️ A `PolygonFront` é caixa FIXA e não escala: a 0,90 a
   conta de parada é 0.36 m, então ela iria a ~0.65 e passaria a pegar obstáculo
   que o robô ia contornar. Teto prático deste desenho.
5. **Girar colado numa parede** segue desprotegido no reflexo (o canto varre
   0.354; a caixa é estreita e frontal). Com o raio 0.32 no costmap o problema
   ficou MUITO menor (o plano não o leva mais pra lá), mas o lugar certo do
   cinto continua sendo o `path_follower` checar o anel antes do point-turn.

---

## 7. Como rodar (harness em `tools/sim_ab/`)

```bash
export SIM_AB_DIR=~/Workspace/Controle_robo_web/log/sim_ab   # onde caem as voltas
mkdir -p $SIM_AB_DIR
cd ~/Workspace/Controle_robo_web
bash tools/sim_ab/run_n.sh nav2_trekking <prefixo> 3     # 3 voltas, desanexado
python3 tools/sim_ab/consolida.py <rotulo> <prefixo>1 <prefixo>2 <prefixo>3
```

- `run_n.sh` tem **lock**: subir outro mata o anterior (dois runs = dois Gazebos).
- `kill_all.sh` limpa tudo, **inclusive os próprios scripts** (probe/colisao) e
  os `teleop_twist_joy`/`joy_node` que o `sim.launch` sobe — um teleop órfão
  publica `joy_vel`, **prioridade 100 no twist_mux**, e sequestra o robô.
- `colisao.py` é o detector de colisão real (ground truth do Gazebo, SAT entre o
  retângulo do robô e as 20 caixas do mundo). **Não confiar em `min_scan`**: o
  laser fica no centro, 0.25 m tanto pode ser "passei raspando" quanto "encostei".

### 🐞 Bugs conhecidos DO HARNESS (não do robô)

- **Bringup do nav2 trava**: se o nav2 sobe em cima do pico de CPU do Gazebo, o
  `map_server` demora a responder o `change_state`, o `lifecycle_manager`
  desiste e a fila para (`map_server` inactive, resto unconfigured) — **o robô
  fica parado sem publicar nada**. Mitigado com 15 s de espera + retry 3x, mas
  **voltou a acontecer** no fim da sessão. Se o robô "não se mexe", cheque
  `ros2 lifecycle get /bt_navigator` ANTES de suspeitar do robô.
- Sempre confirmar com `ps -eo pid,args | grep -E "run_n|gz sim"` que há **um
  só** de cada.

---

## 8. Erros meus nesta sessão (para não repetir)

1. Medi **`min_scan` como se fosse segurança** — é proxy. Só criei detector de
   colisão real quando o dono viu o robô bater e perguntou se eu estava medindo.
2. **Cortei medo e folga juntos** (reflexo + inflation) e quebrei o planner.
3. Deixei **processos órfãos** contaminarem medições: um `probe.py` de uma
   tentativa abortada disputou o `bt_navigator` com o novo (3 goals perdidos), e
   relancei o `run_n` sem matar o anterior (**dois Gazebos**, o dono viu na tela).
4. Baixei `stuck_timeout` para 0,3 s a pedido sem pensar no efeito: **cortava
   point-turn legítimo** (girando no lugar o robô não translada → parece
   encalhe). Era o "não identifica que chegou no ponto" que o dono viu.
5. Propus `turn_exit` e `lookahead` **sem ler o histórico do próprio arquivo**,
   onde as duas já estavam documentadas como revertidas.
