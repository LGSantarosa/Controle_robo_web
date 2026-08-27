# HANDOFF — nav2_trekking (2026-08-27)

> Estado de quem pega o bastão. Tudo aqui foi **medido no sim**, nada foi ao robô real.
> Branch: **`nav2-trekking`**. Último commit: `6a365d0`.

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

## 2. ⚠️ ESTADO DO CÓDIGO AGORA

**Commitado (`6a365d0`)**: o pacote + a fase "sem medo" (0,30 m/s).

**NO WORKING TREE, NÃO COMMITADO** — a fase velocidade:

| mudança | arquivo | validado? |
|---|---|---|
| `speed_for_clearance` (velocidade por folga) + 5 testes | `path_follower.py` | ✅ **sim** — foi o que fez parar de bater |
| `forward_speed` 0.30 → **0.60** | `path_follower.py` | ✅ sim (2 voltas, 16/16) |
| `max_vel_x` / smoother 0.35 → **0.60** | `nav2_params_pi.yaml` | ✅ sim |
| `PolygonFront` frente 0.40 → **0.50** | `nav2_params_pi.yaml` | ✅ sim |
| harness de A/B | `tools/sim_ab/` | ✅ sim |
| dados das corridas | `log/nav2_trekking_velocidade/` | — |

O `path_follower` está no **último estado validado**. Duas tentativas de melhorar
as curvas foram **testadas e revertidas** (ver §5); os comentários no código
guardam o porquê.

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

**Ganho acumulado: 791,6 s → 621,6 s = 21% mais rápido, com 16/16 goals.**

CSVs em `log/nav2_trekking_velocidade/` (um por configuração + os `*_colisao.csv`
brutos, que são a prova ponto a ponto da folga contra as paredes).

---

## 4. A configuração que funciona hoje

- **`collision_monitor`**: UMA zona `PolygonFront`, modo **`limit`**,
  caixa `x 0.25..0.50 × |y| <= 0.22`, `linear_limit 0.0`, `angular_limit 4.0`.
- **`inflation`**: global **0.45**, local **0.35**.
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

Do histórico antigo do arquivo (também não repetir): `turn_exit` 3° → 7° gera
dente-de-serra (sair torto → anda em diagonal → estoura o enter → mais giros);
`lookahead` 0.4 e "mira longe" pioraram em 06-27/28.

---

## 6. ⏳ Próximos passos (em ordem)

1. **HIPÓTESE ABERTA: `lookahead` 0.6 → 1.0.** Como banda e filtro falharam, o
   quadro aponta para **ganho de laço** (pure-pursuit com carrot curto oscila:
   corrige o rumo, segue deslocado de lado, cruza a linha, gira pro outro lado).
   Já foi revertido em 06-27/28 por "raspar" — **mas ali não havia medição**.
   Hoje o `colisao.py` mede a folga em mm: dá pra saber se raspa e quanto.
   Critério: comparar com 621,6 s / 16-16 / folga mínima 6-8 mm.
   *(Foi aplicado e revertido antes de rodar — a sessão acabou.)*
2. Se o lookahead não resolver: **suavizar o plano**. O `nav2_smoother` **não
   está no launch** e o BT não chama `SmoothPath` — o plano do Theta* chega
   quebrado e o carrot persegue os cantos. É a abordagem estrutural.
3. **Degrau 0,90 m/s** (o dono quer o mais rápido possível). ⚠️ A `PolygonFront`
   é caixa FIXA e **não escala**: a 0,90 a conta de parada é 0.27 + 0.09 = 0.36 m,
   então ela teria que ir a ~0.65 — e aí passa a pegar obstáculo que o robô ia
   contornar. É o teto prático deste desenho.
4. **Girar colado numa parede segue desprotegido** por desenho (o canto varre
   0.354 m; o reflexo é estreito e frontal). Lugar certo: o `path_follower`
   checar o anel 0.25..0.36 m antes de iniciar point-turn.
5. **Margem lateral**: a 0,60 m/s ele navega **colado** — passou ~1 s inteiro a
   **6-8 mm** de `wall_10` e `wall_12`. No sim "não bate"; no real, com erro de
   AMCL + derrapagem, isso É batida. Atacar antes de ir pro robô.

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
