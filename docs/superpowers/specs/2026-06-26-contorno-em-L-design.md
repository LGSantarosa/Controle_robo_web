# Contorno em "L" (reta→canto→reta) — decisão de abordagem

**Status:** CHECKPOINT pro dono (sessão autônoma 06-26). NÃO implementado — precisa
da tua escolha de direção antes de tocar o driver ativo (`path_follower`, validado).

## O que o dono quer
Hoje o robô contorna obstáculo/parede numa **diagonal** (volta larga). Ele quer que
isso vire um **"L"**: anda **reto** num eixo, faz **um giro de 90° no lugar** no canto,
anda **reto** no outro eixo. (`reta→canto→reta`.)

## Por que NÃO é "só um simplificador"
- O `path_follower` já faz `reto + giro-no-lugar`, seguindo a FORMA do `/plan`.
- O planner é **Theta\*** (`nav2_params_pi.yaml:85`, any-angle, escolhido em 06-25 pra
  dar retas+point-turn). Theta\* traça o contorno como **UMA diagonal** (line-of-sight,
  menor distância).
- Logo um **Douglas-Peucker / juntar-colineares** sobre o `/plan` **mantém a diagonal**
  (ela já é 2 pontos) — **não vira "L".** Simplificar não cria o canto; o que cria o "L"
  é mudar a ROTA pra **axis-aligned** (Manhattan), o que é **opinativo** (caminho mais
  longo) e muda o que o robô faz.
- Já se sabe (06-26) que `w_traversal_cost` NÃO resolve (só troca "volta larga" por
  "buraco impossível"). Ver `ESTADO_PROJETO.md` seção 06-26.

## Opções (da mais barata/reversível pra mais estrutural)

### A) Theta\* `how_many_corners: 8 → 4`  ⭐ TESTAR PRIMEIRO
- 1 linha em `nav2_params_pi.yaml:86` (`4 = só ortogonal`, já documentado lá).
- **Barato e 100% reversível.** Restringe a expansão a vizinhos ortogonais → enviesa
  pra caminhos ortogonais.
- ⚠️ **Risco:** o passo de **line-of-sight** do Theta\* ainda costura diagonais entre nós
  não-adjacentes → pode continuar cortando o canto. Pode dar "L parcial" ou nada. **Só o
  teste no sim diz.** Custo de testar ≈ zero.

### B) "Manhattan-izar" o /plan dentro do path_follower
- Pós-processa cada segmento diagonal do `/plan` em dois trechos ortogonais (o "L"),
  escolhendo a ordem das pernas que fica **livre de colisão**.
- ❌ **Contra:** pra garantir segurança a perna ortogonal precisa ser checada contra o
  **costmap** (o `path_follower` hoje não lê costmap) — senão a perna do "L" sai do
  corredor seguro do Nav2 e raspa parede. Quebra o princípio "reusar o plano SEGURO do
  Nav2". Mais código + risco no driver ativo. **Evitar, a não ser que A e C falhem.**

### C) Trocar o planner por grid 4-conectado (Smac 2D / A\*) ⭐ FIX ESTRUTURAL se A não bastar
- A fonte axis-aligned correta é o **planner**, não um remendo no seguidor. Um A\*/Smac
  2D com 4-conectividade e **sem smoothing** entrega caminhos ortogonais nativos; o
  `path_follower` atual já os dirige como `reta→canto→reta` sem mudar nada nele.
- Custo: trocar/parametrizar o `planner_server` (config), revalidar no sim. NavFn foi
  rejeitado por ser CURVO (segue gradiente) — Smac 2D puro não é NavFn; é grid-A\*.
- Mantém o driver validado intacto (mexe só no planner) — alinhado com "1 mudança por vez".

## Recomendação
1. **A** (1 linha, sim): `how_many_corners: 4`. Se der o "L" bom → pronto, custo zero.
2. Se A não bastar (LOS ainda corta) → **C** (planner grid-A\*/Smac 2D 4-conn), revalidar
   no sim. **NÃO** mexer no `path_follower` (driver validado) por enquanto.
3. **B** só se A e C falharem (precisa de leitura de costmap no seguidor + testes fortes).

## Verificação (no sim, `--sim --pi --nav2`)
Goal atrás de um obstáculo no `sim_sala`. Sucesso = robô vai reto, **um** giro de 90° no
canto, reto até o goal (sem a diagonal/volta larga). Comparar A vs C lado a lado.
Régua final = validar no real em janela curta.

## Decisão do dono
> Dono escolheu testar **A** e depois **C** (2026-06-27).

## RESULTADOS no sim (2026-06-27) — A e C REPROVADAS
Mapa `sim_sala` construído (tour autônomo, cobre a sala). Forma medida via action
`ComputePathToPose` (start+goal fixos, contorno do box1), sem dirigir.

- **A) Theta\* `how_many_corners` 8→4: REPROVADA.** 4 vs 8 deram caminho quase idêntico
  (mesmo comprimento +25%, curva-total 122° vs 150°, forma visual igual). O passo de
  **line-of-sight** do Theta\* costura diagonal entre nós não-adjacentes → restringir a
  4-vizinhos não cria o canto-90.
- **C) `SmacPlanner2D` (grid A\*): REPROVADA.** O 2D do nav2 é **8-conectado (Moore) FIXO**
  — não há param de 4-conn (VON_NEUMANN); o enum `motion_model_for_search` só aceita `"2D"`.
  Com `smoother.max_iterations: 0` o plano vira **escada de 45°** (17 cantos, curva-total
  765°) — pior que o Theta\*; com smoother on vira diagonal. Nenhum dos dois é o "L".

**Conclusão:** nenhum **planner stock** do nav2 entrega o "L" axis-aligned (todos minimizam
distância → diagonal/escada; o "L" é deliberadamente mais longo). Mantido o Theta\* (diagonal
suave, que o `path_follower` já dirige como reto+point-turn). Caminhos restantes pro "L":
- **B) pós-processar o /plan em pernas ortogonais** (Manhattan-izar) com checagem de costmap.
  Era de-priorizado por exigir leitura de costmap + testes fortes — mas é a via realista que sobrou.
- **D) planner custom com turn-penalty** (penaliza mudança de direção não-axis-aligned).
- **E) reavaliar se o "L" vale o custo** vs aceitar a diagonal suave do Theta\* (que o
  path_follower já discretiza em reto+point-turn).

## B TENTADA E REPROVADA (2026-06-27) ❌ — REVERTIDA
Dono escolheu **B** e eu implementei: nó `plan_manhattanizer` (assina `/plan` Theta* +
`/global_costmap/costmap`; **RDP** colapsava a diagonal ondulada nos cantos reais, depois
trocava cada diagonal por um "L" axis-aligned checando o costmap por perna, fallback =
diagonal; `path_follower` remapeado pra ler `plan_manhattan`). A **forma** ficou certa no
teste estático (contorno do box1: 2 cantos de 90° limpos, livre de colisão; 186 testes ok).

**MAS ao DIRIGIR no sim ficou pior que o anterior** — veredito do dono: "tá uma merda, o
anterior tava bem melhor". O L rígido faz o robô parar-girar-andar-parar-girar (muitos
point-turns secos) em vez de fluir; a diagonal suave do Theta* dirigida pelo `path_follower`
(reto+point-turn discretizado) era visivelmente melhor. **REVERTIDO 2026-06-27**: removidos
o nó `plan_manhattanizer`, os testes, o entry no setup.py e o wiring/remap no `nav2.launch.py`.
Voltou ao estado anterior (Theta* `how_many_corners=8` + path_follower lendo `/plan` direto).

**LIÇÃO (todas as 3 opções de "L" forçado falharam):** A=Theta* 4-conn (line-of-sight
costura diagonal), C=Smac2D (8-conn fixo, escada 45°), B=Manhattan-izar o /plan (forma certa
mas dirige feio). **A diagonal suave do Theta* + path_follower é o melhor que temos** — o "L"
explícito/forçado piora a experiência. Se um dia revisitar, é a opção **E** (aceitar a
diagonal) que vale, não forçar o canto. Não re-tentar A/B/C.
