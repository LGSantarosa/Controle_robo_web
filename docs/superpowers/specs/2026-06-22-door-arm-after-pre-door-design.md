# Door: armar só DEPOIS do ponto pré-porta cumprido (pendência C)

**Data:** 2026-06-22
**Branch:** `feat/door-para-pra-pessoa`
**Arquivo principal:** `ros2_packages/robot_nav/robot_nav/door_crossing.py`

## Problema (evidência de campo, log /rosout 06-22 noite)

A door arma **por proximidade** (`zone_radius=1.1m` ≥ `standoff=1.0m`), então ela assume o
controle ~0.1m **antes** de o robô chegar no ponto pré-porta. Pior: enquanto o robô ainda
está navegando ATÉ o ponto pré-porta, a door já agarra (`door_vel` fura o nav2), começa as
balizas, não consegue alinhar (ainda longe/torta), estoura o `align_timeout` (15s) e aborta.
O nav2, empurrado pra fora da rota, fica patinando.

Timeline real do log:
- `847` nav2 começa a ir pro ponto pré-porta (2.90, -3.38)
- `921` **door arma** (cedo demais — robô ainda a caminho)
- `936` door aborta (15s do align_timeout)
- `936→994` **robô estático ~58s** (door idle, collision calado, unstuck calado/standdown na zona)
- `997.7` goal do ponto pré-porta **succeeded** (76s depois de a door ter armado!)
- `1000→1016` door arma de novo e **cruza limpo em 16s** ✅

**Conclusão:** quando a door arma na hora certa (depois do pré-porta), funciona ótimo. O
estrago vem do arme prematuro. Fix = só armar depois do ponto pré-porta cumprido.

## Sinal de "pré-porta cumprido"

Os waypoints são despachados **um a um** via `navigate_to_pose` (confirmado no bt_navigator:
`Begin navigating → Goal succeeded → Begin navigating …`). O ponto pré-porta é um goal próprio,
a `standoff=1.0m` do centro, **dentro da zona** (1.1m). Logo:

> **Um goal do nav2 termina com SUCCEEDED enquanto o robô está dentro da zona de uma porta
> marcada ⇒ o robô cumpriu o ponto pré-porta daquela porta.**

Por construção da rota (a web só insere um ponto pré-porta no standoff quando um trecho cruza
uma porta), o único goal que termina dentro da zona é o pré-porta. Sinal preciso e disparado
no instante certo: robô parado no standoff, de frente pra porta.

## Abordagem escolhida: inferência no nó (opção A)

Contida no nó `door_crossing` (zero mudança na web — a pendência B do ponto duplicando segue
aberta e NÃO queremos construir em cima dela). O nó já assina o status dos goals; só passa a
detectar o **sucesso** além do "ativo".

Alternativas descartadas: **(B) web publicar sinal explícito** — mais correto semanticamente,
mas mexe web+nó e esbarra na pendência B; **(C) heurística "parado no standoff"** — "parado" é
ambíguo (o robô se debatendo parece parado → falso-positivo, justo o que queremos matar).

## Design

### Estado novo
- `DoorCrossing` ganha `self._cleared: set[int]` = ids de portas "liberadas pra assumir"
  (cumpriram o pré-porta). Vazio no início.

### Entrada nova em `update()`
- Novo parâmetro `goal_succeeded: bool` — `True` **só no tick** em que um goal do nav2
  transicionou pra `SUCCEEDED` (borda; o nó detecta e entrega como pulso de um tick).

### Lógica (pura, testável)
1. **Liberar:** se `goal_succeeded` e o robô está na zona de uma porta marcada
   (`nearest_door_in_zone(pose, doors, zone_radius)`), adiciona o id dela em `_cleared`.
2. **Gate de arme:** no estado `idle`, o `_pick_door` só devolve uma porta cujo id está em
   `_cleared`. (Os outros gates seguem: `goal_active`, `nav_forward`, bearing, zona, cooldown.)
3. **Reset:**
   - ao concluir a travessia (crossing → idle por `s > exit_margin`): remove o id de `_cleared`.
   - quando o robô **não está na zona de nenhuma porta**: limpa `_cleared` (saiu da região →
     uma próxima aproximação exige cumprir o pré-porta de novo).

### Lado do nó (cola de I/O)
- `_on_status(topic, msg)`: além de `_goal_active[topic]`, detecta a borda pra `SUCCEEDED`
  (status `4`) — i.e., um goal que estava ativo aparece como `SUCCEEDED` no `status_list`.
  Marca um flag de um tick (`self._goal_succeeded_edge = True`).
- `_tick()`: lê e **consome** o flag (zera após passar), repassando como `goal_succeeded` pro
  `update()`.

### Edge case: sem ponto pré-porta
Se não houver pré-porta (bug, ou robô já começou dentro da zona), nenhum goal termina na zona →
nenhuma porta é liberada → a door **nunca arma** → o nav2 passa reto sozinho. **Default seguro,
sem fallback** (YAGNI). Se um dia incomodar, adiciona-se um fallback (parado na zona de frente
pra porta por N s → libera).

## Fluxo de dados

```
navigate_to_pose/_action/status ─▶ _on_status ─(borda SUCCEEDED)▶ _goal_succeeded_edge
                                                                          │
pose (TF) ─┐                                                              ▼
doors ─────┼─────────────────────────────────────────────▶ DoorCrossing.update(..., goal_succeeded)
                                                                          │
                                              _cleared (libera na zona / reset ao cruzar/sair)
                                                                          │
                                                          _pick_door só devolve porta liberada
```

## Testes (pure, sem ROS)

- `goal_succeeded` na zona ⇒ porta entra em `_cleared`.
- `idle` com porta na zona mas NÃO liberada ⇒ NÃO arma (fica idle). ← o bug de campo.
- liberada + demais gates ⇒ arma (idle → rotating).
- `goal_succeeded` FORA da zona ⇒ não libera nada.
- reset ao concluir travessia ⇒ id sai de `_cleared`.
- reset ao sair da zona ⇒ `_cleared` esvazia.
- regressão: a sequência feliz (pré-porta cumprido → arma → cruza) continua passando.

## Não-objetivos
- Pendência B (ponto pré-porta duplicando na web) — separada, não tocada aqui.
- Fallback pra "sem pré-porta" — fora de escopo (YAGNI).
- Mudanças no alinhamento/travessia em si (`commit_s`, escapes, etc.) — já feitas/à parte.
