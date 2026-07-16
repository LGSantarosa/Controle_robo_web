# Cara fase 2 — olhos seguem a pessoa (design)

**Data:** 2026-07-16 · **Aprovado pelo dono:** comportamento "sempre que houver
pessoa" + caminho B (arquivo tmpfs), sem mexer na máquina de humor.

## Objetivo

Quando o motion_guard enxerga um cluster móvel (pessoa) no raio do guard, os
olhos do face_web (iPad 2 no tripé em cima do robô) travam na direção dela.
Sem pessoa, a cara volta ao comportamento atual (olhar vagando).

## Arquitetura (caminho B — arquivo em /tmp)

```
motion_guard (ROS, já calcula cbear_deg p/ CSV)
    └── grava JSON atômico em /tmp/motion_guard_face.json (≤5Hz)
face_app.py (Flask puro, porta 7000, SEM ROS)
    └── GET /state lê o arquivo → {"person": bool, "x": -1..1}
face.js (ES5!, iPad 2)
    └── XHR poll ~300ms → gazeTarget trava na pessoa / libera o vagar
```

Sem dependência nova em lugar nenhum: face_app segue sem ROS, o
face_web.service não muda. Stack caída ⇒ arquivo fica velho ⇒ `/state`
devolve `person:false` ⇒ cara só vaga (degradação limpa).

## Componentes

### 1. motion_guard — escritor do JSON

- No `_on_cmd` (onde o `cbear` já é calculado pro CSV), gravar
  `/tmp/motion_guard_face.json` com `{"ts": t, "cbear_deg": <int|null>}`.
- **Atômico**: escreve em `<arquivo>.tmp` + `os.replace` (leitor nunca vê
  JSON pela metade).
- **Throttle 0.2s** (≤5Hz) enquanto há cluster; na TRANSIÇÃO para
  sem-cluster grava UMA vez com `cbear_deg: null` e para de gravar
  (sem writes contínuos à toa; /tmp na Pi é tmpfs, mas mesmo assim).
- Falha de escrita não pode derrubar o guard: `try/except` engolindo com
  log throttled.

### 2. face_app.py — rota `/state`

- `GET /state` → `{"person": true, "x": 0.42}` ou `{"person": false}`.
- `person:false` quando: arquivo não existe, **mtime mais velho que 1,5s**
  (critério único de stale — o `ts` do JSON é só debug), `cbear_deg` é null,
  ou |cbear| > 90° (pessoa atrás da tela — ninguém vê a cara mesmo).
- **Mapeamento no servidor** (face.js fica burro): `x = clamp(cbear/90, -1, 1)
  * FACE_GAZE_SIGN`. `FACE_GAZE_SIGN = ±1` é constante no topo do
  face_app.py — acertamos o sinal na demo com o dono na frente do robô
  (depende da orientação do iPad no tripé).
- Caminho do JSON configurável por env (`FACE_STATE_FILE`) pra teste.

### 3. face.js — olhar que trava

- Poll XHR a cada **300ms** (ES5: `XMLHttpRequest`, sem fetch/Promise);
  erro/timeout de rede = resposta `person:false` (nunca quebra o loop).
- `person:true` ⇒ `gazeTarget.x = x`, `gazeTarget.y = 0.1` (constante:
  levemente abaixo do centro) e **renova `personHoldUntil = now+3s`**.
- Enquanto `now < personHoldUntil`, o vagar aleatório NÃO mexe no
  gazeTarget (pessoa parada some dos clusters móveis — o hold de 3s evita
  ping-pong olhar-pessoa/olhar-vagando).
- Pessoa vence o humor: o lock de centro do `focused` só vale sem pessoa.
- O lerp existente (0.04/frame) já suaviza — sem animação nova.

## Erros e degradação

| Falha | Efeito |
|---|---|
| stack ROS caída | arquivo stale → `person:false` → cara vaga |
| face_app reiniciado | volta sozinho no próximo poll |
| JSON corrompido/parcial | impossível (rename atômico); parse error → `person:false` |
| XHR falha (WiFi) | callback trata como sem pessoa; próximo poll tenta de novo |

## Testes

- **motion_guard**: JSON gravado com cluster (valor = cbear do CSV),
  throttle 5Hz, transição → uma escrita null, exceção de I/O não propaga.
- **face_app**: `/state` nos 5 casos (sem arquivo, stale, null, pessoa na
  frente, pessoa atrás), sinal do FACE_GAZE_SIGN, respeita FACE_STATE_FILE.
- **face.js**: continua passando no léxico ES5 do test_face_app.py.
- **Manual no dev**: rodar face_app + escrever JSON na mão e ver o olho
  acompanhar no browser.

## Validação no robô (depois, com a Pi de volta)

Deploy = fetch/reset + colcon robot_nav + restart face_web. Demo: dono
anda na frente do robô e confere se o olho segue pro lado certo (senão,
flipa `FACE_GAZE_SIGN`). Só então fecha a fase 2.

## Fora de escopo

Humor pelo estado do robô (gancho continua lá), MODO INTERAÇÃO (virar o
corpo pra pessoa), gaze.y real (altura da pessoa — lidar 2D não sabe).
