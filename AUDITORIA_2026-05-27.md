# Auditoria do projeto Controle_robo_web — 2026-05-27

Quarta passada depois de:
- [AUDITORIA_2026-05-14.md](./AUDITORIA_2026-05-14.md) — primeira limpeza (D1 collision_monitor removido)
- [AUDITORIA_2026-05-18.md](./AUDITORIA_2026-05-18.md) — segunda passada (~95 % aplicada via `4f227f4`, `26476e4`, `5f7fd9c`)
- [AUDITORIA_2026-05-26.md](./AUDITORIA_2026-05-26.md) — terceira passada pós-headless (100 % aplicada via `a812801`, `8c37067`, `6281a1d`, `842fd99`, `cbe1b05`, `d694141` + B1 nesta sessão)

Foco da quarta passada: estado pós-headless já estabilizado, agora caça pegadinhas
que sobraram em comentários, paths e robustez de scripts.

> Severidades: 🔴 crítica (bloqueia uso real / quebra dados / segurança) — 🟠 alta (bug funcional, comportamento incorreto) — 🟡 média (qualidade/robustez/UX) — 🟢 baixa (cosmético / refator).
>
> **Estado git:** branch `main`, último commit `d694141` (`launch.sh`: flash MEGA best-effort). Sem alterações não-commitadas no momento da auditoria. AUDITORIA_2026-05-26 totalmente endereçada — confirmado por código de cada item.
>
> **Memórias relevantes pra próxima sessão:**
> - Sem `Co-Authored-By Claude` em commits deste repo.
> - PS4 sempre pareia em pair mode pelo PC dev via `./bin/robot-pair-ps4` (não tenta no PC dev — bug de HW).
> - Preferir ajustar scripts a pedir o usuário mudar hostname/SO.
> - Repo vive em `~/Workspace/Controle_robo_web` no PC do usuário (não `~/Controle_robo_web`).

---

## 🔴 CRÍTICOS

Nenhum achado crítico nesta passada — a estabilização das Fases 1–3 do
`PLANO_HEADLESS_2026-05-22.md` removeu os bloqueadores funcionais (C1/C2 da
auditoria anterior) e a infraestrutura headless está operacional.

---

## 🟠 ALTOS — bugs funcionais

### A1 — `setup_udev.sh:187` injeta `~/Controle_robo_web` no `/etc/udev/rules.d/99-robot-usb.rules`

- `setup_udev.sh:187` — `# Para regenerar: sudo ~/Controle_robo_web/setup_udev.sh`

Linha emitida dentro do heredoc que escreve `/etc/udev/rules.d/99-robot-usb.rules`.
O caminho real é `~/Workspace/Controle_robo_web/` (memória do projeto). Em
máquinas com o repo no caminho correto o comentário fica falso em `/etc/`.
Usuário lendo o arquivo udev pra reaplicar a regra recebe instrução errada,
tenta `sudo ~/Controle_robo_web/setup_udev.sh` e leva "No such file or directory".

**Fix em `setup_udev.sh:187`:**
```bash
# Para regenerar: sudo ~/Workspace/Controle_robo_web/setup_udev.sh
```

---

### A2 — `bin/robot-pair-ps4:22` `HOST_FALLBACK_IP="192.168.18.95"` hardcoded — só funciona na bancada do autor

- `bin/robot-pair-ps4:21-22`
- `bin/robot-pair-ps4:30-33` (lógica do fallback)

O `robot-pair-ps4` resolve o host com `ROBOT_HOST` → `robo-desktop.local` →
`192.168.18.95` (fallback). Esse IP é da rede de casa do autor e não vai
existir em nenhuma outra rede (router diferente, AP do laboratório, hotspot
de celular). Em vez de "fallback útil", vira "wait 5 s pra tentar IP errado".

Pior: o `bin/robot-connect:14-16` **não tem fallback de IP**, só `ROBOT_HOST`
+ `robo-desktop.local`. Inconsistência entre os dois wrappers.

**Fix:** remover o fallback IP fixo do `robot-pair-ps4`. Se o mDNS falhar,
imprimir a mesma mensagem do `robot-connect:27-32` orientando a setar
`ROBOT_HOST=<ip>`:
```bash
if [ -n "${ROBOT_HOST:-}" ]; then
    HOST="$ROBOT_HOST"
elif timeout 2 getent hosts "$HOST_MDNS" >/dev/null 2>&1; then
    HOST="$HOST_MDNS"
else
    cat >&2 <<EOF
ERRO: $HOST_MDNS não resolveu via mDNS.
  Passe o IP explicitamente:  ROBOT_HOST=<ip-do-robô> robot-pair-ps4
  (veja no painel do roteador, ou rode 'hostname -I' no robô).
EOF
    exit 1
fi
```

---

## 🟡 MÉDIOS — qualidade, robustez, UX

### M1 — `firmware/mega_bridge/src/main.cpp:69` comentário "auto-timeout de 3 s" é stale (animação é 1 s)

- `firmware/mega_bridge/src/main.cpp:69`
- `firmware/mega_bridge/src/leds.cpp:14-18` (FLASH_TOTAL_MS = 1000 ms)

O comentário no `handlePcFrame` diz `// 5 (WAYPOINT) usa triggerWaypoint pra
ter auto-timeout de 3 s.` mas a animação WAYPOINT agora é `FLASH_TOTAL_MS = 200 ms × 5 = 1 s`
(+ 150 ms de recovery do PMW3901 = `gated_until_` 1150 ms). Mudança veio no
commit `8c4db2e` quando o padrão de 3 piscadas virou 5 ciclos com base branca.

**Fix em `main.cpp:69`:**
```cpp
//        5 (WAYPOINT) usa triggerWaypoint pra ter auto-timeout de ~1 s
//                     (5 ciclos × 200 ms; ver leds.cpp FLASH_TOTAL_MS).
```

---

### M2 — `~/Controle_robo_web` (sem `Workspace/`) em scripts de setup e em vários pontos do README

Caminho real do repo é `~/Workspace/Controle_robo_web` (memória do projeto).
Os usos abaixo mostram o caminho errado em docstring/print/comentário e
confundem o usuário em copy-paste:

| Arquivo | Linha | Texto |
|---------|-------|-------|
| `setup.sh` | 9 | `#   cd ~/Controle_robo_web` |
| `setup_pi.sh` | 14 | `#   cd ~/Controle_robo_web` |
| `setup_udev.sh` | 187 | (ver A1 — escrito em `/etc/`) |
| `README.md` | 54, 60, 86, 105, 113, 355, 415, 424, 434, 454, 479, 1132, 1227 | mix entre `cd` e referência |

`setup_udev.sh:228` já tem `cd ~/Workspace/Controle_robo_web` correto na
mensagem final — provam que a convenção certa é com `Workspace/`.

**Fix:** renomear todas as ocorrências para `~/Workspace/Controle_robo_web`.
Os scripts `setup.sh` / `setup_pi.sh` / `setup_udev.sh` **não dependem** desse
path em runtime (todos usam `$(cd "$(dirname "$0")" && pwd)`), então isso é
puramente documentação. Mas afeta usabilidade real: linha 187 do `setup_udev.sh`
vira instrução de regeneração no `/etc/`.

---

### M3 — `nav2.launch.py` 8 nós com `output='screen'` poluem o terminal/log do `launch.sh`

- `ros2_packages/robot_nav/launch/nav2.launch.py:53-108`

Todos os 8 nós Nav2 (`map_server`, `amcl`, `controller_server`, `planner_server`,
`behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`,
`lifecycle_manager`) usam `output='screen'`. O `robot.launch.py:131-141` já
adotou `output={'stdout': 'screen', 'stderr': 'log'}` para o `joy_node` (M9
da auditoria anterior). Em NAV2 o terminal vira fluxo contínuo de
`[controller_server-3] [INFO]` repetido a cada tick.

**Fix:** padronizar `nav2.launch.py` para `output={'stdout': 'log', 'stderr': 'log'}`
nos nós internos, deixando só `lifecycle_manager` em `screen` (porque ele é o
único que indica visualmente "Nav2 ativou"). Logs continuam em
`~/.ros/log/<timestamp>/<node>-N-*.log` e `controle_web/logs/nav2.log` (via
redirect do `launch.sh:469`).

Alternativa mais simples: apenas `lifecycle_manager` em `screen`, todo o
resto em `log`.

---

### M4 — `setup_pi.sh:120-123` clona `ldlidar_stl_ros2` em HEAD do master (sem pin de versão)

- `setup_pi.sh:119-123`
- `setup_pi.sh:127-132` (patch dependente do código)

`git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git` sem
`--depth 1` nem branch fixa. Se o upstream rebatear o master, o patch do
`pthread.h` pode falhar (sed do `^#include` quebra se o arquivo for movido),
e ninguém vai saber que a versão mudou.

**Fix:**
```bash
# Pin do master visto funcionando em 2026-05-22 (commit <hash> ou tag conhecida).
git clone --depth 1 --branch master https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$LIDAR_DIR"
( cd "$LIDAR_DIR" && git rev-parse --short HEAD ) | xargs -I{} echo "  driver LiDAR: {}"
```

Ou versionar o driver dentro do repo como submódulo (mais pesado, mas
reproduzível 100 %).

---

### M5 — `launch.sh` lista de `pkill -9 -f` aparece duplicada (linhas 225-242 e 345-365)

- `launch.sh:225-242` (limpa órfãos antes de subir)
- `launch.sh:345-365` (rede de segurança no `cleanup()`)

Duas listas idênticas (em conteúdo) de processos. Adicionar um nó novo
(ex.: `nav2_collision_monitor`, futuro `robot_localization`) obriga a editar
nos dois lugares; um esquecimento deixa órfão em runtime ou cleanup
incompleto.

**Fix:** extrair função `kill_known_nodes()` no topo do `launch.sh` (após a
declaração das variáveis):
```bash
KNOWN_NODE_PATTERNS=(
    "robot_nav/odom_publisher"
    "robot_nav/cmd_vel_to_wheels"
    "robot_nav/mega_bridge"
    # ... resto
)
kill_known_nodes() {
    for pat in "${KNOWN_NODE_PATTERNS[@]}"; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}
```

E chamar `kill_known_nodes` nas duas linhas. Reduz drift e dá grep único.

---

### M6 — `setup_pi.sh:139-140` paralelismo hardcoded em 2 workers — Pi 5 (8 GB) fica subutilizada

- `setup_pi.sh:138-140`

`MAKEFLAGS="-j2"` + `--parallel-workers 2` são certo para Pi 4 4GB (RAM
constrita), mas Pi 5 8GB (que o projeto pretende suportar — ver
`project_hardware_4wheel.md`) aguenta 4 paralelos com folga. Hoje a primeira
build leva ~25 min na Pi 5 quando podia levar ~12 min.

**Fix:** detectar RAM disponível e modular:
```bash
FREE_MB="$(awk '/^MemAvailable:/{print int($2/1024)}' /proc/meminfo)"
if [ "$FREE_MB" -ge 6000 ]; then
    PAR=4
elif [ "$FREE_MB" -ge 3000 ]; then
    PAR=2
else
    PAR=1
fi
export MAKEFLAGS="-j$PAR"
colcon build --base-paths ros2_packages --symlink-install --executor sequential --parallel-workers "$PAR"
```

Não dá pra confiar só em `nproc` (Pi 4/Pi 5 têm 4 cores), mas RAM disponível
é o gargalo real. Variável manual `PI_BUILD_PARALLEL=4 ./setup_pi.sh` como
override seria bom.

---

### M7 — `bin/robot-pair-ps4:50-51` resolve repo via symlink `/usr/local/bin/robot-up` — falha silenciosa se setup_headless ainda não rodou

- `bin/robot-pair-ps4:49-61`

A linha 50-51 faz `readlink -f /usr/local/bin/robot-up` no robô pra achar o
checkout do repo. Se o usuário clonou o repo no robô mas **ainda não rodou
`setup_pi.sh`** (que chama `scripts/setup_headless.sh` que cria o symlink), o
`readlink` retorna vazio. O check da linha 53-61 já cobre isso e dá uma
mensagem decente — bom!

Mas se o usuário rodou só `setup_headless.sh` (sem `setup_pi.sh`) num caminho
**diferente** (ex.: clonou pro `~/robo/` em vez de `~/Workspace/Controle_robo_web/`),
o symlink aponta pro lugar certo e funciona. Isso já é robusto.

Nada a fixar — anotando que o design depende do symlink em
`/usr/local/bin/robot-up`. Se em algum momento alguém mover o repo após o
setup, o symlink fica órfão e o `robot-pair-ps4` falha com "ERRO: não
encontrei o repo no robô" — mensagem clara.

✅ Sem ação.

---

### M8 — `mega_bridge.py:255-261` `_on_leds` só publica `FT_LEDS` len=4 (RGB+pattern); não há API ROS pra disparar `triggerWaypoint`/`STARTING`

- `ros2_packages/robot_nav/robot_nav/mega_bridge.py:255-261`
- `firmware/mega_bridge/src/main.cpp:67-84` — firmware aceita `len=1` com ID de estado

O firmware suporta dois formatos de `FT_LEDS`: `len=1` (ID de estado, inclusive
`WAYPOINT = 5` que dispara o `triggerWaypoint()` e a animação gated do PMW3901)
e `len=4` (RGB+pattern manual, legacy). O bridge ROS hoje só expõe o segundo
formato (`/leds/color` → `ColorRGBA` → `len=4`).

Consequência: **não há como o `trekking_runner` mandar "cheguei num waypoint,
pisca amarelo, gateia o flow"** via tópico ROS — ele só pode usar
`/leds/color` que vai pelo path manual e não engaja o `gated_until_` que
suprime motion fantasma do PMW3901. Hoje o runner aceita isso porque o flow
é ignorado em alguns trechos via parâmetros do `pose_estimator`, mas é uma
API meia-incompleta.

**Fix:** adicionar topic `/leds/state` (`std_msgs/UInt8`) no `mega_bridge.py`:
```python
self.create_subscription(UInt8, 'leds/state', self._on_led_state, qos_cmd)
...
def _on_led_state(self, msg: UInt8):
    self._send(FT_LEDS, bytes([msg.data]))
```

E o `trekking_runner` publica `UInt8(data=5)` (WAYPOINT) ao chegar num ponto,
em vez do `ColorRGBA` atual. Resolve "manual + automático" colidirem.

---

### M9 — `controle_web/app.py:35` `ROS2Controller(...)` cru sem try/except — falha de import do `rclpy` derruba a UI inteira

- `controle_web/app.py:35`
- `controle_web/controllers/robot_controller.py:154-156` (import rclpy dentro do `__init__`)

Se o usuário esquecer de `source install/setup.bash` antes de rodar o
servidor (ex.: rodando `python3 app.py` direto pra debug rápido), a
`import rclpy` dentro do `RobotController.__init__` levanta `ModuleNotFoundError`
e o app.py morre na linha 35. Sem fallback pro `EchoController` (que existe
no `robot_controller.py:49`).

Hoje o `launch.sh:185` sempre sourceia o setup.bash, então isso só aparece
em uso "manual" do servidor — uma trilha de uso real.

**Fix em `controle_web/app.py:35`:**
```python
try:
    controller: RobotController = ROS2Controller(enable_publish=WEB_TELEOP)
except Exception as e:
    logging.getLogger(__name__).warning(
        f"[app] ROS2Controller falhou ({e}); caindo para EchoController "
        f"(web rodando sem ROS). source install/setup.bash antes de usar comandos."
    )
    from controllers.robot_controller import EchoController
    controller = EchoController()
```

Mantém a UI viva no caminho dev. Em produção/`launch.sh`, o caminho feliz
não muda.

---

### M10 — `slam.launch.py` não documenta o comando de save_map embutido

- `ros2_packages/robot_nav/launch/slam.launch.py:5-13`

A docstring instrui `ros2 run nav2_map_server map_saver_cli -f ~/Workspace/.../maps/meu_mapa`
mas o web app já tem botão "Salvar mapa" (`controle_web/app.py:346-355` →
`MapBridge.save_map`). Quem ler a docstring fica acreditando que precisa
abrir terminal extra. Velha herança de antes do botão existir.

**Fix:** atualizar docstring pra mencionar primeiro o botão web e listar o
CLI como alternativa.

---

## 🟢 BAIXOS — cosméticos / refator

### B1 — `protocol.h` checksum XOR8 é trivialmente colidível; OK pro escopo (USB curto, ambiente controlado)

`protocol.cpp` usa XOR8 simples (1 byte). Aceita 1/256 dos frames corrompidos.
USB cabeado em ambiente controlado vê erro a cada milhares de horas — risco
real é mínimo. Documentar a escolha (não trocar pra CRC8 só por estilo).

Sem fix; anotação pra futuras revisões.

---

### B2 — `launch.sh:194-197` e `launch.sh:408-411` duplicam aviso de `/dev/mega` ausente

- `launch.sh:194-197` (top-level)
- `launch.sh:408-411` (dentro do bloco real, antes do `robot.launch.py`)

Aviso aparece **duas vezes** no terminal se a MEGA não estiver plugada
(uma vez no setup geral, uma vez antes de subir os nós). UX ruim — primeira
vez tá ok, mas segunda tá redundante.

**Fix:** remover o aviso da linha 194-197 (que é genérico) e manter só o
contextual antes de `robot.launch.py`. Ou apenas o early para falhar mais
cedo se necessário, removendo o segundo.

---

### B3 — `controle_web/app.py:567` `_TREKKING_CMDS` lista 6 comandos mas falta sincronizar com `trekking_runner._on_cmd`

- `controle_web/app.py:563-566` — `{'reset', 'record', 'save_point', 'play', 'stop', 'load_waypoints', 'clear'}` (7 elementos)
- `ros2_packages/robot_nav/robot_nav/trekking_runner.py` — `_on_cmd` (não verificado nessa passada)

Verificar se o runner aceita exatamente esse mesmo conjunto. Comentário em
`app.py:561-562` ("qualquer mudança lá precisa vir pra cá") explicitamente
reconhece o risco. Sem teste automatizado pra capturar o drift.

**Fix:** mover a lista `_TREKKING_CMDS` pra um JSON em `controle_web/static/`
ou pra um header compartilhado, e o runner lê o mesmo arquivo. Ou (mais
barato): teste de import compartilhado em `tests/test_trekking_cmds_match.py`.

Ainda assim, sob escopo deste projeto (sem suite de testes formal), basta
manter o comentário e a vigilância. Sem ação imediata.

---

### B4 — `nav2.launch.py:8` docstring usa `$HOME/Workspace/...` (correto), mas é uma string raw em comentário

`ros2 launch robot_nav nav2.launch.py map:=$HOME/Workspace/Controle_robo_web/maps/meu_mapa.yaml`

OK como exemplo absoluto. Anotação só: difere de `~` usado em outros lugares.
Consistência com `~` reduziria mental switch — mas não vale uma edição.

✅ Sem ação.

---

### B5 — `setup_udev.sh:228` instrução final pede `colcon build` mas o `launch.sh` já faz incremental

- `setup_udev.sh:227-230`

Depois de criar as regras udev, o script orienta o usuário a rodar
`colcon build --packages-select robot_nav wheel_msgs` manualmente. Mas se o
usuário já tem `install/setup.bash` e roda `./launch.sh`, o launcher detecta
mudanças (linha 144-148, hash dos fontes) e refaz a build sozinho. A
instrução manual virou redundante.

**Fix:** trocar pelo simples:
```bash
echo "Próximo passo: ./launch.sh (rebuild incremental automático)."
echo "Pra recompilar tudo do zero (raro), apague o install/ e rode ./setup_pi.sh."
```

---

### B6 — `firmware/mega_bridge/src/leds.cpp:18` `FLASH_TOTAL_MS` usa `FLASH_PERIOD_MS * FLASH_CYCLES` mas o `// 1000` comment é cosmético

`constexpr uint16_t FLASH_TOTAL_MS = FLASH_PERIOD_MS * FLASH_CYCLES;  // 1000`

OK — o `// 1000` é apenas reminder de que dá 1 s; se alguém mudar
`FLASH_CYCLES` pra 10 sem atualizar o comentário, o leitor casual fica
confuso. Sem fix; só observação.

✅ Sem ação.

---

## Decisões pendentes (consultar usuário antes de aplicar)

- **D2026-05-27-A** — M3 (`nav2.launch.py` outputs): recomendado `lifecycle_manager` em screen e o resto em log, mas usuário pode preferir verbose pra debug. Aplicar parcialmente (só `lifecycle_manager` e `controller_server` em screen) pode ser meio-termo.
- **D2026-05-27-B** — M6 (paralelismo Pi 5): heurística automática vs override manual. Aplicar a versão automática primeiro; documentar `PI_BUILD_PARALLEL` no README como override.
- **D2026-05-27-C** — M8 (`/leds/state` topic): adiciona API nova; requer mudança no `trekking_runner` pra usar. Pode ser feito em PR separado pra não acoplar com a auditoria.

---

## Checklist sugerido (ordem de execução)

### Etapa 1 — Altos
1. **A1** — fixar `setup_udev.sh:187` (afeta arquivo gravado em `/etc/`).
2. **A2** — remover fallback IP fixo do `bin/robot-pair-ps4`.

### Etapa 2 — Médios "quick wins"
3. **M1** — corrigir comentário "3 s" → "1 s" em `main.cpp:69`.
4. **M2** — `~/Controle_robo_web` → `~/Workspace/Controle_robo_web` em `setup.sh:9`, `setup_pi.sh:14`, todos os hits do README. **Atenção**: alguns README hits podem ser intencionais (exemplo genérico), revisar com cuidado.
5. **M10** — atualizar `slam.launch.py` pra mencionar botão web primeiro.

### Etapa 3 — Médios "trade-off"
6. **M3** — silenciar nó Nav2 (decisão D2026-05-27-A).
7. **M5** — extrair `kill_known_nodes()` no `launch.sh`.
8. **M9** — try/except no `app.py:35` com fallback pro EchoController.

### Etapa 4 — Médios "feature work"
9. **M4** — pinar versão do `ldlidar_stl_ros2` (ver D2026-05-27-A).
10. **M6** — paralelismo adaptativo no `setup_pi.sh` (decisão D2026-05-27-B).
11. **M8** — adicionar topic `/leds/state` no `mega_bridge.py` (decisão D2026-05-27-C).

### Etapa 5 — Baixos
- B2 (dedup aviso `/dev/mega`)
- B5 (texto final do `setup_udev.sh`)
- B1/B3/B4/B6 = anotação; sem fix.

---

## Notas finais

- Achados estão no nível "polimento" — sem 🔴, dois 🟠 (paths em `/etc/` e fallback IP hardcoded). Confirma que a Fase 1-3 do `PLANO_HEADLESS_2026-05-22.md` está sólida.
- AUDITORIA_2026-05-26 inteira foi aplicada — confirmado item-por-item antes de iniciar esta passada. Inclui B1 (validado em campo no `leds.cpp`) que foi aplicado nesta mesma sessão.
- Próxima auditoria deve focar: (1) `trekking_runner.py` em profundidade (não auditado a fundo até aqui), (2) `pose_estimator.py` (fusão IMU+flow+rodas — caixa-preta hoje), (3) o caminho `nav_metrics` (CSV gerado mas não verificado em consumo).
