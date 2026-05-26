# Auditoria do projeto Controle_robo_web — 2026-05-26

Terceira passada depois de:
- [AUDITORIA_2026-05-14.md](./AUDITORIA_2026-05-14.md) — primeira limpeza (D1 collision_monitor removido)
- [AUDITORIA_2026-05-18.md](./AUDITORIA_2026-05-18.md) — segunda passada (~95% aplicada via commits `4f227f4`, `26476e4`, `5f7fd9c`)

Cobre o que foi adicionado desde `5f7fd9c`: infraestrutura headless (Fase 1/2/3 do `PLANO_HEADLESS_2026-05-22.md`), `twist_mux` arbitrando `/cmd_vel`, web em "modo monitor", `robot-up/robot-key/robot-connect`, `pair-ps4.sh`, novo padrão de LED com base branca + gating do PMW3901, mudança de default de modo para TELEOP.

> Severidades: 🔴 crítica (bloqueia uso real / quebra dados / segurança) — 🟠 alta (bug funcional, comportamento incorreto) — 🟡 média (qualidade/robustez/UX) — 🟢 baixa (cosmético / refator).
>
> **Estado git:** branch `main`, último commit `c84d804` (LEDs: corrige comentário do step-down). **Arquivos modificados não commitados** mudam `robot.local` → `robo-desktop.local` em scripts/README e adicionam fixes BlueZ ao `setup_headless.sh`. Esses changes vão entrar no próximo commit — considerá-los aplicados ao revisar este doc.
>
> **Memórias relevantes pra próxima sessão:**
> - Sem `Co-Authored-By Claude` em commits deste repo.
> - PS4 sempre pareia em pair mode pelo PC dev via `./bin/robot-pair-ps4` (não tenta no PC dev — bug de HW).
> - Preferir ajustar scripts a pedir o usuário mudar hostname/SO.
> - `robot.local` → `robo-desktop.local` é decisão em andamento — pode rebatizar de novo.

---

## 🔴 CRÍTICOS

### C1 — `sim.launch.py` não sobe `twist_mux`: modo SIM perde toda arbitragem de `/cmd_vel`

- `ros2_packages/robot_nav/launch/sim.launch.py:83-95` (bridge publica `/cmd_vel` direto)
- `ros2_packages/robot_nav/launch/nav2.launch.py:96-98` (comentário **já reconhece** o problema)

O `nav2.launch.py:98` remapeia a saída do `velocity_smoother` para `nav_vel` esperando que o `twist_mux` arbitre. Mas o `sim.launch.py` não inclui `joy_node`, `teleop_twist_joy` nem `twist_mux`. Resultado em SIM + Nav2:

- `velocity_smoother` publica em `nav_vel` ✅
- **ninguém** publica em `/cmd_vel` (não há mux)
- bridge GZ consome `/cmd_vel` → robô não anda em simulação com Nav2

Hoje funciona "por sorte" porque ninguém testa `--sim --nav2` ativamente. Mesmo com `WEB_TELEOP=on` no sim, há um segundo problema: web publica em `/cmd_vel` direto enquanto Nav2 publica em `nav_vel` que ninguém lê — controle web vence o vácuo.

O comentário em `nav2.launch.py:96-97` reconhece literalmente: "o sim.launch.py não sobe o twist_mux, então no sim ninguém arbitra nav_vel→cmd_vel — ver PLANO_HEADLESS_2026-05-22 §2.4."

**Fix (escolher uma):**
- **(a) Recomendado:** subir o `twist_mux` no `sim.launch.py` também (sem joy/teleop_twist_joy, só o mux). Cliente web pode então publicar em `web_vel` (precisa adicionar essa entrada no `twist_mux.yaml`) ou continuar em `/cmd_vel` se aceitarmos que web bypassa o mux no SIM.
- **(b)** No SIM, fazer o `velocity_smoother` publicar em `/cmd_vel` direto (segundo remap dedicado pro sim) — mais simples, mas inconsistente com o real.
- **(c)** Bloquear `--sim --nav2` no `launch.sh` até resolver — preserva a invariante mas reduz funcionalidade.

---

### C2 — `setup_headless.sh` não cobre fix #4 do BlueZ (`ClassicBondedOnly=false`); só o `pair-ps4.sh` faz

- `scripts/setup_headless.sh:35-89` (cobre ERTM + ControllerMode + AutoEnable + rfkill)
- `pair-ps4.sh:128-144` (único lugar que mexe em `/etc/bluetooth/input.conf`)

O `setup_headless.sh` aplica 3 dos 4 fixes "persistentes" do BlueZ (`ERTM`, `ControllerMode=bredr`, `AutoEnable=true`) mas **não** `ClassicBondedOnly=false` em `/etc/bluetooth/input.conf`. Sem ele, `bluetoothd` rejeita HID de devices sem bonding completo (sintoma: PS4 fica `Connected: yes` mas `/dev/input/js0` nunca materializa).

Hoje o `pair-ps4.sh` salva a situação porque aplica o fix antes do pair. Mas:
1. Se o usuário rodar só `setup_headless.sh` (sem pair) e quiser usar um PS4 já pareado (caso de imagem clonada), o controle conecta mas não vira `js0`.
2. A *promessa* do `setup_headless.sh` (comentário linha 6-7: "deixa a máquina pronta pra ser o robô") é incompleta.
3. Risco real: o usuário re-paira o controle só com `bluetoothctl pair` (sem o `pair-ps4.sh`), confiando que a máquina foi "preparada" pelo `setup_headless.sh` — quebra.

**Fix:** mover o bloco `ClassicBondedOnly` do `pair-ps4.sh:128-144` pro `setup_headless.sh` (após o bloco do `MAINCONF` em `:75-78`). Manter o bloco no `pair-ps4.sh` também (idempotente; defesa em profundidade).

---

## 🟠 ALTOS — bugs funcionais

### A1 — `bin/robot-up` exec do `launch.sh` sem checar existência

- `bin/robot-up:28` — `exec tmux new -s "$SESSION" "./launch.sh --$MODE $*; bash"`

Se o repo for movido após o symlink ser criado (`setup_headless.sh:26-33`), o `cd "$REPO_DIR"` pode falhar silenciosamente (`set -euo pipefail` pega o cd mas não o `./launch.sh` inexistente dentro do tmux). Pior: o tmux fica aberto com `bash` (linha 28: `"; bash"`), então o usuário vê uma sessão "viva" — vai pensar que está rodando.

**Fix em `bin/robot-up:21`** (após o `cd`):
```bash
[ -f "$REPO_DIR/launch.sh" ] || { echo "ERRO: launch.sh não existe em $REPO_DIR" >&2; exit 1; }
```

---

### A2 — `bin/robot-up` reanexa sessão mesmo se `launch.sh` morreu — usuário vê tmux vivo achando que robô está rodando

- `bin/robot-up:22-24` — `tmux has-session && exec tmux attach`

A linha 28 já termina o comando do pane com `; bash` (intencional: deixa o pane aberto pra ver erro do launch.sh). Mas no `attach` (linha 24) o usuário entra num painel onde o `launch.sh` saiu e o bash herdou — fica vivo silenciosamente, sem indicar que a stack não está rodando.

**Fix em `bin/robot-up`:** ao reanexar, checar se o `launch.sh` ainda está ativo no host:
```bash
if tmux has-session -t "$SESSION" 2>/dev/null; then
    if ! pgrep -af "[l]aunch\.sh" >/dev/null; then
        echo "AVISO: sessão '$SESSION' existe mas launch.sh não está rodando."
        echo "       Saia (Ctrl+D) e rode 'tmux kill-session -t $SESSION && robot-up' pra recomeçar."
    fi
    exec tmux attach -t "$SESSION"
fi
```

---

### A3 — `bin/robot-key` falha em ambiente sem ROS sourceado se `install/setup.bash` não existir

- `bin/robot-key:15-18` — só sourceia se `install/setup.bash` existe

O `robot-key` é executado dentro de uma sessão SSH iniciada pelo `robot-connect` — esse shell SSH não roda `~/.bashrc` por default em modo não-interativo. Se o repo nunca foi `colcon build`, `install/setup.bash` não existe, e `ros2 run teleop_twist_keyboard` falha com "command not found" sem mensagem clara.

**Fix em `bin/robot-key:15`:**
```bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    # Fallback: sourceia ROS base se workspace ainda não foi compilado.
    for d in /opt/ros/*/setup.bash; do
        [ -f "$d" ] && source "$d" && break
    done
fi
command -v ros2 >/dev/null || {
    echo "ERRO: ros2 não encontrado. Compile o workspace primeiro (./start.sh) ou source /opt/ros/<distro>/setup.bash." >&2
    exit 1
}
```

---

### A4 — `setup_headless.sh` aplica rfkill em runtime mas não persiste — Pi reinicia, BT volta a "Soft blocked: yes"

- `scripts/setup_headless.sh:85-88` (uncommitted diff) — `rfkill unblock bluetooth` só em runtime

Em alguns combos de Raspberry Pi + drivers, o adaptador BT acorda com `rfkill soft-block: yes` em todo boot. O `setup_headless.sh` chama `rfkill unblock` uma vez, mas isso não persiste — após reboot, o BT volta a bloqueado e o `pair-ps4.sh` falha em "Powered: no".

O `pair-ps4.sh:153-157` também só destrava em runtime — defesa em profundidade.

**Fix:** salvar a config de rfkill após o unblock:
```bash
if rfkill list bluetooth 2>/dev/null | grep -q "Soft blocked: yes"; then
    echo "  → Destravando bluetooth no rfkill (e salvando)"
    sudo rfkill unblock bluetooth
    # Persistência: depende da distro. systemd-rfkill salva automático em /var/lib/systemd/rfkill/.
    # Garante que o serviço esteja habilitado pra restaurar no boot.
    sudo systemctl enable systemd-rfkill.service 2>/dev/null || true
fi
```

> Nota: validar em campo. Se `systemd-rfkill` não estiver no Pi, alternativa é dropear em `/etc/systemd/system/bluetooth.service.d/override.conf` com `ExecStartPre=/usr/sbin/rfkill unblock bluetooth`.

---

### A5 — `bin/robot-connect` SSH sem health check; falha de rede leva a "trava com Ctrl+C inútil"

- `bin/robot-connect:26-28` — `exec ssh -t ... "robot-up ..."`

Se o `robo-desktop.local` não resolver (mDNS quebrado, WiFi diferente), o ssh entra em `Could not resolve hostname` e o `exec` substitui o shell — saída é uma mensagem de erro genérica do SSH, sem dicas. Já temos `ServerAliveInterval=15 + ServerAliveCountMax=3` (45 s de hard timeout depois de conectar), mas falta o pré-check de conectividade.

**Fix em `bin/robot-connect:23`** (antes do exec):
```bash
if ! ssh -o BatchMode=yes -o ConnectTimeout=3 "${REMOTE_USER}@${HOST}" true 2>/dev/null; then
    cat >&2 <<EOF
ERRO: não consegui conectar em ${REMOTE_USER}@${HOST}.
  - mDNS quebrado? Tente: ROBOT_HOST=<ip-do-robô> robot-connect ${MODE}
  - Chave SSH não copiada? Rode: ssh-copy-id ${REMOTE_USER}@${HOST}
  - Usuário diferente? ROBOT_USER=<seu-user> robot-connect ${MODE}
EOF
    exit 1
fi
```

---

### A6 — `leds.cpp:35-45` `transition_()` reseta `state_start_` mesmo em transição idempotente (WAYPOINT→WAYPOINT)

- `firmware/mega_bridge/src/leds.cpp:35-45`
- `firmware/mega_bridge/src/main.cpp:73-77` — `FT_LEDS` com id `WAYPOINT` chama `triggerWaypoint()` direto

Se o PC mandar `FT_LEDS` com `id=WAYPOINT` enquanto o anel já está em `WAYPOINT`, o `transition_()` reseta `state_start_` (linha 37) e `gated_until_` (linha 41) → o flash recomeça do zero e o PMW3901 fica gateado por mais 1150 ms. Em modo trekking, isso pode ser disparado várias vezes em sequência (ex: ressending de comando após restart de nó ROS).

Impacto: dead-reckoning extra ~57 cm a 0,5 m/s por reset espúrio — somando 2-3 resets, perda de localização perceptível.

**Fix em `firmware/mega_bridge/src/leds.cpp:35`:**
```cpp
void Ring::transition_(State s) {
    if (s == state_ &&
        (s == State::WAYPOINT || s == State::STARTING)) {
        // Já estamos no estado animado — não reiniciar o flash nem o gating.
        return;
    }
    state_       = s;
    state_start_ = millis();
    // ... resto
}
```

---

### A7 — `pair-ps4.sh:206-209` usa `awk '/Wireless Controller/'` que pode pegar device errado

- `pair-ps4.sh:206-209` (remoção do "OLD_MAC")
- `pair-ps4.sh:256-263` (scan novo)

`bluetoothctl devices` retorna **todos** os devices conhecidos, e `awk '/Wireless Controller/{print $2; exit}'` pega o **primeiro**. Se o usuário tem mais de um controle pareado (PS4 + outro de outro robô na bancada, controle PS5 com o mesmo nome genérico, ou um Xbox Wireless Controller), o `bluetoothctl remove "$OLD_MAC"` pode apagar o controle do colega.

**Fix:** restringir por classe de dispositivo (`Class: 0x002508` para gamepad HID PS4) ou por nome exato:
```bash
OLD_MAC=$(bluetoothctl devices 2>/dev/null | awk '/Wireless Controller$/{print $2}' | while read mac; do
    if bluetoothctl info "$mac" 2>/dev/null | grep -qE "(0x002508|Sony Computer Entertainment)"; then
        echo "$mac"
        break
    fi
done)
```

Ou: pedir confirmação interativa quando há mais de um match.

---

## 🟡 MÉDIOS — qualidade, robustez, UX

### M1 — `gamepad.js:107` emite `gamepad_event` no disconnect mesmo com `WEB_TELEOP=off`

- `controle_web/static/js/gamepad.js:100-109`

A linha 107 (`socket.emit('gamepad_event', { type: 'axis', linear: 0, angular: 0 })`) é executada no handler de `gamepaddisconnected` **sem checar** `window._robotIsTeleopEnabled()`. O `pollLoop` (linha 128) **sim** checa — inconsistente.

Hoje é inócuo: o handler Flask em `app.py:448` retorna early com `WEB_TELEOP=off`. Mas viola a semântica "em modo monitor a web não emite movimento nunca" e gera ruído no log do servidor.

**Fix em `gamepad.js:106`:**
```javascript
if (activeGamepadIndex === e.gamepad.index) {
    activeGamepadIndex = null;
    stopPolling();
    if (!window._robotIsTeleopEnabled || window._robotIsTeleopEnabled()) {
        socket.emit('gamepad_event', { type: 'axis', linear: 0, angular: 0 });
    }
}
```

---

### M2 — `leds.h:15` comentário diz "3 piscadas verdes" mas implementação usa 5 ciclos

- `firmware/mega_bridge/include/leds.h:15` — `STARTING = 3, // 3 piscadas verdes ao sair de IDLE pro RUN`
- `firmware/mega_bridge/src/leds.cpp:17` — `constexpr uint8_t FLASH_CYCLES = 5;`

Commit `8c4db2e` mudou de 3 piscadas pra 5 ciclos com base branca alternada, mas o comentário no header não foi atualizado. O `WAYPOINT` na linha 17 do mesmo header diz "pisca laranja ~3 s" — também desatualizado (1 s = `FLASH_TOTAL_MS`).

**Fix:** atualizar comentários:
```cpp
STARTING = 3,  // 5 ciclos verde/branco (1 s) ao sair de IDLE pro RUN
...
WAYPOINT = 5,  // 5 ciclos amarelo/branco (1 s) ao chegar num ponto + 150 ms recovery do PMW
```

---

### M3 — `twist_mux.yaml:23` `key_vel` não tem publisher dentro do `robot.launch.py`

- `ros2_packages/robot_nav/config/twist_mux.yaml:22-25`
- `ros2_packages/robot_nav/launch/robot.launch.py:159-174` — só sobe joy + teleop_twist_joy, não keyboard

`key_vel` vem de `bin/robot-key` (terminal SSH separado) — não é bug, é design. Mas a comunicação no README/comentários pode confundir: "twist_mux espera key_vel" sem dizer onde está o publisher. Se o usuário rodar `./launch.sh` e tentar usar WASD, vai pensar que está quebrado.

**Fix:** adicionar comentário em `twist_mux.yaml:22`:
```yaml
keyboard:
  topic: key_vel       # publicado por bin/robot-key (executado em terminal SSH separado)
  timeout: 0.5
  priority: 90
```

E adicionar uma linha em `robot.launch.py:11` da docstring:
```
  6.5 (manual, fora desta launch): bin/robot-key → key_vel
```

---

### M4 — `pair-ps4.sh:6-22` docstring diz "5 pegadinhas" mas o script aplica 6 fixes (HID drivers não está na lista)

- `pair-ps4.sh:6-22` (lista 5 problemas numerados)
- `pair-ps4.sh:172-182` (Fix 4: HID drivers — não está numerado na docstring)

Não é bug funcional — o `modprobe hid_playstation; modprobe joydev` está implementado e funciona. Mas a docstring lista exatamente 5 problemas e o código tem 6 seções, confundindo quem for editar.

**Fix:** adicionar ao docstring após o item 5:
```
#   6) HID drivers (hid_playstation/hid_sony + joydev) podem não estar carregados
#                     em builds custom de kernel — `modprobe` antes do pair garante
#                     que /dev/input/jsN materialize quando HID profile subir.
```

---

### M5 — `setup_headless.sh` e `pair-ps4.sh` editam `/etc/bluetooth/main.conf` com regex idênticas — risco de divergir

- `scripts/setup_headless.sh:46-78` (uncommitted)
- `pair-ps4.sh:93-126`

Dois lugares fazem exatamente a mesma edição (`ControllerMode=bredr` + `AutoEnable=true`) com a mesma regex sed. Idempotente, mas se um dia alguém ajustar a regex num lugar e esquecer no outro, o resultado é diferente em cada caminho.

**Fix:** mover a lógica para um helper compartilhado em `scripts/_bluez_fixes.sh` que ambos sourceiam. Ou, mais barato: comentar no topo de cada bloco "manter sincronizado com `<outro arquivo>:<linha>`".

---

### M6 — `robot-connect:27` `ServerAliveCountMax=3` pode ser curto em WiFi de roteador doméstico

- `bin/robot-connect:26-27`

`ServerAliveInterval=15 × CountMax=3 = 45 s` de tolerância antes de derrubar SSH. Em handoffs de AP ou interferência, perde-se a sessão tmux remota (precisa rodar `robot-connect` de novo pra reanexar). Não é bug — o tmux preserva o estado da stack — mas a UX pode ser ruim ("ué, caiu de novo").

**Fix:** subir pra `ServerAliveCountMax=6` (90 s) ou adicionar `TCPKeepAlive=yes`. Documentar a escolha.

---

### M7 — `robot-connect:28` passa `$*` (modo + extras) sem escaping pro ssh

- `bin/robot-connect:28` — `exec ssh -t ... "robot-up ${MODE} $*"`

Se o usuário rodar `robot-connect nav2 --map="meu mapa.yaml"`, o `$*` quebra o quoting e o SSH recebe `robot-up nav2 --map=meu mapa.yaml` (sem aspas). Funciona pra args simples, falha em paths com espaço.

**Fix em `bin/robot-connect:28`:**
```bash
# Constrói comando preservando o quoting
remote_cmd="robot-up $(printf '%q ' "$MODE" "$@")"
exec ssh -t \
    -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
    "${REMOTE_USER}@${HOST}" "$remote_cmd"
```

---

### M8 — `launch.sh:5-13` cabeçalho não menciona `--web-teleop` na lista de modos

- `launch.sh:1-20`

A nova flag `--web-teleop` é o caminho pra reativar o controle web (já que o default agora é "monitor"). Está em `--help` (linha 63) mas não no cabeçalho documentado. Quem ler o topo do arquivo pra entender o fluxo vai pensar que web nunca dirige.

**Fix:** adicionar entre linhas 13 e 15:
```bash
# Variantes do modo TELEOP:
#   ./launch.sh --web-teleop          # reativa o controle web (default: web é só visualização)
```

---

### M9 — `robot.launch.py:131` `joy_node` com `device_id: 0` hardcoded; sem `output: own_log` polui terminal

- `ros2_packages/robot_nav/launch/robot.launch.py:122-135`

`device_id: 0` assume `/dev/input/js0`. Se o PS4 não estiver conectado, o `joy_node` loga warning a cada ~1 s ("Couldn't open joystick force feedback"). Com `output='screen'` (linha 129), polui o terminal misturando com logs reais.

**Fix:**
- (a) Trocar `output='screen'` → `output={'stderr': 'log'}` pro joy só.
- (b) Adicionar parâmetro `coalesce_interval_ms: 50` ou similar pra reduzir reconnect attempts.

---

### M10 — `launch.sh` aceita `--sim --teleop` mas SIM não tem `joy_node`/`twist_mux` (modo silenciosamente vazio)

- `launch.sh:42-48`
- `sim.launch.py` (não inclui PS4/teleop)

Se o usuário rodar `./launch.sh --sim` (modo default TELEOP) ou `./launch.sh --sim --teleop`, vai sair o Gazebo mas sem nenhuma forma de mover o robô (a menos que ative `--web-teleop`). Não há erro, só "robô parado". Relacionado com C1.

**Fix:** quando `SIM=true` e o modo é TELEOP, exigir `--web-teleop` explícito ou imprimir aviso:
```bash
if [ "$SIM" = true ] && [ "$MODE" = "teleop" ] && [ "$WEB_TELEOP" = "off" ]; then
    echo "[AVISO] --sim --teleop sem --web-teleop: nenhum publisher de /cmd_vel será iniciado."
    echo "        Para dirigir no sim, use --web-teleop ou rode um ros2 run teleop_* à parte."
fi
```

---

### M11 — `bin/robot-pair-ps4` e `pair-ps4.sh` não compartilham código; risco de divergir

- `bin/robot-pair-ps4` (29 linhas — wrapper SSH simples)
- `pair-ps4.sh` (436 linhas — toda a lógica)

`bin/robot-pair-ps4` apenas roda `ssh ... "cd repo && ./pair-ps4.sh --no-prompt"`. Não há código duplicado, mas implícito: `pair-ps4.sh` precisa existir no robô. Se o usuário só clonou o repo no PC dev, o `robot-pair-ps4` falha com erro críptico ("bash: ./pair-ps4.sh: No such file").

**Fix:** adicionar mensagem clara em `bin/robot-pair-ps4` se o arquivo remoto não existir:
```bash
if ! ssh "${REMOTE_USER}@${HOST}" "test -f ~/Workspace/Controle_robo_web/pair-ps4.sh" 2>/dev/null; then
    echo "ERRO: pair-ps4.sh não encontrado em ${REMOTE_USER}@${HOST}:~/Workspace/Controle_robo_web/"
    echo "      Clone/atualize o repo no robô primeiro (git pull)."
    exit 1
fi
```

---

### M12 — `pair-ps4.sh:436` última linha menciona `~/workspace/...` (minúsculo) mas o repo é `~/Workspace/...`

- `pair-ps4.sh:435` — `cd ~/workspace/Controle_robo_web && ./launch.sh`

Tipo cosmético, mas confunde no copy-paste. Linux é case-sensitive — `~/workspace/...` não existe na maioria dos setups (memória do projeto: repo está em `~/Workspace/Controle_robo_web`).

**Fix:** trocar para `~/Workspace/Controle_robo_web`.

---

## 🟢 BAIXOS — cosméticos / refator

### B1 — `firmware/mega_bridge/src/leds.cpp:30` `setBrightness(100)` sem comment-anchor pra "brightness máximo testado"

`leds.cpp:25-29` documenta os trade-offs (1,2 A Pi + 0,58 A LEDs + step-down 3 A) mas não diz "valor empírico testado em casa em piso X". Pra próxima sessão alguém pode mexer sem repetir o teste.

**Fix:** adicionar uma linha "Validado em campo: ROS2 + Pi + WiFi, piso branco, 5 ciclos sem brownout em 30 min."

---

### B2 — `teleop_ps4.yaml:18-21` comentário sobre angular=6.0 menciona `robot_controller.py:128-135` mas a constante mudou de lugar

- `ros2_packages/robot_nav/config/teleop_ps4.yaml:19-21`
- `controle_web/controllers/robot_controller.py` — verificar linha real de `BASE_ANGULAR_SPEED`

A linha 128-135 do robot_controller.py é referência fixa em comentário. Se alguém refatorar o controller (e mudanças recentes já mexeram nele — `+31 linhas`), o ponteiro fica desatualizado.

**Fix:** trocar para "ver `BASE_ANGULAR_SPEED` em `controle_web/controllers/robot_controller.py`" (sem linha).

---

### B3 — `bin/robot-up:18` `BASH_SOURCE[0]` mas script é `#!/usr/bin/env bash` — OK; `bin/robot-key` idem; `bin/robot-pair-ps4` também — consistente

Sem achado, anotando que os 3 scripts seguem o mesmo padrão. ✅

---

### B4 — `nav2.launch.py:8` docstring exemplo usa `$HOME/Workspace/...` mas é um exemplo, não um comando real

- `ros2_packages/robot_nav/launch/nav2.launch.py:8`

Exemplo de uso usa path absoluto. OK como exemplo, mas pode confundir leitor casual.

---

### B5 — README ainda lista `robot.local` em algum lugar? — confirmar após o próximo commit aplicar o rename

Modificações não-commitadas mudam `robot.local` → `robo-desktop.local` no README, `setup.sh`, `setup_pi.sh`, `bin/robot-connect`, `bin/robot-pair-ps4`. Faltam:

```
$ grep -rn "robot.local" --include="*.md" --include="*.sh" --include="*.py" --include="robot-*" .
```

Rodar antes de commitar pra confirmar zero hits. **(Validar em sessão de execução.)**

---

### B6 — `setup_headless.sh:24-32` symlinks em `/usr/local/bin/` com `ln -sf` sobrescreve sem aviso

`ln -sf` sobrescreve qualquer arquivo existente. Se o usuário tiver um script `robot-up` próprio em `/usr/local/bin/`, será silenciosamente substituído. Cenário improvável, mas vale um echo:

```bash
if [ -e "/usr/local/bin/$tool" ] && [ ! -L "/usr/local/bin/$tool" ]; then
    echo "  AVISO: /usr/local/bin/$tool é um arquivo real (não symlink), pulando."
    continue
fi
```

---

### B7 — `pair-ps4.sh:43` `sed -n '2,25p' "$0" | sed 's/^# \?//'` para `--help` perde formatação numerada

Cosmético. O help imprime do comentário inicial, OK.

---

## Decisões pendentes (consultar usuário antes de aplicar)

- **D2026-05-26-A** — C1 do `sim.launch.py` precisa direção. Opções a/b/c acima. Recomendo (a) por consistência com hardware real, mas (b) é mais barato se o sim é só pra desenvolvimento de algoritmos.
- **D2026-05-26-B** — Hostname `robo-desktop.local` é definitivo? Várias edits parciais no diff atual ainda. Confirmar antes de aplicar B5.
- **D2026-05-26-C** — `setup_headless.sh` deve absorver TODOS os fixes do BlueZ (C2, A4, M5) e o `pair-ps4.sh` virar só "pareia + valida"? Reduz divergência mas centraliza um arquivo pesado.

---

## Checklist sugerido (ordem de execução)

### Etapa 1 — Críticos
1. **C1** — Decidir SIM + Nav2 (a/b/c). Aplicar.
2. **C2** — Mover bloco `ClassicBondedOnly` para `setup_headless.sh`.

### Etapa 2 — Altos
3. **A1, A2** — Checks de existência e estado em `bin/robot-up`.
4. **A3** — Fallback de ROS source em `bin/robot-key`.
5. **A4** — Persistência de `rfkill unblock`.
6. **A5** — Pré-check de SSH em `bin/robot-connect`.
7. **A6** — Guard em `Ring::transition_()` pra estados idempotentes.
8. **A7** — Filtro por classe/info no `pair-ps4.sh` `OLD_MAC`.

### Etapa 3 — Médios
- M1 (gamepad guard) → quick win
- M2 (comentários LED) → quick win
- M3 (docs key_vel)
- M4 (docstring pair-ps4)
- M5 (DRY bluez fixes) — depois de C2 e A4
- M6 (SSH timeout)
- M7 (quoting do robot-connect)
- M8 (header launch.sh)
- M9 (joy_node noise)
- M10 (aviso --sim --teleop)
- M11 (check remote pair-ps4.sh)
- M12 (path lowercase fix)

### Etapa 4 — Baixos
B1–B7: aplicar conforme quiser polir. B5 obrigatório antes do próximo commit.

---

## Notas finais

- O agente ROS2 reportou um falso positivo sobre `scale_angular_turbo: 6.0` "ser igual ao normal" — o YAML em `teleop_ps4.yaml:18-25` documenta explicitamente que é intencional (sem turbo, o `teleop_twist_joy` cai pro default `1.0` rad/s, o que efetivamente *desabilita* o giro no turbo). **Não é bug.**
- O `sensor_flags` (byte 15 do STATE) que o firmware agora publica **já é lido** pelo `mega_bridge.py:345` e publicado em `/system/health` como JSON com `imu_ok`/`flow_ok`. Resolve completamente o A2/M4 da AUDITORIA_2026-05-18. ✅
- O `twist_mux` resolve o B20 da AUDITORIA_2026-05-18 (múltiplos publishers em `/cmd_vel`) **no hardware real**. No SIM o problema persiste — ver C1.
- Modo "web monitor" (Fase 2) está consistente: `WEB_TELEOP` é gateado em `app.py:373,448,524` e o controller só publica se `enable_publish=WEB_TELEOP`. O único ruído residual é o M1 (emit no disconnect).
- A infra headless está robusta no caminho feliz; os achados acima são edge cases (perda de SSH, repo movido, multiusuário com vários controles na bancada).
