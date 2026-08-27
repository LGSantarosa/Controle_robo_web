#!/bin/bash
# Uma volta do A/B: sobe gazebo+nav2 do PACOTE indicado, roda a rota, mata tudo.
# Uso: run_one.sh <pacote> <tag>
# Sai 0 = volta completa (result.json escrito). Sai 2 = falha de SETUP (não é
# resultado do robô — a volta deve ser refeita, não contabilizada).
set +u
PKG="$1"; TAG="$2"
REPO=/home/rbe-luis/Workspace/Controle_robo_web
SP=${SIM_AB_DIR:-/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab}
OUT="$SP/$TAG"; rm -rf "$OUT"; mkdir -p "$OUT"
WORLD="$REPO/worlds/sala_grande.sdf"
MAP="$REPO/maps/sala_grande.yaml"
ROTA="$REPO/maps/routes/rota1.json"
SX=2.0; SY=0.0

cd "$REPO" || exit 2          # cwd fixo: os CSVs dos nós vão pro controle_web/logs certo
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"

cleanup() { bash "$SP/kill_all.sh" >> "$OUT/cleanup.log" 2>&1; }
trap cleanup EXIT INT TERM

# --- SETUP com retry: boot do nav2 falha de vez em quando (bond/lifecycle) e
#     isso NÃO é resultado do robô. Tenta 3x antes de desistir.
for BOOT in 1 2 3; do
    echo "=== [$TAG] boot $BOOT/3 (pacote=$PKG) ==="
    bash "$SP/kill_all.sh" || { echo "[$TAG] ambiente sujo, abortando"; exit 2; }
    sleep 2

    setsid ros2 launch "$PKG" sim.launch.py world:="$WORLD" \
        spawn_x:=$SX spawn_y:=$SY > "$OUT/sim.log" 2>&1 &
    if ! timeout 90 bash -c 'until ros2 topic list 2>/dev/null | grep -q "^/clock$"; do sleep 2; done'; then
        echo "[$TAG] sem /clock — retry"; continue
    fi
    if ! timeout 60 bash -c 'until ros2 topic list 2>/dev/null | grep -q "^/scan$"; do sleep 2; done'; then
        echo "[$TAG] sem /scan — retry"; continue
    fi
    sleep 15   # gazebo assentando: subir o nav2 em cima do pico de CPU do gz
               # trava o bringup no map_server (lifecycle desiste de esperar)

    PARAMS="$(ros2 pkg prefix $PKG)/share/$PKG/config/nav2_params_pi.yaml"
    setsid ros2 launch "$PKG" nav2.launch.py map:="$MAP" use_sim_time:=true \
        params_file:="$PARAMS" set_initial_pose:=true init_x:=$SX init_y:=$SY init_yaw:=0.0 \
        > "$OUT/nav2.log" 2>&1 &

    # Prontidão REAL: não basta o tópico do costmap existir — o que o probe usa
    # é o action server. Espera os dois (180 s: a 1ª subida é a mais lenta).
    ok=1
    # bt_navigator ATIVO é o sinal de que o lifecycle percorreu a fila inteira.
    timeout 90 bash -c 'until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; do sleep 3; done' || ok=0
    [ "$ok" = "1" ] && { timeout 60 bash -c 'until ros2 topic list 2>/dev/null | grep -q "global_costmap/costmap$"; do sleep 2; done' || ok=0; }
    [ "$ok" = "1" ] && { timeout 60 bash -c 'until ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; do sleep 2; done' || ok=0; }
    if [ "$ok" = "1" ]; then
        echo "[$TAG] nav2 pronto (costmap + action server)"
        sleep 8
        break
    fi
    echo "[$TAG] nav2 não ativou — retry"
    [ "$BOOT" = "3" ] && { echo "[$TAG] FALHA DE SETUP após 3 tentativas"; exit 2; }
done

# DETECTOR DE COLISÃO (ground truth do Gazebo). Antes eu só tinha min_scan, que
# é proxy: o laser fica no centro do robô, então 0.25 m tanto pode ser "passei
# raspando" quanto "já encostei". Aqui a conta é geométrica (OBB do robô x AABB
# das paredes) e diz em metros se ENCOSTOU.
setsid python3 "$SP/colisao.py" "$WORLD" "$OUT/colisao.csv" > "$OUT/colisao.log" 2>&1 &
sleep 2

echo "[$TAG] rodando a rota..."
timeout 6000 python3 "$SP/probe.py" "$ROTA" "$OUT/result.json" 600 2>&1 | tee "$OUT/probe.log"
# os nós gravam em controle_web/logs e SOBRESCREVEM a cada launch: arquiva já.
for c in follow_debug freeze_capture unstuck follow_plan_last; do
    [ -f "$REPO/controle_web/logs/$c.csv" ] && cp "$REPO/controle_web/logs/$c.csv" "$OUT/$c.csv"
done
[ -f "$OUT/result.json" ] || { echo "[$TAG] probe não produziu resultado"; exit 2; }
echo "[$TAG] fim."
