#!/bin/bash
# Uma volta do A/B: sobe gazebo+nav2 do PACOTE indicado, roda a rota, mata tudo.
# Uso: run_one.sh <pacote> <tag>
#
# 2026-08-28: world/mapa/rota/params/spawn eram FIXOS em sala_grande + o
# nav2_params_pi.yaml, entao este harness nao conseguia testar outro perfil nem
# outra arena. Agora tudo entra por env var, com os valores antigos de default
# (nenhuma chamada existente muda de comportamento):
#   AB_WORLD  AB_MAP  AB_ROTA  AB_PARAMS  AB_SX  AB_SY  AB_EXTRA_LAUNCH
# Exemplo (perfil da arena):
#   AB_PARAMS=nav2_params_arena.yaml \
#   AB_EXTRA_LAUNCH="follow_clear_full:=1.2 follow_clear_min:=0.35" \
#   ./run_one.sh robot_nav arena_v1
# Sai 0 = volta completa (result.json escrito). Sai 2 = falha de SETUP (não é
# resultado do robô — a volta deve ser refeita, não contabilizada).
set +u
PKG="$1"; TAG="$2"
REPO=/home/rbe-luis/Workspace/Controle_robo_web
SP=${SIM_AB_DIR:-/home/rbe-luis/Workspace/Controle_robo_web/log/sim_ab}
# 2026-08-28: os SCRIPTS agora saem do repo (TOOLS = a pasta deste arquivo), nao
# de $SIM_AB_DIR. Antes o run_n chamava "$SP/run_one.sh" e log/sim_ab/ guardava
# COPIAS do harness inteiro — que e' gitignore'd (.gitignore:20). Resultado: o
# codigo que rodava nao era o do git, correcao no repo nao chegava na execucao, e
# o `rm -rf "$OUT"` podia apagar as proprias ferramentas. $SP agora e' SO' saida.
TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ---- GUARDA ANTES DO rm -rf ------------------------------------------------
# 2026-08-28: com `set +u`, chamar sem <tag> fazia OUT virar o PROPRIO $SP e o
# `rm -rf` levava log/sim_ab/ inteiro — que guarda as voltas E as ferramentas
# (kill_all.sh, colisao.py, consolida.py), inclusive a que este script executa
# logo abaixo. Um argumento esquecido apagava o harness.
if [ $# -ne 2 ] || [ -z "$PKG" ] || [ -z "$TAG" ]; then
    echo "USO: run_one.sh <pacote> <tag>   (os DOIS obrigatorios e nao vazios)" >&2
    exit 2
fi
case "$TAG" in
    */*|.|..|-*) echo "SETUP: tag invalida: '$TAG' (sem '/', '.', '..' ou '-' no inicio)" >&2; exit 2 ;;
esac
OUT="$SP/$TAG"
# cinto e suspensorio: OUT tem que ser ESTRITAMENTE mais fundo que SP
case "$OUT" in
    "$SP"|"$SP"/) echo "SETUP: OUT colapsou em SP ($SP) — abortando antes do rm" >&2; exit 2 ;;
esac
rm -rf "$OUT"; mkdir -p "$OUT"
WORLD="${AB_WORLD:-$REPO/worlds/sala_grande.sdf}"
MAP="${AB_MAP:-$REPO/maps/sala_grande.yaml}"
ROTA="${AB_ROTA:-$REPO/maps/routes/rota1.json}"
PARAMS_NAME="${AB_PARAMS:-nav2_params_pi.yaml}"
SX=${AB_SX:-2.0}; SY=${AB_SY:-0.0}
for _f in "$WORLD" "$MAP" "$ROTA"; do
    [ -f "$_f" ] || { echo "SETUP: nao existe: $_f" >&2; exit 2; }
done

cd "$REPO" || exit 2          # cwd fixo: os CSVs dos nós vão pro controle_web/logs certo
source /opt/ros/jazzy/setup.bash
source "$REPO/install/setup.bash"

cleanup() { bash "$TOOLS/kill_all.sh" >> "$OUT/cleanup.log" 2>&1; }
trap cleanup EXIT INT TERM

# --- SETUP com retry: boot do nav2 falha de vez em quando (bond/lifecycle) e
#     isso NÃO é resultado do robô. Tenta 3x antes de desistir.
for BOOT in 1 2 3; do
    echo "=== [$TAG] boot $BOOT/3 (pacote=$PKG) ==="
    bash "$TOOLS/kill_all.sh" || { echo "[$TAG] ambiente sujo, abortando"; exit 2; }
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

    PARAMS="$(ros2 pkg prefix $PKG)/share/$PKG/config/$PARAMS_NAME"
    # FALHA FECHADA, mesma razao do ./launch.sh --arena: o params_file carrega a
    # geometria de seguranca. Rodar um A/B com outro footprint do que foi pedido
    # produz numero que parece valido e nao e'.
    [ -f "$PARAMS" ] || { echo "SETUP: params nao encontrado: $PARAMS" >&2; exit 2; }
    setsid ros2 launch "$PKG" nav2.launch.py map:="$MAP" use_sim_time:=true \
        params_file:="$PARAMS" set_initial_pose:=true init_x:=$SX init_y:=$SY init_yaw:=0.0 \
        $AB_EXTRA_LAUNCH \
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
setsid python3 "$TOOLS/colisao.py" "$WORLD" "$OUT/colisao.csv" > "$OUT/colisao.log" 2>&1 &
sleep 2

echo "[$TAG] rodando a rota..."
timeout 6000 python3 "$TOOLS/probe.py" "$ROTA" "$OUT/result.json" 600 2>&1 | tee "$OUT/probe.log"
# os nós gravam em controle_web/logs e SOBRESCREVEM a cada launch: arquiva já.
for c in follow_debug freeze_capture unstuck follow_plan_last; do
    [ -f "$REPO/controle_web/logs/$c.csv" ] && cp "$REPO/controle_web/logs/$c.csv" "$OUT/$c.csv"
done
[ -f "$OUT/result.json" ] || { echo "[$TAG] probe não produziu resultado"; exit 2; }
echo "[$TAG] fim."
