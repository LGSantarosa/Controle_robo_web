#!/bin/bash
# Launcher completo: hoverboard driver + LiDAR + servidor web.
#
# Modos:
#   ./launch.sh                                   # TELEOP (padrão) — sem autônomo, dirigir via PS4/WASD
#   ./launch.sh --slam                            # SLAM — mapeia o ambiente em tempo real
#   ./launch.sh --nav2                            # NAV2 — navegação autônoma (mapa padrão)
#   ./launch.sh --nav2 --map=/caminho/sala.yaml   # NAV2 — mapa específico
#   ./launch.sh --trekking                        # TREKKING — ponto-a-ponto com PID (sem Nav2)
#   ./launch.sh --sim                             # SIM — Gazebo Harmonic + robô diff-drive
#   ./launch.sh --sim --slam                      # SIM + SLAM (mapeia a sala no Gazebo)
#   ./launch.sh --sim --nav2                      # SIM + NAV2 (navega com mapa salvo)
#   ./launch.sh --sim --world=worlds/sala.sdf     # SIM com mundo customizado
#
# Web teleop:
#   ./launch.sh --web-teleop                      # reativa o controle pelo browser (mux prio 50);
#                                                 # default é só visualização (movimento via PS4/WASD)
#
# Outras flags:
#   --no-lidar             desabilita o LiDAR (só modo real)
#   --lidar-port=/dev/X    sobrescreve a porta do LiDAR (padrão /dev/lidar)
#   --pi                   usa nav2_params_pi.yaml (perfil leve pra Raspberry Pi)
#
# Ctrl+C encerra todos os processos.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_SETUP="$SCRIPT_DIR/install/setup.bash"

# --- Argumentos ---
NO_LIDAR=false
LIDAR_PORT="/dev/lidar"
MODE="teleop"                     # teleop | slam | nav2 | trekking
WEB_TELEOP="off"                  # off = web só visualização; --web-teleop reativa
MAP_FILE="$SCRIPT_DIR/maps/sala.yaml"
PI_PROFILE=false
SIM=false
WORLD_FILE="$SCRIPT_DIR/worlds/empty.sdf"
SPAWN_X="2.0"
SPAWN_Y="2.5"
SPAWN_Z="0.2"
# Firmware MEGA: 'auto' = flasheia só se hash de firmware/mega_bridge mudou.
# --flash-mega força; --no-flash-mega pula sempre.
FLASH_MEGA="auto"

for arg in "$@"; do
    case $arg in
        --teleop)          MODE="teleop" ;;
        --slam)            MODE="slam" ;;
        --nav2)            MODE="nav2" ;;
        --trekking)        MODE="trekking" ;;
        --web-teleop)      WEB_TELEOP="on" ;;
        --sim)             SIM=true ;;
        --world=*)         WORLD_FILE="${arg#*=}" ;;
        --map=*)           MAP_FILE="${arg#*=}" ;;
        --spawn-x=*)       SPAWN_X="${arg#*=}" ;;
        --spawn-y=*)       SPAWN_Y="${arg#*=}" ;;
        --spawn-z=*)       SPAWN_Z="${arg#*=}" ;;
        --no-lidar)        NO_LIDAR=true ;;
        --lidar-port=*)    LIDAR_PORT="${arg#*=}" ;;
        --pi)              PI_PROFILE=true ;;
        --no-pi)           PI_PROFILE=false ;;
        --flash-mega)      FLASH_MEGA="force" ;;
        --no-flash-mega)   FLASH_MEGA="off" ;;
        --help|-h)
            echo "Uso: $0 [--teleop|--slam|--nav2|--trekking] [--sim] [--web-teleop] [--no-lidar] [--lidar-port=/dev/...] [--map=...] [--world=...] [--pi|--no-pi] [--flash-mega|--no-flash-mega]"
            echo ""
            echo "  --web-teleop     reativa o controle de movimento pela web (default: off — use PS4/WASD)"
            echo "  --flash-mega     força \`pio run -t upload\` mesmo sem mudança"
            echo "  --no-flash-mega  pula o flash da MEGA sempre"
            echo "  (sem flag)       auto: flasheia só quando o hash de firmware/mega_bridge/{src,include,platformio.ini} muda"
            exit 0
            ;;
        # Sem ramo "*) erro" o launch.sh aceitava typos como --slamm silenciosamente
        # e o usuário só descobria pelo modo TELEOP padrão. Falha rápido.
        *)
            echo "ERRO: flag desconhecida '$arg'. Use --help."
            exit 1
            ;;
    esac
done

# Auto-detecta Pi (arm64) se o usuário não passou --pi explicitamente.
if [ "$PI_PROFILE" = false ] && [ "$(uname -m)" = "aarch64" ]; then
    PI_PROFILE=true
    echo "Detectado arm64 — usando perfil --pi automaticamente (override com --no-pi)."
fi
for arg in "$@"; do
    [ "$arg" = "--no-pi" ] && PI_PROFILE=false
done

# Normaliza caminho do mundo (aceita relativo a SCRIPT_DIR)
if [ "$SIM" = true ] && [ "${WORLD_FILE:0:1}" != "/" ]; then
    WORLD_FILE="$SCRIPT_DIR/$WORLD_FILE"
fi

# Em SLAM e NAV2 o LiDAR é obrigatório (no modo real; no sim o Gazebo simula).
if [ "$SIM" = false ] && [ "$MODE" != "teleop" ] && [ "$NO_LIDAR" = true ]; then
    echo "ERRO: modo $MODE precisa do LiDAR. Remova --no-lidar."
    exit 1
fi

# SIM + TELEOP sem --web-teleop = sem nenhum publisher de movimento. Avisa.
if [ "$SIM" = true ] && [ "$MODE" = "teleop" ] && [ "$WEB_TELEOP" = "off" ]; then
    echo "[AVISO] --sim --teleop sem --web-teleop: nenhum publisher de movimento será iniciado"
    echo "        no SIM (não tem PS4/WASD nativos lá). Adicione --web-teleop pra dirigir pelo browser,"
    echo "        ou use --sim --slam/--nav2/--trekking pra ter um publisher autônomo."
fi

# Em NAV2 o arquivo de mapa precisa existir antes de subir.
if [ "$MODE" = "nav2" ] && [ ! -f "$MAP_FILE" ]; then
    echo "ERRO: mapa '$MAP_FILE' não encontrado."
    if [ "$SIM" = true ]; then
        echo "  Rode primeiro: ./launch.sh --sim --slam  (mapeie a sala e clique em 'Salvar mapa')"
    else
        echo "  Rode primeiro: ./launch.sh --slam  (mapeie a sala e clique em 'Salvar mapa')"
    fi
    exit 1
fi

# Em SIM o arquivo de mundo precisa existir.
if [ "$SIM" = true ] && [ ! -f "$WORLD_FILE" ]; then
    echo "ERRO: mundo '$WORLD_FILE' não encontrado."
    echo "  Coloque seu .sdf em $SCRIPT_DIR/worlds/ ou passe --world=/caminho/absoluto.sdf"
    exit 1
fi

# SIM requer ros_gz (Gazebo Harmonic + bridges).
if [ "$SIM" = true ]; then
    if ! ros2 pkg list 2>/dev/null | grep -q "^ros_gz_sim$"; then
        echo "ERRO: pacote ros_gz_sim não encontrado. Instale:"
        echo "  sudo apt install ros-\$ROS_DISTRO-ros-gz ros-\$ROS_DISTRO-ros-gz-sim ros-\$ROS_DISTRO-ros-gz-bridge"
        exit 1
    fi
fi

mkdir -p "$SCRIPT_DIR/maps"

WS_DIR="$SCRIPT_DIR"

# --- colcon build incremental (hash dos fontes do workspace) ---
# Pacotes vivem em ros2_packages/ — colcon descobre via --base-paths.
# Hash cobre robot_nav E wheel_msgs (incluindo .msg) — sem wheel_msgs no scan,
# alterar WheelSpeeds.msg não dispara rebuild apesar do colcon recompilá-lo.
PKG_STAMP="$WS_DIR/install/.robot_nav.sha1"
PKG_HASH=$(find "$SCRIPT_DIR/ros2_packages/robot_nav" "$SCRIPT_DIR/ros2_packages/wheel_msgs" -type f \
    \( -name "*.py" -o -name "*.xml" -o -name "*.xacro" -o -name "*.yaml" -o -name "*.msg" \) \
    -not -path "*/build/*" -not -path "*/install/*" \
    2>/dev/null | sort | xargs sha1sum 2>/dev/null | sha1sum | awk '{print $1}')

if [ ! -f "$ROS2_SETUP" ] \
   || [ ! -f "$PKG_STAMP" ] \
   || [ "$(cat "$PKG_STAMP" 2>/dev/null)" != "$PKG_HASH" ]; then
    if [ -z "$ROS_DISTRO" ]; then
        for d in /opt/ros/*/setup.bash; do
            [ -f "$d" ] && source "$d" && break
        done
    fi
    if ! command -v colcon >/dev/null 2>&1; then
        echo "ERRO: colcon não encontrado. Instale: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    if [ ! -f "$ROS2_SETUP" ]; then
        # Primeira build: compila todos os pacotes (incluindo os de terceiros).
        echo "Compilando workspace ROS2 (primeira build — todos os pacotes)..."
        (cd "$WS_DIR" && colcon build --base-paths ros2_packages --symlink-install) || {
            echo "ERRO: colcon build falhou."
            exit 1
        }
    else
        echo "Compilando workspace ROS2 (mudanças detectadas em robot_nav)..."
        (cd "$WS_DIR" && colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav wheel_msgs) || {
            echo "ERRO: colcon build falhou."
            exit 1
        }
    fi
    echo "$PKG_HASH" > "$PKG_STAMP"
fi

if [ ! -f "$ROS2_SETUP" ]; then
    echo "ERRO: $ROS2_SETUP não encontrado."
    echo "Execute: cd $SCRIPT_DIR && colcon build --base-paths ros2_packages"
    exit 1
fi

source "$ROS2_SETUP"

# --- python3-serial (dependência do mega_bridge) ---
if ! python3 -c "import serial" 2>/dev/null; then
    echo "Instalando python3-serial (sudo)..."
    sudo apt install -y python3-serial
fi

# Aviso de /dev/mega vinha aqui antes — virou redundante porque o bloco real
# (linha ~408, antes de robot.launch.py) já loga o mesmo. Mantemos só um aviso.

# --- Bootstrap do venv com dependências Python ---
VENV_DIR="$SCRIPT_DIR/controle_web/.venv"
REQ_FILE="$SCRIPT_DIR/controle_web/requirements.txt"
REQ_STAMP="$VENV_DIR/.requirements.sha1"

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "Criando venv em $VENV_DIR..."
    python3 -m venv "$VENV_DIR" || {
        echo "ERRO: falha ao criar venv. Instale python3-venv: sudo apt install python3-venv"
        exit 1
    }
fi

# Reinstala apenas se requirements.txt mudou
REQ_HASH=$(sha1sum "$REQ_FILE" | awk '{print $1}')
if [ ! -f "$REQ_STAMP" ] || [ "$(cat "$REQ_STAMP" 2>/dev/null)" != "$REQ_HASH" ]; then
    echo "Instalando dependências Python ($REQ_FILE)..."
    "$VENV_DIR/bin/pip" install --upgrade pip >/dev/null
    "$VENV_DIR/bin/pip" install -r "$REQ_FILE" || {
        echo "ERRO: falha ao instalar dependências."
        exit 1
    }
    echo "$REQ_HASH" > "$REQ_STAMP"
fi

# --- Limpa órfãos de execuções anteriores (nós ROS2 e app.py) ---
# Mesma lista usada no cleanup() ao final — uma fonte só pra evitar drift
# quando adicionar/remover um nó (M5 da AUDITORIA_2026-05-27).
KNOWN_NODE_PATTERNS=(
    "robot_nav/cmd_vel_to_wheels"
    "robot_nav/mega_bridge"
    "robot_nav/pose_estimator"
    "robot_nav/cone_detector"
    "robot_nav/trekking_runner"
    "robot_nav/unstuck_supervisor"
    "robot_nav/scan_sanitizer"
    "robot_nav/door_crossing"
    "twist_mux"
    "joy_node"
    "teleop_node"
    "collision_monitor"
    "robot_state_publisher"
    "ldlidar_stl_ros2_node"
    "async_slam_toolbox_node"
    "nav2_map_server"
    "nav2_amcl"
    "nav2_planner"
    "nav2_controller"
    "nav2_behaviors"
    "nav2_bt_navigator"
    "nav2_velocity_smoother"
    "nav2_lifecycle_manager"
    "nav2_waypoint_follower"
)
kill_known_nodes() {
    for pat in "${KNOWN_NODE_PATTERNS[@]}"; do
        pkill -9 -f "$pat" 2>/dev/null
    done
}
kill_known_nodes

# --- [opcional] Flash da MEGA (firmware/mega_bridge) ---
# Default: auto. Hash de src/, include/ e platformio.ini define quando
# refazer o upload — assim mudar só app.py ou um YAML do Nav2 não dispara
# `pio run`. Pula em SIM e quando o usuário pediu --no-flash-mega.
# pkill acima já liberou /dev/mega (mega_bridge antigo morto).
if [ "$SIM" = false ] && [ "$FLASH_MEGA" != "off" ]; then
    FW_DIR="$SCRIPT_DIR/firmware/mega_bridge"
    if [ -d "$FW_DIR" ]; then
        FW_STAMP="$FW_DIR/.pio/.flash.sha1"
        FW_HASH=$(find "$FW_DIR/src" "$FW_DIR/include" "$FW_DIR/platformio.ini" \
            -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.ini" \) 2>/dev/null \
            | sort | xargs sha1sum 2>/dev/null | sha1sum | awk '{print $1}')
        NEED_FLASH=false
        FLASH_REASON=""
        if [ "$FLASH_MEGA" = "force" ]; then
            NEED_FLASH=true
            FLASH_REASON="--flash-mega: forçando upload"
        elif [ -z "$FW_HASH" ]; then
            echo "[MEGA] não consegui calcular hash de $FW_DIR — pulando flash."
        elif [ ! -f "$FW_STAMP" ] || [ "$(cat "$FW_STAMP" 2>/dev/null)" != "$FW_HASH" ]; then
            NEED_FLASH=true
            FLASH_REASON="firmware mudou"
        fi

        # Flash é best-effort: se MEGA não está plugada ou pio não existe,
        # avisa e segue (não aborta o launch). Só `pio run` falhando vira fatal,
        # porque aí a MEGA está lá mas o upload travou — sintoma de hardware.
        if [ "$NEED_FLASH" = true ]; then
            if [ ! -e /dev/mega ]; then
                echo "[MEGA] $FLASH_REASON, mas /dev/mega ausente — pulando flash."
            elif ! command -v pio >/dev/null 2>&1; then
                echo "[MEGA] $FLASH_REASON, mas 'pio' não encontrado — pulando flash."
                echo "       Pra flashear: instale PlatformIO; pra silenciar este aviso: use --no-flash-mega."
            else
                echo "[MEGA] $FLASH_REASON — flasheando..."
                (cd "$FW_DIR" && pio run -t upload)
                FW_RC=$?
                if [ $FW_RC -eq 0 ]; then
                    mkdir -p "$FW_DIR/.pio"
                    echo "$FW_HASH" > "$FW_STAMP"
                    echo "[MEGA] flash concluído."
                else
                    echo "ERRO: pio run -t upload falhou (exit=$FW_RC)."
                    exit $FW_RC
                fi
            fi
        elif [ -n "$FW_HASH" ]; then
            echo "[MEGA] firmware atualizado (hash bate) — pulando flash."
        fi
    fi
fi

# --- Libera porta 5000 se já estiver em uso ---
# grep+cut em vez de awk match(re,arr) porque o 3-arg match() é só gawk —
# em Pi/Ubuntu o /usr/bin/awk é mawk, e o match(...,arr) ali dá "syntax error".
PORT_PID=$(ss -tlnp 2>/dev/null | grep -E ':5000 ' | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
if [ -n "$PORT_PID" ]; then
    echo "Porta 5000 em uso pelo PID $PORT_PID — encerrando antes de subir..."
    kill -9 "$PORT_PID" 2>/dev/null
    sleep 1
fi

SERVER_PID=""
ROBOT_PID=""
LIDAR_PID=""
WATCHDOG_PID=""
LIDAR_OK=false
NAV2_PID=""
SLAM_PID=""
SIM_PID=""
TAIL_PID=""

kill_tree() {
    # Mata o processo e todos os descendentes (filhos, netos...).
    # Necessário porque `ros2 launch` spawna nós filhos que não morrem
    # só matando o pai.
    local pid="$1"
    [ -z "$pid" ] && return
    local children
    children=$(pgrep -P "$pid" 2>/dev/null)
    for c in $children; do
        kill_tree "$c"
    done
    kill "$pid" 2>/dev/null
}

cleanup() {
    trap '' EXIT INT TERM
    echo ""
    echo "Encerrando todos os processos..."
    # PRIMEIRO o watchdog da LiDAR: senão ele relança o LD06 enquanto derrubamos.
    kill_tree "$WATCHDOG_PID"
    [ -n "$TAIL_PID" ]     && kill "$TAIL_PID"     2>/dev/null
    kill_tree "$SERVER_PID"
    kill_tree "$SLAM_PID"
    kill_tree "$NAV2_PID"
    kill_tree "$LIDAR_PID"
    kill_tree "$ROBOT_PID"
    kill_tree "$SIM_PID"
    sleep 1
    # Segunda passada: SIGKILL em qualquer filho que tenha sobrevivido
    for pid in $WATCHDOG_PID $SERVER_PID $SLAM_PID $NAV2_PID $LIDAR_PID $ROBOT_PID $SIM_PID; do
        for desc in $(pgrep -P "$pid" 2>/dev/null) $pid; do
            kill -9 "$desc" 2>/dev/null
        done
    done
    # Rede de segurança: mata qualquer nó conhecido órfão (mesma lista do top).
    kill_known_nodes
    # SIM-only: Gazebo + ros_gz_bridge não estão em KNOWN_NODE_PATTERNS porque
    # só sobem em --sim e o launch.sh não usa pkill deles antes do start.
    pkill -9 -f "ruby.*gz sim"                  2>/dev/null
    pkill -9 -f "gz sim"                        2>/dev/null
    pkill -9 -f "parameter_bridge"              2>/dev/null
    echo "Pronto."
    exit 0
}
trap cleanup INT TERM EXIT

LOG_DIR="$SCRIPT_DIR/controle_web/logs"
mkdir -p "$LOG_DIR"

# Health-check em vez de `sleep N` fixo: espera um tópico ROS aparecer no
# discovery (até $2 segundos). Em hardware lento (Pi 4 / SD) o sleep curto
# pode subir o próximo nó antes do anterior estar pronto; o longo desperdiça
# tempo no PC. Sai 0 quando o tópico aparece, 1 no timeout (o caller decide
# se segue mesmo assim).
wait_for_topic() {
    local topic=$1 timeout=${2:-30} elapsed=0
    while [ "$elapsed" -lt "$timeout" ]; do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "  [wait_for_topic] timeout $timeout s aguardando $topic"
    return 1
}

# /scan publicando DE FATO (não só listado no discovery)? O LD06 cria o tópico
# logo no start e só ~3s depois morre ("abnormal"), então a presença na lista
# não basta — confirma dado fluindo via `topic hz`.
lidar_scan_healthy() {
    timeout 6 ros2 topic hz /scan 2>/dev/null | grep -q "average rate"
}

# Sobe o LD06 com retry. O sensor quase nunca vinga de 1ª: ~3s após o start solta
# "ldlidar communication is abnormal" e o nó morre (exit 1). Lança, espera passar
# da janela de morte, e se caiu / sem /scan, mata, deixa a serial assentar e
# relança — até LIDAR_TRIES vezes. Seta LIDAR_PID. Retorna 0 se /scan vingou.
# Usada no BOOT e pelo watchdog de runtime (lidar_watchdog).
LIDAR_TRIES=5
start_lidar() {
    local try
    for ((try = 1; try <= LIDAR_TRIES; try++)); do
        echo "      [lidar] tentativa $try/$LIDAR_TRIES..."
        ros2 launch robot_nav lidar.launch.py lidar_port:="$LIDAR_PORT" > "$LIDAR_LOG" 2>&1 &
        LIDAR_PID=$!
        sleep 5   # passa a janela do "abnormal" antes de julgar
        if pgrep -f ldlidar_stl_ros2_node >/dev/null 2>&1 \
           && wait_for_topic /scan 5 && lidar_scan_healthy; then
            echo "      [lidar] OK — /scan publicando (PID $LIDAR_PID, tentativa $try)."
            return 0
        fi
        echo "      [lidar] caiu / sem /scan — matando e repetindo."
        kill_tree "$LIDAR_PID"
        LIDAR_PID=""
        sleep 2   # deixa a porta serial assentar antes de reabrir
    done
    return 1
}

# Watchdog de RUNTIME (o retry acima é só no BOOT). Visto 2026-06-09: o LD06 subiu,
# o robô navegou, e o nó MORREU no meio da operação → /scan mudo → nav2 parou, sem
# ninguém relançar. Aqui monitoramos a liveness do nó e relançamos. Checagem BARATA
# por processo (pgrep): o "abnormal" MATA o nó (exit 1) — é o modo recuperável por
# software. NÃO chamamos `ros2 topic hz` em loop (cria nó + 6s a cada ciclo = CPU
# cara nesta Pi); o caso "nó vivo mas /scan mudo" é HW travado (precisa replug),
# que relançar não resolve. Back-off quando o relance falha (não martelar física).
LIDAR_WATCH_INTERVAL="${LIDAR_WATCH_INTERVAL:-15}"
lidar_watchdog() {
    local fails=0
    while true; do
        sleep "$LIDAR_WATCH_INTERVAL"
        if pgrep -f ldlidar_stl_ros2_node >/dev/null 2>&1; then
            fails=0
            continue
        fi
        echo "  [lidar-watchdog] nó da LiDAR caiu em runtime — relançando..."
        kill_tree "$LIDAR_PID"; LIDAR_PID=""
        if start_lidar; then
            echo "  [lidar-watchdog] LiDAR recuperada."
            fails=0
        else
            fails=$((fails + 1))
            echo "  [lidar-watchdog] não recuperou (falha #$fails) — provável HW (replug/power). Back-off."
            sleep $((LIDAR_WATCH_INTERVAL * 4))
        fi
    done
}

if [ "$SIM" = true ]; then
    # --- [SIM] Gazebo Harmonic + robô diff-drive + bridges ROS↔GZ ---
    echo "[1/4] Modo SIM — subindo Gazebo com mundo: $WORLD_FILE"
    SIM_LOG="$LOG_DIR/sim.log"
    SIM_WORLD="$WORLD_FILE" ros2 launch robot_nav sim.launch.py \
        world:="$WORLD_FILE" \
        spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y" spawn_z:="$SPAWN_Z" > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    echo "      PID: $SIM_PID  |  Log: $SIM_LOG"
    # Espera o /clock vir do bridge GZ → ROS antes de seguir.
    wait_for_topic /clock 30 || echo "  AVISO: Gazebo ainda não publicou /clock — seguindo mesmo assim."
    # Hardware desligado no sim
    NO_LIDAR=true
else
    # --- [1+2] Nós do robô (mega_bridge + URDF + odom + cmd_vel_to_wheels) ---
    echo "[1/4] Iniciando nós do robô (MEGA bridge, URDF, odometria, cmd_vel->wheels)..."
    if [ ! -e "/dev/mega" ]; then
        echo "      AVISO: /dev/mega não encontrado — rode sudo ./setup_udev.sh primeiro,"
        echo "      ou plug a Arduino MEGA antes de subir."
    fi
    ROBOT_LOG="$LOG_DIR/robot_nodes.log"
    ros2 launch robot_nav robot.launch.py > "$ROBOT_LOG" 2>&1 &
    ROBOT_PID=$!
    echo "      PID: $ROBOT_PID  |  Log: $ROBOT_LOG"

    wait_for_topic /odom 15 || echo "  AVISO: pose_estimator ainda não publicou /odom — seguindo."

    # --- [3] LiDAR LD06 + detector de obstáculos ---
    if [ "$NO_LIDAR" = false ]; then
        if [ -e "$LIDAR_PORT" ]; then
            echo "[2/4] Iniciando LiDAR LD06 em $LIDAR_PORT..."
            LIDAR_LOG="$LOG_DIR/lidar.log"
            # Retry no BOOT via start_lidar(); o watchdog de runtime (lidar_watchdog,
            # iniciado mais abaixo) cuida das mortes do LD06 DURANTE a operação.
            if start_lidar; then
                LIDAR_OK=true
            else
                echo "  AVISO: LiDAR não subiu após $LIDAR_TRIES tentativas — seguindo sem /scan."
            fi
        else
            echo "[2/4] AVISO: Porta do LiDAR $LIDAR_PORT não encontrada. Pulando LiDAR."
            echo "      Para especificar outra porta: ./launch.sh --lidar-port=/dev/ttyUSB2"
            NO_LIDAR=true
        fi
    else
        echo "[2/4] LiDAR desativado (--no-lidar)"
    fi
fi

# --- [4] SLAM ou Nav2 ou Collision Monitor (conforme modo) ---
SIM_TIME_ARG="use_sim_time:=false"
if [ "$SIM" = true ]; then
    SIM_TIME_ARG="use_sim_time:=true"
fi

case "$MODE" in
    slam)
        echo "[3/4] Modo SLAM — subindo slam_toolbox (mapping online)..."
        SLAM_LOG="$LOG_DIR/slam.log"
        ros2 launch robot_nav slam.launch.py $SIM_TIME_ARG > "$SLAM_LOG" 2>&1 &
        SLAM_PID=$!
        echo "      PID: $SLAM_PID  |  Log: $SLAM_LOG"
        wait_for_topic /map 30 || echo "  AVISO: slam_toolbox ainda não publicou /map — seguindo."
        ;;
    nav2)
        NAV2_PARAMS_ARG=""
        if [ "$PI_PROFILE" = true ]; then
            PI_YAML="$(ros2 pkg prefix robot_nav 2>/dev/null)/share/robot_nav/config/nav2_params_pi.yaml"
            if [ -f "$PI_YAML" ]; then
                NAV2_PARAMS_ARG="params_file:=$PI_YAML"
                echo "[3/4] Modo NAV2 (perfil PI) — params: $PI_YAML"
            else
                echo "[3/4] Modo NAV2 — aviso: nav2_params_pi.yaml não encontrado, usando defaults"
            fi
        else
            echo "[3/4] Modo NAV2 — subindo Nav2 com mapa $MAP_FILE..."
        fi
        NAV2_LOG="$LOG_DIR/nav2.log"
        ros2 launch robot_nav nav2.launch.py map:="$MAP_FILE" $SIM_TIME_ARG $NAV2_PARAMS_ARG > "$NAV2_LOG" 2>&1 &
        NAV2_PID=$!
        echo "      PID: $NAV2_PID  |  Log: $NAV2_LOG"
        # Nav2 demora pra ativar todos os lifecycle nodes; espera o costmap global.
        wait_for_topic /global_costmap/costmap 30 || echo "  AVISO: Nav2 ainda não publicou /global_costmap/costmap — seguindo."
        ;;
    trekking)
        echo "[3/4] Modo TREKKING — subindo cone_detector + trekking_runner (pose_estimator já vem do robot.launch)..."
        NAV2_LOG="$LOG_DIR/trekking.log"
        ros2 launch robot_nav trekking.launch.py > "$NAV2_LOG" 2>&1 &
        NAV2_PID=$!
        echo "      PID: $NAV2_PID  |  Log: $NAV2_LOG"
        wait_for_topic /trekking/pose 15 || echo "  AVISO: trekking_runner ainda não publicou /trekking/pose — seguindo."
        ;;
    teleop)
        echo "[3/4] Modo TELEOP — dirija manualmente (sem camada extra de segurança)."
        ;;
esac

# --- [5] Servidor web ---
echo ""
SIM_TAG=""
[ "$SIM" = true ] && SIM_TAG=" [SIM/Gazebo]"
echo "[4/4] Iniciando servidor web em http://0.0.0.0:5000 (modo: $MODE$SIM_TAG)"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
case "$MODE" in
    slam)
        echo "  MODO SLAM$SIM_TAG — dirija o robô para mapear. Salve o mapa pelo botão web."
        echo "  slam_toolbox publicando /map (~1 Hz) e TF map→odom."
        ;;
    nav2)
        echo "  MODO NAV2$SIM_TAG — clique no mapa web para enviar o robô a um destino."
        echo "  Mapa: $MAP_FILE"
        echo "  AMCL publicando map→odom. bt_navigator consome /goal_pose."
        ;;
    trekking)
        echo "  MODO TREKKING$SIM_TAG — ponto-a-ponto com PID e snap-to-cone."
        echo "  1) Aperte ● Gravar  2) dirija até cada ponto e + Ponto"
        echo "  3) volte ao início  4) ▶ Play"
        ;;
    teleop)
        echo "  MODO TELEOP$SIM_TAG — Web → /cmd_vel → robô"
        ;;
esac
if [ "$SIM" = true ]; then
    echo "  Mundo Gazebo: $WORLD_FILE"
    echo "  Robô simulado publicando /scan, /odom e TF odom→base_link"
elif [ "$NO_LIDAR" = false ]; then
    echo "  LiDAR LD06 publicando em: /scan"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Watchdog de runtime da LiDAR: só se ela subiu no boot (LIDAR_OK) e é hardware
# real. Roda em background; cleanup() o mata ANTES de derrubar a LiDAR (senão
# ressuscita no shutdown). Se o LD06 não vingou no boot, não vigia (provável HW).
if [ "$SIM" = false ] && [ "$NO_LIDAR" = false ] && [ "$LIDAR_OK" = true ]; then
    lidar_watchdog &
    WATCHDOG_PID=$!
    echo "  [lidar-watchdog] ativo (PID $WATCHDOG_PID — checa o LD06 a cada ${LIDAR_WATCH_INTERVAL}s)."
fi

cd "$SCRIPT_DIR/controle_web"
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
fi

echo "Logs dos nós em $LOG_DIR/ (ex: tail -f $LOG_DIR/robot_nodes.log)"
echo ""

# Passa o modo e o diretório de mapas para o app.py via env.
export ROBOT_MODE="$MODE"
export WEB_TELEOP="$WEB_TELEOP"
export ROBOT_MAPS_DIR="$SCRIPT_DIR/maps"
export ROBOT_MAP_FILE="$MAP_FILE"
export ROBOT_SIM="$SIM"

# Servidor em primeiro plano — Ctrl+C aqui dispara cleanup() via trap.
python3 app.py
SERVER_EXIT=$?
echo "Servidor web encerrou (exit=$SERVER_EXIT)"
