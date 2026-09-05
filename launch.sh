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
MAP_FILE="$SCRIPT_DIR/maps/hotmilk_portas.yaml"
PI_PROFILE=false
SIM=false
ARENA=false                       # --arena: perfil da prova do galpao (05/09)
FECHA_FRESTA=false                # --fecha-fresta: BOTAO DE PANICO (ver PERFIL_ARENA)
WORLD_FILE="$SCRIPT_DIR/worlds/sala.sdf"   # default: sala c/ paredes+obstáculos+porta 0.93m (empty.sdf = template vazio)
SPAWN_X="2.0"
SPAWN_Y="2.5"
SPAWN_Z="0.2"
# Firmware MEGA: 'auto' = flasheia só se hash de firmware/mega_bridge mudou.
# --flash-mega força; --no-flash-mega pula sempre.
FLASH_MEGA="auto"
# Teto de velocidade do path_follower (o teto EFETIVO da autonomia — ele ganha
# o twist_mux_auto do nav_vel). Vazio = cada perfil usa o seu default:
# --arena 0.35, resto o default do launch (0.30). `--follow-speed=X` sobrescreve
# SEM desmontar o perfil: e' o rollback de campo do degrau de velocidade, ja que
# `ros2 param set` nao funciona (o no' le' parametro so' no __init__) e subir
# sem --arena trocaria junto mapa, guard, door_crossing e velocidade-por-folga.
FOLLOW_SPEED=""
# O operador passou o argumento na mao? So' entao o --arena NAO sobrescreve.
MAP_EXPLICITO=false
WORLD_EXPLICITO=false
SPAWN_EXPLICITO=false

for arg in "$@"; do
    case $arg in
        --teleop)          MODE="teleop" ;;
        --slam)            MODE="slam" ;;
        --nav2)            MODE="nav2" ;;
        --trekking)        MODE="trekking" ;;
        --web-teleop)      WEB_TELEOP="on" ;;
        --sim)             SIM=true ;;
        --world=*)         WORLD_FILE="${arg#*=}"; WORLD_EXPLICITO=true ;;
        --map=*)           MAP_FILE="${arg#*=}";   MAP_EXPLICITO=true ;;
        --spawn-x=*)       SPAWN_X="${arg#*=}";    SPAWN_EXPLICITO=true ;;
        --spawn-y=*)       SPAWN_Y="${arg#*=}";    SPAWN_EXPLICITO=true ;;
        --spawn-z=*)       SPAWN_Z="${arg#*=}" ;;
        --no-lidar)        NO_LIDAR=true ;;
        --lidar-port=*)    LIDAR_PORT="${arg#*=}" ;;
        --pi)              PI_PROFILE=true ;;
        --no-pi)           PI_PROFILE=false ;;
        --arena)           ARENA=true ;;
        --follow-speed=*)
            FOLLOW_SPEED="${arg#*=}"
            # Este numero vai DIRETO pro teto de velocidade do no' que dirige o
            # robo. Um dedo gordo (3.5 em vez de 0.35) seria aceito calado e
            # mandado pro path_follower. Valida formato E faixa.
            case "$FOLLOW_SPEED" in
                ''|*[!0-9.]*|*.*.*)
                    echo "ERRO: --follow-speed='$FOLLOW_SPEED' nao e' um numero." >&2
                    exit 1 ;;
            esac
            # FAIXA [0.22, 0.35].
            #
            # TETO 0.35: e' o degrau em teste, e o unico valor com baseline pra
            # comparar. Acima disso ninguem mediu frenagem REAL — o follow_vel
            # nao passa pelo velocity_smoother, entao o decel_lim_x do YAML nao
            # prova nada. Pra subir: medir primeiro
            # (docs/baselines/2026-09-05-arena-velocidade-teto-035/).
            #
            # PISO 0.22 = min_speed do path_follower (achado do review 09-05).
            # Abaixo dele o speed_for_clearance INVERTE, nao so' rasteja: ele
            # interpola entre min_speed (folga <= clear_min) e forward_speed
            # (folga >= clear_full), entao com forward_speed 0.10 o robo anda
            # 0.22 APERTADO e 0.10 LIVRE — mais rapido onde e' perigoso. Medido
            # com o codigo real: folga 0.30 m -> 0.220 | 1.20 m -> 0.100.
            # (O mesmo vale pra freada de chegada da linha 588, que tambem tem
            # piso em min_speed.) E 0.11 ja' e' zona-morta do chassi: nao anda.
            # Pra andar mais devagar de proposito, o botao e' min_speed, junto.
            if [ "$(awk -v v="$FOLLOW_SPEED" 'BEGIN{print (v>=0.22 && v<=0.35)?1:0}')" != 1 ]; then
                echo "ERRO: --follow-speed=$FOLLOW_SPEED fora da faixa [0.22, 0.35]." >&2
                echo "      Teto 0.35: e' o degrau em teste; acima nao ha medicao de frenagem real." >&2
                echo "      Piso 0.22: e' o min_speed do path_follower — abaixo dele a" >&2
                echo "      velocidade-por-folga INVERTE (anda mais rapido no apertado)." >&2
                echo "      Ver docs/baselines/2026-09-05-arena-velocidade-teto-035/" >&2
                exit 1
            fi ;;
        --fecha-fresta)    FECHA_FRESTA=true ;;
        --flash-mega)      FLASH_MEGA="force" ;;
        --no-flash-mega)   FLASH_MEGA="off" ;;
        --help|-h)
            echo "Uso: $0 [--teleop|--slam|--nav2|--trekking] [--sim] [--arena] [--follow-speed=X] [--fecha-fresta] [--web-teleop] [--no-lidar] [--lidar-port=/dev/...] [--map=...] [--world=...] [--pi|--no-pi] [--flash-mega|--no-flash-mega]"
            echo ""
            echo "  --arena          perfil da prova do galpao (05/09): mundo arena_galpao.sdf,"
            echo "                   mapa arena_galpao.yaml (fresta A ABERTA -> o robo PASSA),"
            echo "                   spawn 1.0/1.0, motion_guard OFF, robot_radius 0.32,"
            echo "                   inflacao maior, PolygonFront unico. Na geometria continua"
            echo "                   isso fecha vao < 0.64 m — CONFIRME no mapa rasterizado com"
            echo "                   tools/mapa_passagens.py. NAO cobre raspao em point-turn."
            echo "  --follow-speed=X teto de velocidade do path_follower (m/s). E' o teto EFETIVO"
            echo "                   da autonomia; o max_vel_x do nav2_params NAO manda no robo."
            echo "                   Faixa [0.22, 0.35]. Default: 0.35 com --arena, 0.30 sem."
            echo "                   Rollback de campo do"
            echo "                   degrau SEM desmontar o perfil (restart e' obrigatorio: o no"
            echo "                   le parametro so no __init__, 'ros2 param set' nao funciona)."
            echo "  --fecha-fresta   BOTAO DE PANICO: troca o mapa do --arena pelo TAMPADO"
            echo "                   (arena_galpao_semA.yaml). A fresta A de 0,90 m fica FECHADA"
            echo "                   so' pro planejador (o vao segue aberto no mundo) e o robo"
            echo "                   CONTORNA. 4 voltas medidas com 0 colisao e 0 raspao."
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

# --- PERFIL ARENA: mapa/mundo/spawn da prova de 05/09 ------------------------
# 2026-09-01 (decisao do dono, DIARIO_ARENA §2G): `--arena` era SO' o perfil de
# params — mundo, mapa e spawn ficavam nos defaults (sala.sdf + hotmilk_portas +
# spawn 2.0/2.5, que na arena cai EM CIMA do muro sul). Isso ja' tinha feito um
# bloco inteiro da §4 do diario "rodar a arena" sem rodar a arena. Agora o
# --arena carrega a prova inteira, e cada peca so' e' sobrescrita se o operador
# passou o argumento na mao.
#
# 2026-09-02 (decisao do dono, DIARIO_ARENA §2G.10): *"vamos ter que fazer ele
# passar nessa porra 100% das vezes"*. O MAPA default do --arena volta a ser o
# ABERTO (`arena_galpao.yaml`) — a fresta A de 0,90 m existe pro planejador e o
# robo PASSA por ela. Entre 01/09 e 02/09 o default era o TAMPADO
# (`arena_galpao_semA.yaml`), que fazia o robo CONTORNAR; isso virou a rede de
# seguranca e agora se pede com `--fecha-fresta` (botao de panico, 4 voltas
# medidas com 0 colisao e 0 raspao em docs/baselines/
# 2026-09-01-arena-contorno-fresta-A/).
#
# ATENCAO: o mapa aberto sozinho NAO faz o robo passar com seguranca — o
# orcamento lateral do vao e' +-0,20 m e o erro do AMCL medido nesta arena chega
# a 0,49 m (§2G.10). Quem controla a travessia e' o `door_crossing` (scan-relative);
# rodar com o mapa aberto SEM ele e' a configuracao que bateu na fresta em 1 de 3
# voltas (`noguard3`).
# O test_arena_perfil_prova.py EXECUTA o bloco entre os marcadores abaixo (em vez
# de reconstruir a logica, que seria tautologia). Nao renomeie os marcadores sem
# ajustar o teste, e mantenha cada um sozinho na sua linha.
# >>> PERFIL_ARENA_DEFAULTS
if [ "$ARENA" = true ]; then
    _ARENA_MAPA="arena_galpao.yaml"
    [ "$FECHA_FRESTA" = true ] && _ARENA_MAPA="arena_galpao_semA.yaml"
    [ "$MAP_EXPLICITO"   = false ] && MAP_FILE="$SCRIPT_DIR/maps/$_ARENA_MAPA"
    [ "$WORLD_EXPLICITO" = false ] && WORLD_FILE="$SCRIPT_DIR/worlds/arena_galpao.sdf"
    if [ "$SPAWN_EXPLICITO" = false ]; then SPAWN_X="1.0"; SPAWN_Y="1.0"; fi
    # FALHA FECHADA nos DOIS sentidos: o mapa pedido tem que existir. Se falta o
    # TAMPADO, cair no aberto em silencio mandaria o robo pela fresta justamente
    # quando o operador apertou o botao de panico; se falta o ABERTO, cair no
    # default de fora da arena (hotmilk_portas) subiria a prova com o mapa errado.
    if [ "$MAP_EXPLICITO" = false ] && [ ! -f "$MAP_FILE" ]; then
        echo "ERRO: --arena pedido, mas o mapa da prova nao existe:"
        echo "      $MAP_FILE"
        if [ "$FECHA_FRESTA" = true ]; then
            echo "      Gere com: python3 tools/gera_arena_galpao.py --mapa maps/ --fecha-fresta A"
        else
            echo "      Gere com: python3 tools/gera_arena_galpao.py --mapa maps/"
        fi
        echo "      Abortando (nao vou subir a prova com o mapa errado por acidente)."
        exit 1
    fi
fi
# <<< PERFIL_ARENA_DEFAULTS

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
        # FIDELIDADE SIM=REAL: o --sim também usa nav2_params_pi.yaml (a config que
        # roda no robô). Sem isto o sim caía no nav2_params.yaml antigo (DWB puro,
        # sem RotationShim, max_vel_theta 0.8) → outro nav, resultado que não vale
        # pro real. Ver ESTADO_PROJETO.md (análise de lacunas sim vs real).
        if [ "$PI_PROFILE" = true ] || [ "$SIM" = true ] || [ "$ARENA" = true ]; then
            _PARAMS_NAME="nav2_params_pi.yaml"
            # 2026-08-28: perfil ARENA (galpao 05/09). Mesma stack; so a geometria
            # muda — ver config/nav2_params_arena.yaml e
            # docs/superpowers/specs/2026-08-28-arena-galpao-design.md
            [ "$ARENA" = true ] && _PARAMS_NAME="nav2_params_arena.yaml"
            PI_YAML="$(ros2 pkg prefix robot_nav 2>/dev/null)/share/robot_nav/config/$_PARAMS_NAME"
            if [ -f "$PI_YAML" ]; then
                NAV2_PARAMS_ARG="params_file:=$PI_YAML"
                _perfil="PI"; [ "$SIM" = true ] && _perfil="SIM=REAL"
                [ "$ARENA" = true ] && _perfil="ARENA"
                echo "[3/4] Modo NAV2 (perfil $_perfil) — params: $PI_YAML"
            elif [ "$ARENA" = true ]; then
                # FALHA FECHADA: --arena troca a GEOMETRIA DE SEGURANCA (raio,
                # inflacao, collision). Cair no default em silencio subiria o robo
                # com outro footprint do que o operador pediu — exatamente o tipo
                # de divergencia que a prova existe pra nao ter.
                echo "ERRO: --arena pedido, mas $_PARAMS_NAME nao foi encontrado em"
                echo "      $(ros2 pkg prefix robot_nav 2>/dev/null)/share/robot_nav/config/"
                echo "      Rode um colcon build antes. Abortando (nao vou subir com outra geometria)."
                exit 1
            else
                echo "[3/4] Modo NAV2 — aviso: $_PARAMS_NAME não encontrado, usando defaults"
            fi
        else
            echo "[3/4] Modo NAV2 — subindo Nav2 com mapa $MAP_FILE..."
        fi
        NAV2_LOG="$LOG_DIR/nav2.log"
        # SIM: o spawn é fixo e conhecido → o AMCL já nasce localizado lá
        # (set_initial_pose), sem ter que setar /initialpose toda vez. No real fica off.
        INIT_POSE_ARG=""
        if [ "$SIM" = true ]; then
            INIT_POSE_ARG="set_initial_pose:=true init_x:=$SPAWN_X init_y:=$SPAWN_Y init_yaw:=0.0"
            echo "      [SIM] AMCL nasce em ($SPAWN_X, $SPAWN_Y, yaw 0) — sem setar pose na mão"
        fi
        # A velocidade-por-folga do path_follower nao mora no params_file (o nó
        # nao le' ele) — entra por launch arg, so' no perfil ARENA.
        # 2026-09-05 (fase VELOCIDADE reaberta): follow_forward_speed 0.30 -> 0.35
        # SO' na arena. Este e' o teto EFETIVO — o path_follower ganha o
        # twist_mux_auto (prio 15) do nav_vel (10), entao mexer no max_vel_x do
        # nav2_params NAO acelera o robo. 0.35 apenas alinha o follower aos tetos
        # que a cadeia nav_vel ja tem; nada mais precisa se mover.
        # ⚠️ NAO subir pra 0.50/0.60 sem medir frenagem REAL: o follow_vel nao
        # passa pelo velocity_smoother, entao o decel_lim_x do YAML nao prova a
        # desaceleracao fisica. Baseline do 0.30 e a analise inteira em
        # docs/baselines/2026-09-05-arena-velocidade-teto-035/.
        #
        # ⚠️ ROLLBACK E' RESTART, NAO `ros2 param set` (achado do review 09-05):
        # o path_follower le os parametros UMA VEZ no __init__ e congela em
        # self.cfg (path_follower.py:642); nao ha add_on_set_parameters_callback.
        # `ros2 param set /path_follower forward_speed 0.30` muda o valor no
        # servidor de parametros e o no' SEGUE A 0.35 — silenciosamente.
        # Pra voltar: Ctrl-C e subir de novo com `--follow-speed=0.30`, que
        # mantem o perfil inteiro. NAO subir sem --arena pra baixar velocidade:
        # isso troca junto mapa, motion_guard, door_crossing e a
        # velocidade-por-folga — era a recomendacao velha, e estava errada.
        #
        # O test_arena_perfil_prova.py EXECUTA o bloco entre os marcadores
        # abaixo (em vez de reimplementar a logica e virar tautologia — BO 63).
        # >>> PERFIL_ARENA_FOLLOW
        ARENA_FOLLOW_ARG=""
        [ "$ARENA" = true ] && ARENA_FOLLOW_ARG="follow_clear_full:=1.2 follow_clear_min:=0.35 follow_forward_speed:=0.35"
        # `--follow-speed=X` vence o default do perfil e NAO desmonta nada mais:
        # o resto do ARENA_FOLLOW_ARG (clear_full/clear_min), o mapa, o guard e o
        # door_crossing ficam de pe'. Vale com ou sem --arena.
        if [ -n "$FOLLOW_SPEED" ]; then
            ARENA_FOLLOW_ARG="$(echo "$ARENA_FOLLOW_ARG" | sed 's/ *follow_forward_speed:=[^ ]*//')"
            ARENA_FOLLOW_ARG="${ARENA_FOLLOW_ARG:+$ARENA_FOLLOW_ARG }follow_forward_speed:=$FOLLOW_SPEED"
        fi
        # <<< PERFIL_ARENA_FOLLOW
        # motion_guard DESLIGADO na arena (dono, 2026-08-31). Ele e' o vigia de
        # PESSOA; a arena da prova nao tem pessoa, tem CONE — e a vigilia fechava
        # em cima do cone e ZERAVA o comando por ~27 s (3 episodios em 11 voltas,
        # DIARIO_ARENA §2B.7). Fora da arena ele continua ligado por default.
        #
        # ⚠️ ISTO VALE PARA O ROBO REAL TAMBEM (achado do review 2026-08-31): a
        # medicao que justificou desligar foi feita no SIM, e a ausencia de
        # <actor> no mundo so prova que o MUNDO SIMULADO nao tem gente. No real,
        # `--arena` sem `--sim` sobe o robo SEM o vigia de pessoa. So' e'
        # aceitavel com arena controlada, gente fora da pista e E-STOP humano na
        # mao. O collision_monitor continua ligado, mas ele e' reflexo geometrico
        # de obstaculo — nao substitui o vigia de coisa que se MOVE.
        ARENA_GUARD_ARG=""
        if [ "$ARENA" = true ]; then
            ARENA_GUARD_ARG="motion_guard:=false"
            if [ "$SIM" != true ]; then
                echo "      ⚠️  ARENA no ROBO REAL: motion_guard DESLIGADO (vigia de pessoa)."
                echo "          Exija pista sem gente e E-STOP na mao. Ligar de volta:"
                echo "          rode sem --arena, ou passe motion_guard:=true no launch."
            fi
        fi
        # door_crossing SO' na arena (2026-09-02, DIARIO_ARENA §2H.4): na fresta
        # A de 0,90 m o seguidor entra SEMPRE torto (13 travessias medidas:
        # -4,8° a -15,8°) e com desvio de ate' 12,1 cm — folga de 3,7 cm no pior
        # caso, que foi contato. O door_crossing zera as duas parcelas antes de
        # entrar. As portas vem do <mapa>.doors.json, que o gerador escreve junto
        # com o mapa: assim o robo NUNCA arma travessia numa fresta que o mapa
        # que ele carregou trata como parede.
        # >>> PERFIL_ARENA_DOOR
        ARENA_DOOR_ARG=""
        if [ "$ARENA" = true ]; then
            _DOORS_FILE="${MAP_FILE%.yaml}.doors.json"
            # FALHA FECHADA AQUI, e nao dentro do no' (decisao 2026-09-02, review):
            # se o arquivo falta ou esta' malformado, o no' subiria e ficaria
            # `idle` — e no' idle e' indistinguivel de no' MORTO: nos dois casos
            # ninguem dirige a travessia, que e' exatamente o caso que bateu
            # (noguard3). Matar o no' nao protege mais que logar. Quem protege e'
            # NAO SUBIR A STACK, que so' da' pra fazer aqui, antes do launch — e
            # com o erro na tela do operador, nao enterrado no nav2.log.
            if [ ! -f "$_DOORS_FILE" ]; then
                echo "ERRO: --arena pedido, mas as portas do mapa nao existem:"
                echo "      $_DOORS_FILE"
                echo "      Gere com: python3 tools/gera_arena_galpao.py --mapa maps/"
                echo "      Abortando (nao vou subir a travessia da fresta sem porta marcada)."
                exit 1
            fi
            # Conteudo, nao so' existencia: valida com a MESMA funcao que o no'
            # usa (nada de reimplementar o schema aqui — seria outra fonte de
            # verdade divergindo em silencio).
            if ! _DOORS_OUT=$(PYTHONPATH="$SCRIPT_DIR/ros2_packages/robot_nav" \
                    python3 -c '
import sys
from robot_nav.door_crossing import doors_de_arquivo
try:
    print(len(doors_de_arquivo(sys.argv[1])))
except ValueError as e:          # mensagem limpa, sem traceback na tela
    sys.stderr.write(str(e) + "\n"); sys.exit(1)
' "$_DOORS_FILE" 2>&1); then
                echo "ERRO: --arena pedido, mas as portas do mapa nao prestam:"
                echo "      $_DOORS_FILE"
                echo "      $_DOORS_OUT"
                echo "      Regere com: python3 tools/gera_arena_galpao.py --mapa maps/"
                echo "      Abortando (nao vou subir a travessia com porta invalida)."
                exit 1
            fi
            echo "      [ARENA] door_crossing LIGADO — $_DOORS_OUT porta(s) em $(basename "$_DOORS_FILE")"
            ARENA_DOOR_ARG="door_crossing:=true doors_file:=$_DOORS_FILE"
        fi
        # <<< PERFIL_ARENA_DOOR
        ros2 launch robot_nav nav2.launch.py map:="$MAP_FILE" $SIM_TIME_ARG $NAV2_PARAMS_ARG $INIT_POSE_ARG $ARENA_FOLLOW_ARG $ARENA_GUARD_ARG $ARENA_DOOR_ARG > "$NAV2_LOG" 2>&1 &
        NAV2_PID=$!
        echo "      PID: $NAV2_PID  |  Log: $NAV2_LOG"
        # Nav2 demora pra ativar todos os lifecycle nodes; espera o costmap global.
        wait_for_topic /global_costmap/costmap 30 || echo "  AVISO: Nav2 ainda não publicou /global_costmap/costmap — seguindo."
        ;;
    trekking)
        echo "[3/4] Modo TREKKING — subindo cone_detector + trekking_runner (pose_estimator já vem do robot.launch)..."
        NAV2_LOG="$LOG_DIR/trekking.log"
        # SIM: o pose_estimator (quem publica /trekking/pose no real) vive no
        # robot.launch.py, que o --sim não usa -> sem isto o trekking fica sem
        # pose e NÃO SAI DO LUGAR no Play.
        TREK_SIM_ARG=""
        [ "$SIM" = true ] && TREK_SIM_ARG="sim_pose_from_odom:=true"
        # TREK_EXTRA_ARGS: passagem direta pro trekking.launch.py, sem precisar
        # de uma flag nova no launch.sh a cada experimento. Ex.:
        #   TREK_EXTRA_ARGS="cone_fix_repeat:=true" ./launch.sh --sim --trekking
        ros2 launch robot_nav trekking.launch.py $SIM_TIME_ARG $TREK_SIM_ARG \
            ${TREK_EXTRA_ARGS:-} > "$NAV2_LOG" 2>&1 &
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
