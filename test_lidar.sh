#!/bin/bash
# Teste rápido do LiDAR LD06: sobe o driver + abre o RViz já configurado.
# Uso: ./test_lidar.sh
# Ctrl+C encerra os dois processos.

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
RVIZ_CFG="$HERE/ros2_packages/robot_nav/rviz/ld06_test.rviz"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Aborta se já houver um nó do LD06 rodando (senão dois processos disputam
# /dev/lidar e o stream corrompe: "get ldlidar data is time out").
if pgrep -x ldlidar_stl_ros2_node >/dev/null; then
    echo "ERRO: já existe um nó ldlidar_stl_ros2_node rodando (segurando /dev/lidar)."
    echo "      Encerre-o antes: pkill -9 -x ldlidar_stl_ros2_node"
    exit 1
fi

# Mata o driver e o rviz ao sair (Ctrl+C). 'ros2 run' cria processos-filho,
# então o pkill no executável garante que a porta serial seja liberada.
cleanup() {
    echo ""
    echo "Encerrando LiDAR e RViz..."
    kill "$RVIZ_PID" 2>/dev/null || true
    kill "$LIDAR_PID" 2>/dev/null || true
    pkill -9 -x ldlidar_stl_ros2_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">> Iniciando driver do LD06 em /dev/lidar ..."
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
    -p product_name:=LDLiDAR_LD06 \
    -p topic_name:=scan \
    -p frame_id:=base_laser \
    -p port_name:=/dev/lidar \
    -p port_baudrate:=230400 \
    -p laser_scan_dir:=true &
LIDAR_PID=$!

sleep 3

echo ">> Abrindo RViz (Fixed Frame=base_laser, LaserScan em /scan) ..."
rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

echo ""
echo "LiDAR + RViz rodando. Pressione Ctrl+C aqui para encerrar."
wait
