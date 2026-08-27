#!/bin/bash
# Mata tudo que uma volta do A/B deixa pra trás — inclusive os MEUS scripts
# (probe.py), que já contaminaram uma medição inteira em 2026-08-27.
# NUNCA mata a própria árvore de processos (self + ancestrais), senão o script
# que chama isto morre no meio do setup.
PADRAO='ab/probe\.py|ab/colisao\.py|gz sim|ruby.*gz|parameter_bridge|ros_gz_bridge|nav2_trekking/lib|robot_nav/lib|nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_waypoint_follower|nav2_velocity_smoother|nav2_collision_monitor|nav2_lifecycle_manager|twist_mux|robot_state_publisher|teleop_twist_joy|teleop_node|joy_node|joy_linux|ros2 launch (nav2_trekking|robot_nav)'

# ancestrais do processo atual: protegidos
PROTEGIDOS=" "
p=$$
while [ "$p" -gt 1 ]; do
    PROTEGIDOS="$PROTEGIDOS$p "
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    [ -z "$p" ] && break
done

for tentativa in 1 2 3 4 5; do
    alvos=""
    for pid in $(pgrep -f "$PADRAO" 2>/dev/null); do
        case "$PROTEGIDOS" in *" $pid "*) continue ;; esac
        alvos="$alvos $pid"
    done
    if [ -z "$alvos" ]; then
        [ "$tentativa" -gt 1 ] && echo "[kill_all] limpo após $tentativa tentativa(s)" || echo "[kill_all] já estava limpo"
        exit 0
    fi
    for pid in $alvos; do kill -9 "$pid" 2>/dev/null; done
    sleep 2
done
echo "[kill_all] FALHOU — ainda vivos:$alvos"
exit 1
