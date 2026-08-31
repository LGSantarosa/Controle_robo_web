#!/bin/bash
# Mata tudo que uma volta do A/B deixa pra trás — inclusive os MEUS scripts
# (probe.py), que já contaminaram uma medição inteira em 2026-08-27.
# 2026-08-31: entrou `gz-transport-topic` (e `gz topic`). O colisao.py abre um
# subscriber de pose como PROCESSO FILHO; matar o colisao.py com kill -9 NÃO
# mata o filho, e o binário real é
# .../gz_transport_vendor/libexec/gz/transport13/gz-transport-topic — que não
# casava com `gz sim` nem com `ruby.*gz`. Ficou 17 min vivo depois da volta,
# assinando a pose de um mundo que não existe mais. Achado pelo revisor.
# NUNCA mata a própria árvore de processos (self + ancestrais), senão o script
# que chama isto morre no meio do setup.
PADRAO='ab/probe\.py|ab/colisao\.py|gz sim|gz-transport-topic|gz topic|ruby.*gz|parameter_bridge|ros_gz_bridge|nav2_trekking/lib|robot_nav/lib|nav2_map_server|nav2_amcl|nav2_controller|nav2_planner|nav2_behaviors|nav2_bt_navigator|nav2_waypoint_follower|nav2_velocity_smoother|nav2_collision_monitor|nav2_lifecycle_manager|twist_mux|robot_state_publisher|teleop_twist_joy|teleop_node|joy_node|joy_linux|ros2 launch (nav2_trekking|robot_nav)'

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
