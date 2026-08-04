# HANDOFF — instalar o projeto do ZERO num SD de 16 GB (Raspberry Pi do robô 1)

**Data:** 2026-08-04
**Para:** o Claude que estiver no **outro PC**, com o cartão de 16 GB.
**Status:** não começado.

---

## O que o dono quer (escopo, exatamente isso)

> "Quero criar um novo SD card, só que com 16 GB, fazer todos os passos de instalação nele até rodar
> o projeto. Só isso."

Ou seja: **instalação limpa**, do zero, seguindo os passos do `README.md` deste repo, até o
`./launch.sh` subir. O robô 1 perdeu o SD antigo — este é o substituto.

**NÃO é restaurar backup.** Existe uma imagem do SD antigo (2026-07-24, ~29,7 GiB) no PC de
desenvolvimento do Luiz, mas (a) ela não cabe num cartão de 16 GB sem encolher e (b) não é o que ele
pediu. Ela serve só como **consulta**, se faltar algum arquivo de configuração no fim — ver o
apêndice.

---

## Antes de começar — 2 perguntas para o dono

1. **Qual Ubuntu?** A Pi rodava **Ubuntu 24.04 arm64 + ROS 2 Jazzy**. Num cartão de 16 GB o
   **Server** é o recomendado (o Desktop come ~6 GB só de sistema, e a operação é headless por SSH
   de qualquer jeito). Confirmar antes de gravar.
2. **Qual branch vai pra Pi?** A `main` não tem tudo: em 2026-08-04 existem `seguir-pessoa`
   (seguir pessoa v2) e `motion-guard-release-corredor` (vigília-por-movimento + faxina do launch)
   com trabalho validado e não mergeado.

E o aviso honesto: **16 GB é apertado, mas cabe.** Contas na seção "Espaço" no fim — a folga some se
a Pi ficar gravando log e vídeo POV.

---

## Passo 0 — Gravar o sistema no cartão

Raspberry Pi Imager, **Ubuntu Server 24.04 LTS (64-bit)**. Na engrenagem de configuração, **antes**
de gravar, já deixar pronto (evita precisar de monitor/teclado):

- **hostname:** `robo-desktop` — é o que o resto do projeto assume (`ssh robo@robo-desktop.local`);
- **usuário:** `robo` (mesmo nome de antes, os caminhos e o serviço do `face_web` contam com ele);
- **SSH habilitado** com senha ou a chave pública do PC;
- **WiFi** da casa/hotspot configurado.

Boota a Pi, e do outro PC:

```bash
ssh robo@robo-desktop.local          # se não resolver, procurar o IP no roteador
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

> Se não conectar: quase sempre é bateria do robô desligada **ou** o PC noutro WiFi. Não é crash.

## Passo 1 — ROS 2 Jazzy

Guia oficial (~10 min, arm64 tem pacote deb):
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ros2 --help          # tem que listar os comandos
```

## Passo 2 — Clonar o repo

Na Pi o caminho é **`~/workspace/`** (w **minúsculo**) — diferente do `~/Workspace/` do PC dev:

```bash
git clone git@github.com:LGSantarosa/Controle_robo_web.git ~/workspace/Controle_robo_web
cd ~/workspace/Controle_robo_web
git checkout <branch combinada com o dono>
```

O clone é pequeno (~3 MB de objetos, 334 arquivos): os 1,5 GB que aparecem no PC dev são `log/`,
`build/`, `install/` e os `.mkv` de POV — todos no `.gitignore`. **Os mapas vêm no repo**
(`maps/sala.*`, o golden `mapa_golden_2026-06-10.*`, `hotmilk*`), então não se perdem.

Se preferir HTTPS (sem chave SSH configurada na Pi):
`https://github.com/LGSantarosa/Controle_robo_web.git`.

## Passo 3 — `./setup_pi.sh` (o passo longo)

```bash
cd ~/workspace/Controle_robo_web
./setup_pi.sh
```

É a versão enxuta do setup, feita pra Pi. O que ele faz (ler `setup_pi.sh`, é comentado):

- **apt:** `git`, `python3-venv/pip/serial`, `python3-colcon-common-extensions`, `xacro`,
  `robot-state-publisher`, `tf2-ros`, `tf2-tools`, `slam-toolbox`, `nav2-bringup`, `nav2-costmap-2d`,
  `nav2-core`, `nav2-util`, `nav2-map-server`, `nav2-amcl`, `dwb-critics`, `nav-2d-utils`, `joy`,
  `teleop-twist-joy`, `teleop-twist-keyboard`, `twist-mux`. **Não** instala `ros-gz*` (Pi não roda
  Gazebo). Nav2 + slam_toolbox são obrigatórios até pra buildar (`costmap_converter` e
  `teb_local_planner` dependem de `nav2_costmap_2d` em tempo de build).
- **`scripts/setup_headless.sh`:** `openssh-server`, `avahi-daemon` (mDNS `<hostname>.local`),
  `libnss-mdns`, `tmux`, `bluez`, `rfkill`, `joystick`, e os atalhos `robot-up`/`robot-connect`/
  `robot-pair-ps4` no PATH.
- **Clona o driver do LiDAR** `ldlidar_stl_ros2` em `ros2_packages/` e aplica o patch
  `#include <pthread.h>` no `log_module.cpp` — **sem esse patch o build quebra no ARM**.
- **`colcon build`** com `--executor sequential --parallel-workers` ajustado à RAM e `MAKEFLAGS=-j`
  igual — sem isso a Pi 4 de 4 GB estoura RAM e trava.
- `pip install --user platformio` (pro firmware) e `usermod -aG dialout,input`.

**Expectativas honestas:** o próprio script avisa que rodar de microSD deixa o `colcon build` em
**20+ min** e que abaixo de ~1,5 GB de RAM livre ele pode morrer de OOM. Se cair OOM, criar swap
(`sudo fallocate -l 2G /swapfile` → `chmod 600` → `mkswap` → `swapon`) e rodar de novo — **mas isso
come 2 GB dos 16**; melhor apagar o swapfile depois do build.

Depois de `usermod`, **deslogar e logar de novo** (ou `newgrp dialout`) pra valer o grupo.

## Passo 4 — Fixar as portas USB (obrigatório no hardware real)

```bash
sudo ~/workspace/Controle_robo_web/setup_udev.sh
ls -l /dev/mega /dev/lidar     # têm que existir com a MEGA e o LiDAR plugados
```

## Passo 5 — Firmware da Arduino MEGA

Só se a MEGA ainda não estiver flasheada com a versão atual do repo (o firmware fica na MEGA, não no
SD — provavelmente já está certo, mas conferir vale):

```bash
cd ~/workspace/Controle_robo_web/firmware/mega_bridge
pio run -t upload                       # se 'pio' não achar: export PATH="$HOME/.local/bin:$PATH"
python3 tools/test_mega.py --front-only  # testa o protocolo 0xAA 0x55 e o feedback STATE
```

## Passo 6 — Subir o projeto

O venv Python do servidor web (`flask`, `flask-socketio`, `simple-websocket`, `pyyaml`, `numpy`,
`Pillow`) é criado **automaticamente** pelo `launch.sh`, com cache por hash do `requirements.txt`.

```bash
cd ~/workspace/Controle_robo_web
./launch.sh --trekking      # ou --slam / --nav2
```

O `launch.sh` **detecta arm64 sozinho** e usa o perfil leve `nav2_params_pi.yaml` (AMCL 200–800
partículas, `ObstacleLayer` no lugar do `VoxelLayer`, menos amostras DWB). Depois abrir
`http://robo-desktop.local:5000` (ou o IP) no navegador.

A **cara do iPad** (`face_web/face_app.py`) roda como processo **separado** do `launch.sh` — tem um
`face_web/face_web.service` no repo pra instalar como systemd, se o dono quiser igual antes.

---

## Como saber que acabou (critério de pronto)

1. `ssh robo@robo-desktop.local` funciona por mDNS;
2. `ros2 --help` e `ros2 topic list` respondem;
3. `/dev/mega` e `/dev/lidar` existem;
4. `./launch.sh --nav2` sobe sem erro e a UI abre em `:5000`;
5. o mapa (`maps/sala.yaml` ou `hotmilk`) carrega na UI e o LiDAR aparece;
6. teleop move as rodas de verdade.

Reportar cada item com a saída real do comando — nada de "deve estar funcionando".

---

## Espaço em 16 GB — a conta

Cartão "16 GB" costuma ter ~14,8 GiB reais (e **varia por marca** — conferir com `lsblk -b` antes de
prometer qualquer coisa). Estimativa de ocupação:

| Item | Aprox. |
|---|---|
| Ubuntu Server 24.04 arm64 (após upgrade) | 3–4 GB |
| ROS 2 Jazzy + Nav2 + slam_toolbox | 3–4 GB |
| `build/` + `install/` do colcon | ~1 GB |
| Repo + venv Python | ~0,3 GB |
| **Total** | **~8–9 GB** |

Sobra folga, mas ela evapora com: `log/` (no PC dev está com **860 MB** de CSVs) e as gravações
**POV** (`.mkv` de 300–375 MB cada). Recomendação pro dono: **não gravar vídeo POV na Pi com esse
cartão**, e limpar `log/` periodicamente. Se for Ubuntu **Desktop** em vez de Server, somar ~4 GB e
a coisa fica realmente apertada.

---

## Regras da casa (valem durante toda a tarefa)

- **Quem roda comando é o dono**, principalmente `sudo` (pede senha em terminal) e qualquer coisa que
  toque em `/dev/sdX`. O Claude prepara o comando/script pronto; ele executa e manda a saída.
- **Antes de qualquer `dd`/gravação, conferir o device duas vezes** (`lsblk`): device errado apaga o PC.
- **Uma mudança por vez**, e avisar quando a Pi/robô precisa estar **ligado** (flash, udev, teste) vs
  **desligado** (só instalar/compilar).
- **Deploy depois do setup nunca é `scp`:** edita no PC dev → commit → push → na Pi
  `git fetch && git reset --hard origin/<branch>` + `colcon build`.

---

## Apêndice — a imagem de backup (só consulta)

`/home/rbe-luis/backups/pi_robo/pi_robo_2026-07-24.img` (~29,7 GiB) e `.img.zst` (~14 GB), **só no PC
de desenvolvimento do Luiz**. Se no fim faltar alguma config que não está no repo — rede/hotspot com
IP fixo, unidades systemd, ajuste de sistema não documentado — dá pra montar a imagem **read-only**
lá e pescar o arquivo:

```bash
LOOP=$(sudo losetup --find --show --read-only --offset $((1050624*512)) <img>)
sudo mount -o ro "$LOOP" /mnt   # ...copiar o que faltou...
sudo umount /mnt && sudo losetup -d "$LOOP"
```

Gravar a imagem inteira no cartão de 16 GB **não é opção** sem encolher antes (`e2fsck -f` →
`resize2fs` → reduzir partição → truncar), e não é o que o dono pediu.
