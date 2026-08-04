# HANDOFF — montar um SD de 16 GB para a Raspberry Pi do robô 1

**Data:** 2026-08-04
**Para:** o Claude que estiver rodando no **outro PC**, com o cartão de 16 GB plugado lá.
**Status:** tarefa **não começada**. Este arquivo é o briefing.

---

## O que o dono quer

O robô 1 **perdeu o SD card**. Ele quer deixar um **microSD de 16 GB** funcionando na Raspberry Pi
exatamente como o robô era antes: Ubuntu 24.04 arm64 + ROS 2 **Jazzy** + este repositório
(`Controle_robo_web`) + os mapas + o `launch.sh` subindo tudo.

O cartão de 16 GB **estará no outro PC** — não neste. Por isso este briefing está no repo (git), e não
na memória local do Claude do PC de desenvolvimento.

---

## ⚠️ O obstáculo #1 — leia antes de propor qualquer coisa

Existe um **backup bit-a-bit** do SD antigo, feito em **2026-07-24**, mas ele mora **só no PC de
desenvolvimento do Luiz** (usuário `rbe-luis`), em:

```
/home/rbe-luis/backups/pi_robo/pi_robo_2026-07-24.img       ~29,7 GiB (31.902.400.512 bytes, dono root)
/home/rbe-luis/backups/pi_robo/pi_robo_2026-07-24.img.zst   ~14 GB (comprimido)
```

Duas consequências:

1. **A imagem não cabe num cartão de 16 GB por `dd` direto.** São ~29,7 GiB de imagem
   (partições: FAT32 512 M + ext4 29,2 G). Tem que **encolher antes**.
2. **A imagem não está no PC onde o cartão vai estar.** Ou se transfere o `.zst` de ~14 GB (pendrive/rede),
   ou se escolhe um caminho que não precisa dela.

---

## Os três caminhos possíveis

### A) Encolher a imagem e gravar (preserva TUDO — recomendado se o "usado" couber)

Preserva usuário `robo`, serviços, workspace `colcon` já compilado, udev, rede/hotspot, calibrações —
inclusive coisas que ninguém documentou.

Primeiro passo, barato, **antes de mover 14 GB de arquivo**: medir quanto está **realmente ocupado**
dentro da imagem (rodar no PC que tem o `.img`):

```bash
LOOP=$(sudo losetup --find --show --read-only --offset $((1050624*512)) \
       /home/rbe-luis/backups/pi_robo/pi_robo_2026-07-24.img)
sudo dumpe2fs -h "$LOOP" | grep -Ei 'block count|free blocks|block size'
sudo losetup -d "$LOOP"
```

Usado = (Block count − Free blocks) × Block size. Se der abaixo de ~13 GB, cabe folgado num 16 GB
(descontando os 512 M de boot e a margem do resize2fs).

Depois: `PiShrink` ou na mão — `e2fsck -f` → `resize2fs` para o mínimo → reduzir a partição
(`parted`/`sfdisk`) → truncar o `.img` → `dd` no cartão. Um `dd` bruto leva tabela de partição,
bootloader e PARTUUID, então boota direto na Pi.

### B) Instalação limpa (plano B, bem suportado pelo repo)

Se o "usado" não couber em 16 GB, instalar do zero. O `README.md` deste repo tem o caminho conferido:

- ROS 2 Jazzy em Ubuntu 24.04 arm64 (README §1 "Instalar o ROS2 Jazzy");
- `./setup_pi.sh` (README §5 "Raspberry Pi — setup enxuto") — pula Gazebo, instala Nav2 +
  slam_toolbox, clona `wheel_msgs` e o driver do LiDAR com o patch de `pthread.h`, builda com
  `--parallel-workers 2` pra não estourar a RAM;
- `sudo ./setup_udev.sh` — fixa `/dev/mega` e `/dev/lidar` (**obrigatório**);
- os **mapas vêm no próprio repo** (`maps/` — `sala` é o golden, mais o `hotmilk`), então não se perdem;
- o firmware da MEGA também está no repo (`firmware/`), flasheável por USB.

O que a instalação limpa **não** traz de graça e precisa ser refeito/copiado de dentro da imagem
montada: configuração de rede (hotspot + IP fixo), hostname `robo-desktop.local`, usuário `robo`,
qualquer serviço systemd, e o `face_web` (que roda como processo **separado** do `launch.sh`).

### C) Gravar o cartão no PC do backup

Se for possível levar o cartão + leitor USB até o PC de desenvolvimento, esse é o caminho de menor
atrito: a imagem já está lá, e foi assim (cartão FORA da Pi, em leitor USB) que o backup foi feito.
Vale perguntar ao dono antes de organizar transferência de 14 GB.

**Recomendação:** medir o "usado" primeiro (comando acima) → se couber, caminho **A** (ou **C**, se o
cartão puder vir até a imagem). Só cair no **B** se não couber.

---

## Detalhes que vão morder

- **Cartão "16 GB" tem menos de 16 GB reais**, e varia por marca. Conferir com `lsblk -b` **antes** de
  dimensionar o `resize2fs`.
- **Quem roda comando com `sudo` é o dono**, não o Claude: `sudo` aqui exige senha em terminal.
  O fluxo de trabalho combinado é: eu preparo o script/comandos → ele executa → me manda a saída.
  Nada de "roda aí e me diz o que apareceu" no meio de uma gravação de cartão; script pronto,
  com `status=progress`, e checagem do device (`lsblk`) **antes** de qualquer `of=/dev/sdX`.
- **Conferir o device de destino duas vezes.** `dd` no disco errado apaga o PC.
- **Caminho do repo na Pi é `~/workspace/Controle_robo_web`** (w minúsculo), diferente do
  `~/Workspace/...` que aparece no README (esse é o do PC dev).
- **Deploy na Pi nunca é `scp`**: edita no dev → commit → push → na Pi `git fetch && git reset --hard
  origin/main` + `colcon build`.
- ⚠️ **Nem tudo está na `main`.** Em 2026-08-04 existem branches com trabalho validado mas não
  mergeado: `seguir-pessoa` (seguir pessoa v2 goal-based) e `motion-guard-release-corredor`
  (vigília-por-movimento + faxina do launch). Se o cartão for montado do zero e for rodar no robô,
  perguntar ao dono qual branch ele quer na Pi.
- Bootar de microSD é o modo suportado, mas o `setup_pi.sh` avisa que **SSD USB3 é o recomendado** —
  e com 16 GB o espaço vai ficar curto pra logs/gravações. Vale avisar o dono.

---

## Estado de referência (o que "funcionando" significa)

Depois de bootar, o mínimo que precisa estar de pé:

```bash
ssh robo@robo-desktop.local          # se não conectar: bateria do robô OU PC em WiFi diferente
cd ~/workspace/Controle_robo_web
./launch.sh --nav2                   # detecta arm64 e usa nav2_params_pi.yaml
```

E `/dev/mega` + `/dev/lidar` existindo (udev), MEGA flasheada, `maps/sala.*` presentes.
O `face_web` (cara no iPad) sobe **separado** — lembrar de reiniciar ele no deploy.
