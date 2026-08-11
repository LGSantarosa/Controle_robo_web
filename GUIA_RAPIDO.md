# Guia rápido — subir o robô

Para quem nunca mexeu no sistema. Assume que o robô **já está instalado e configurado**
(setup feito). Se for máquina nova, aí sim vá no `README.md`.

---

## 1. Ligar o robô

1. Bateria 12 V conectada, chave ligada.
2. Confira que os dois USB estão na Raspberry Pi: **Arduino MEGA** e **LiDAR**.
3. Espere **~1 minuto** — a Pi boota e entra sozinha no WiFi. Não precisa de monitor
   nem teclado nela.

> Vai usar o robô **noutro lugar**, fora do WiFi de sempre? A rede tem que estar
> cadastrada na Pi **antes** de sair — veja **[6. WiFi](#6-wifi--as-redes-que-a-pi-tenta-no-boot)**.

## 2. Conectar do seu PC

No terminal do **seu PC** (na mesma rede WiFi do robô):

```bash
robot-connect
```

Só isso. Ele entra por SSH no robô e sobe a stack numa sessão tmux.
Sem argumento = modo **teleop** (dirigir na mão, sem mapa).

Outros modos:

```bash
robot-connect slam                          # mapear a sala
robot-connect nav2                          # navegação autônoma (click-to-go)
robot-connect nav2 --map=maps/sala.yaml     # ...com outro mapa
```

Deu certo quando o terminal para de rolar log e fica vivo mostrando os nós.

## 3. Dirigir

**Controle Xbox Series X|S (o do dia a dia desde 2026-08-11):**
1. Ligue no botão **Xbox**. Se já foi pareado, ele reconecta sozinho.
2. **Segure o LB** e mexa o analógico esquerdo. Soltou o LB, o robô para —
   isso é o dead-man, é de propósito.
3. **RB** = turbo.

**Controle PS4:**
1. Aperte o botão **PS** para ligar/parear.
2. **Segure o L1** (mesmo papel do LB) e mexa o analógico esquerdo.
3. **R1** = turbo.

> A stack detecta sozinha qual dos dois está conectado e carrega o mapa de
> botões certo — os números são diferentes (LB=6/RB=7 contra L1=4/R1=5), e é
> por isso que existe um arquivo pra cada. Não precisa passar flag nenhuma.

**Teclado (alternativa):** noutro terminal do seu PC:

```bash
robot-connect        # (numa aba já aberta)  — ou:
ssh robo@robo-desktop.local
robot-key            # WASD; espaço freia. Para sozinho ~0,6 s após soltar a tecla.
```

### Parear o controle de novo (quando ele "some")

Do **seu PC**, com o controle em modo pareamento:

```bash
robot-pair-xbox      # Xbox: ligue no botão Xbox, depois segure PAIR (aresta de
                     #       cima, ao lado do USB-C) 3-5s até piscar RÁPIDO
robot-pair-ps4       # PS4:  PS 10s pra apagar, depois SHARE+PS 5s pra piscar rápido
```

O script cuida de tudo (BlueZ, driver, bonding) e só declara sucesso quando o
`/dev/input/jsN` aparece de verdade. Se o controle nunca foi pareado nesta Pi,
é este mesmo comando.

⚠️ **Xbox e PS4 brigam por uma config do BlueZ.** O Xbox Series é Bluetooth LE
e exige `ControllerMode = dual`; o PS4 pedia `bredr`. Hoje a Pi está em `dual`
(que é o default do BlueZ). Se alguém rodar `pair-ps4.sh`, ele volta pra
`bredr` e **o Xbox para de conectar** — nesse caso, rode `robot-pair-xbox` de
novo, que ele conserta.

## 4. Ver o mapa / clicar destino

Abra no navegador:

```
http://robo-desktop.local:5000
```

- Modo **SLAM**: o mapa vai crescendo enquanto você dirige. Botão **Salvar mapa**
  grava em `maps/`.
- Modo **NAV2**: clique num ponto livre do mapa e o robô vai até lá.
- Modo **teleop**: a página funciona, mas não há mapa nem navegação.

Dirigir **pelo navegador** (WASD na página) não vem ligado por padrão — precisa
subir com `--web-teleop`.

## 5. Sair / desligar

| Quero… | Faço |
|--------|------|
| Sair do terminal mas **deixar o robô rodando** | `Ctrl+B` e depois `D` |
| Voltar para a sessão que ficou rodando | `robot-connect` de novo (reanexa) |
| **Parar tudo** | `Ctrl+C` dentro da sessão |
| Desligar a Pi direito | `ssh robo@robo-desktop.local "sudo poweroff"`, espere 20 s, corte a energia |

## 6. WiFi — as redes que a Pi tenta no boot

A Pi roda **Ubuntu 24.04 com NetworkManager**. No boot ela tenta **todas as redes
salvas**, sozinha: a que estiver ao alcance, ela entra. Não existe "rede principal"
que precise ser ligada primeiro — se duas estiverem ao alcance, ganha a de
**prioridade maior**.

Ou seja: **para levar o robô pra outro lugar, basta cadastrar a rede de lá antes de
sair.** Se não cadastrar, a Pi boota sem rede e não tem como acessá-la sem monitor
e teclado.

### Redes já cadastradas

| Rede (SSID) | Prioridade | Observação |
|-------------|-----------:|------------|
| `Trafico de banana` | 100 | hotspot/rede do Luiz — ganha de todas quando está no ar |
| `Padm3` | 50 | DHCP, WPA2 — cadastrada em 2026-08-11, **ainda não testada no local** |
| `isa` | 50 | DHCP, WPA2 — cadastrada em 2026-08-11, **ainda não testada no local** |
| `Edu Criativa ` | 0 | IP **fixo** (`ipv4.method manual`) — ⚠️ ver avisos abaixo |
| `netplan-eth0` | — | cabo de rede, se alguém plugar |

Conferir a lista a qualquer momento, na Pi:

```bash
nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
nmcli -t -f NAME,DEVICE connection show --active   # em qual ela está agora
```

### Cadastrar uma rede nova (o caso das outras pessoas)

Faça isso **com a Pi ainda acessível** (SSH aqui, ou monitor + teclado nela). Não
precisa estar ao alcance da rede nova — dá pra cadastrar "no escuro":

```bash
ssh robo@robo-desktop.local

sudo nmcli connection add type wifi ifname wlan0 \
  con-name "NOME_DA_REDE" ssid "NOME_DA_REDE" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "SENHA_DA_REDE" \
  connection.autoconnect yes \
  connection.autoconnect-priority 50 \
  ipv4.method auto
```

Troque `NOME_DA_REDE` e `SENHA_DA_REDE`. O SSID tem que ser **idêntico** ao da rede
(maiúsculas, acentos e espaços contam).

Variações:

```bash
# Rede oculta (não aparece na lista de redes):
sudo nmcli connection modify "NOME_DA_REDE" 802-11-wireless.hidden yes

# Rede ao alcance agora, jeito curto:
sudo nmcli device wifi connect "NOME_DA_REDE" password "SENHA"

# Mudar prioridade / apagar:
sudo nmcli connection modify "NOME_DA_REDE" connection.autoconnect-priority 60
sudo nmcli connection delete "NOME_DA_REDE"
```

**Prioridade — como escolher:** número maior ganha quando mais de uma rede está ao
alcance. Redes de outros lugares podem ficar em `50` sem problema: elas só perdem
para o `Trafico de banana` (100), que não existe lá longe mesmo.

### Rede de resgate (recomendado)

Cadastre também o **hotspot do celular** de quem vai levar o robô, com prioridade
baixa (ex.: `10`). Se a rede do local falhar, essa pessoa liga o hotspot do celular
e a Pi entra nele sozinha — sem isso, robô sem rede = precisa de monitor e teclado.

### Avisos

- 🔑 **Não coloque as senhas neste arquivo.** Ele vai pro GitHub. A senha entra no
  comando `nmcli`, uma vez, direto na Pi.
- ⚠️ **`Edu Criativa ` tem um espaço no fim do SSID.** Se o nome real da rede for
  `Edu Criativa` (sem espaço), esse perfil **nunca vai conectar**. Confira no local
  e, se for o caso, corrija:
  ```bash
  sudo nmcli connection modify "Edu Criativa " 802-11-wireless.ssid "Edu Criativa"
  sudo nmcli connection modify "Edu Criativa " connection.id "Edu Criativa"
  ```
- ⚠️ Esse mesmo perfil usa **IP fixo**. IP fixo só funciona se o roteador de lá
  usar exatamente aquela faixa. Em rede desconhecida, prefira
  `sudo nmcli connection modify "<rede>" ipv4.method auto`.
- Cadastrou e quer testar antes de viajar? Ligue o hotspot com o mesmo nome e senha
  da rede de destino e reinicie a Pi — se ela entrar, o perfil está certo.

### Achar o robô na rede nova

`robo-desktop.local` funciona em qualquer rede (mDNS). Se não resolver:

```bash
ROBOT_HOST=<ip-do-robô> robot-connect      # IP no painel do roteador do local
```

E lembre: quem for falar ROS com o robô precisa de `export ROS_DOMAIN_ID=42`.

---

## Dicas de mapeamento (modo SLAM)

1. Comece parado no centro da área.
2. Dirija **devagar** — o SLAM precisa casar scans consecutivos.
3. Retas longas; evite girar parado.
4. **Feche loops**: volte por onde já passou.

---

## Se der errado

| Sintoma | O que fazer |
|---------|-------------|
| `robo-desktop.local` não resolve | Pegue o IP no roteador: `ROBOT_HOST=192.168.0.50 robot-connect` |
| Pede senha / dá erro de chave | `ssh-copy-id robo@robo-desktop.local` (uma vez só) |
| Usuário diferente no robô | `ROBOT_USER=pi robot-connect` |
| **Nenhum tópico ROS aparece no seu PC** (RViz vazio, `ros2 topic list` vazio) | Falta `export ROS_DOMAIN_ID=42` no seu PC. Há **outro robô ROS2** nesta rede no domain 0 — sem isso você não vê o nosso, ou pior, vê o dele. Ponha no `~/.bashrc`. |
| Robô não anda | Confira `/dev/mega`: `ls -l /dev/mega` no robô. Sumiu → replugue o USB da MEGA. |
| Sem `/scan` (LiDAR) | `ls -l /dev/lidar`. Cheque a fiação do LD06 (vermelho→5V, preto→GND, amarelo→RXD, **PWM→3,3 V**). |
| `robot-connect` diz "sessão viva mas launch.sh não está rodando" | No robô: `tmux kill-session -t robo` e rode `robot-connect` de novo. |

---

## Avisos

- **Modo teleop não tem camada de segurança** — nada de collision monitor. Você é
  o responsável por não bater.
- O robô é pesado e as rodas são de hoverboard: comece com o multiplicador baixo.
- ⚠️ O `launch.sh` deste checkout tem correções **não commitadas**. Não rode
  `git checkout launch.sh` — você perde o fix que impede a faxina do script de
  matar a própria sessão tmux.

Detalhes de tudo: `README.md`. Fiação: `CONEXOES.txt`.
