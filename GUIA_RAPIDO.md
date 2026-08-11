# Guia rápido — subir o robô

Para quem nunca mexeu no sistema. Assume que o robô **já está instalado e configurado**
(setup feito). Se for máquina nova, aí sim vá no `README.md`.

---

## 1. Ligar o robô

1. Bateria 12 V conectada, chave ligada.
2. Confira que os dois USB estão na Raspberry Pi: **Arduino MEGA** e **LiDAR**.
3. Espere **~1 minuto** — a Pi boota e entra sozinha no WiFi. Não precisa de monitor
   nem teclado nela.

## 2. Conectar do seu PC

No terminal do **seu PC** (na mesma rede WiFi do robô):

```bash
robot-connect
```

Só isso. Ele entra por SSH no robô e sobe a stack numa sessão tmux.
Sem argumento = modo **teleop** (dirigir na mão, sem mapa).

Outros modos:

```bash
robot-connect slam                        # mapear a sala
robot-connect nav2 --map=maps/sala.yaml   # navegação autônoma (click-to-go)
```

Deu certo quando o terminal para de rolar log e fica vivo mostrando os nós.

## 3. Dirigir

**Controle PS4 (jeito padrão):**
1. Aperte o botão **PS** para ligar/parear.
2. **Segure o L1** e mexa o analógico esquerdo. Soltou o L1, o robô para —
   isso é o dead-man, é de propósito.
3. **R1** = turbo.

**Teclado (alternativa):** noutro terminal do seu PC:

```bash
robot-connect        # (numa aba já aberta)  — ou:
ssh robo@robo-desktop.local
robot-key            # WASD; espaço freia. Para sozinho ~0,6 s após soltar a tecla.
```

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
