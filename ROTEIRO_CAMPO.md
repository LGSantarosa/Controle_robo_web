# Roteiro de campo — estreia da BNO055 + validação de movimento no robô real

> **Data de uso:** 2026-08-26 (primeira aplicação).
> **Contexto:** a BNO055 nunca foi montada e todo o tuning de trekking de 08-25
> foi feito no simulador. Este roteiro sobe do mais simples pro mais complexo e
> **só deixa avançar quando a fase anterior passou.**
>
> Cada fase tem **critério de passagem**. Se falhar, a coluna "se falhar" diz o
> que fazer — na maioria dos casos é desligar aquele pedaço e seguir o dia, não
> parar tudo.
>
> Preencha as tabelas na hora. Elas viram a entrada do `ESTADO_PROJETO.md` depois.

---

## Por que esta ordem (leia uma vez)

1. **A `/odom` virou a régua, e a BNO055 virou 80% da régua.** O
   `imu2_rate_weight` está em `0.8` — a IMU nova responde por 80% da taxa de
   yaw da pose fundida. O `spin_calib` e o `arc_calib` medem *pela `/odom`*.
   Medir giro com a régua torta é caçar bug no lugar errado o dia inteiro.
   **Por isso a Fase 0 (bancada) vem antes de qualquer movimento.**

2. **Uma mudança por vez.** Estreia de sensor + perfil de velocidade novo
   juntos não mede nenhum dos dois. Isso já custou dois diagnósticos errados em
   08-25 (o raio do snap e o giro 4,5). Fase 1 roda com a IMU **desligada** de
   propósito: é a linha de base conhecida.

3. **Uma verdade que não passe pela `/odom`.** Nas Fases 1-3, meça **com trena
   e giz** pelo menos uma repetição. A `/odom` do sim mentiu 48 cm e ninguém
   viu por meses porque não havia régua externa.


**4. A pilha de hoje é roda + 2 IMUs + LiDAR. Sem flow.**
O PMW3901 foi arrancado do robô em 2026-07-01 (commit `33647e4`). O `use_flow`
ficou com default `true` por quase dois meses depois disso — **corrigido em
2026-08-26**, agora nasce `false` no `robot.launch.py` e no `pose_estimator`. O
código do flow continua lá: `use_flow:=true` reativa tudo no dia que o sensor
voltar pro chassi.

Isso nunca deu conta errada — sem dados, `flow_age = inf` → `α = 0` → a
translação já caía pra 100% roda sozinha. O que dava eram dois warnings por
minuto (`flow stale` e `alpha=0.000`) sobre um sensor ausente.

As linhas de launch deste roteiro seguem com `use_flow:=false` explícito de
propósito: **o checkout da Pi está divergente**, e se ela não pegar o commit de
hoje o default velho ainda vale lá. Explícito funciona nos dois casos.


---

## Pré-voo

### Antes de sair de casa

- [ ] **`git status` na Pi** — o checkout dela está divergente (branch
      `seguir-pessoa` + arquivos entregues por `scp` + correções do fluxo
      headless aplicadas lá e **nunca commitadas**). Um `git pull` ou `reset`
      descuidado perde os fixes.
      ```bash
      ssh robo@robo-desktop.local
      cd ~/workspace/Controle_robo_web && git status && git stash list && git branch -vv
      ```
      Guarde o que estiver sujo (`git stash push -m "fixes headless da Pi"`)
      **antes** de trazer a `main` nova. O firmware da BNO055 só existe na
      `main` — sem esse pull, não há o que flashear.

- [ ] **Rede do local cadastrada na Pi** (prioridade 50, DHCP). Se a rede de lá
      não estiver cadastrada, você chega e não conecta. Ver `GUIA_RAPIDO.md`.

- [ ] **Na mochila:** trena, giz ou fita crepe, a BNO055, fios/barra de bornes,
      multímetro se tiver.

### Ao chegar

- [ ] No **seu PC**: `export ROS_DOMAIN_ID=42` — há **outro robô ROS2** nessas
      redes publicando no domain 0. Sem isso você vê o `/tf` e o `base_link`
      dele e acha que é o nosso.
- [ ] Bateria 12 V, chave ligada, os dois USB na Pi (MEGA e LiDAR).
- [ ] `robot-connect` conecta. Se `robo-desktop.local` não resolver, pegue o IP
      no roteador: `ROBOT_HOST=192.168.x.x robot-connect`.

---

# FASE 0 — Bancada: a IMU nova
### 🔧 Robô **escorado, rodas NO AR**. Nada roda no chão nesta fase.

### 0.1 — Montagem física

Os 4 fios da BNO055 vão para o **mesmo sinal** que a MPU já usa. I²C é
barramento: dois sensores convivem no mesmo par SDA/SCL porque têm endereços
diferentes (MPU = 0x68, BNO055 = 0x28). **Não existe "segundo I²C" pra ligar
aqui.** Detalhes em `CONEXOES.txt:19`.

| BNO055 | no mesmo sinal que | observação |
|---|---|---|
| VIN | 5 V (ou 3V3) | **ver o aviso abaixo** — 5 V só se o breakout tiver regulador |
| GND | GND da MEGA | o **mesmo** GND, obrigatório |
| SDA | pino 20 (SDA) | já ocupado pelo dupont da MPU — **derive**, ver abaixo |
| SCL | pino 21 (SCL) | já ocupado pelo dupont da MPU — **derive**, ver abaixo |
| ADR, PS0, PS1, RST, INT | — | deixe **soltos**. Solto/GND = 0x28, o normal |

### Como derivar (a parte prática — não cabe segundo dupont no pino)

Os pinos 20/21 da MEGA já estão com o fio da IMU #1, e **não entra um segundo
conector no mesmo pino macho**. Você precisa ramificar o barramento em algum
ponto. Escolha um:

- **(a) barra de bornes ou protoboard — recomendado.** Leve 5V, GND, SDA e SCL
  da MEGA para uma barrinha, e saia dela para os dois sensores. É o mais limpo
  e o mais fácil de desfazer se precisar mexer em campo.
- **(b) emendar.** Solde os dois fios de SDA num terminal só (idem SCL, 5V e
  GND). Funciona, mas você fica sem como separar os sensores depois.
- **(c) cabo Y / furos duplicados.** Alguns breakouts trazem furos de passagem
  que servem pra encadear sensor → sensor; splitters dupont em Y fazem o mesmo.

```
MEGA (pinos 20/21) ──┬── SDA/SCL ──> IMU #1 (MPU, 0x68)
                     └── SDA/SCL ──> IMU #2 (BNO055, 0x28)
```

**Não precisa de resistor de pull-up** — cada breakout já traz os seus, e dois
pares em paralelo no barramento é normal.

- [ ] **⚠️ OLHE A PLACA ANTES DE ENERGIZAR.** Tem um CI regulador e um shifter
      perto do conector? É 5 V-safe. Só tem o quadradinho do BNO055 e mais
      nada? **5 V queima o sensor** — alimente pelo 3V3 e ponha conversor de
      nível nos SDA/SCL (os pull-ups da MEGA levam essas linhas a 5 V).
- [ ] Chip **plano** (deitado) e **preso firme**. Se balançar, o heading balança.
- [ ] **O mais longe possível** dos motores das rodas e dos cabos grossos de
      12 V. É magnetômetro: ímã de motor puxa o "norte" dele. **Alto e no
      centro** é melhor que baixo e perto da roda. Longe de ferro/aço e
      alto-falante.
- [ ] Não precisa ficar no centro geométrico — giro e orientação não dependem
      de onde o sensor está no corpo rígido.

### 0.2 — Firmware novo na MEGA

O `mega_bridge` velho segura a `/dev/mega`. Derrube a stack antes:

```bash
tmux kill-session -t robo 2>/dev/null            # se estiver de pé
cd ~/workspace/Controle_robo_web/firmware/mega_bridge
pio run -t upload
```

> O `launch.sh` também flasheia sozinho quando o hash do firmware muda
> (`--flash-mega` força), mas ele sobe a stack inteira junto. Nesta fase é
> melhor o upload isolado.

- [ ] Compilou e subiu sem erro.

### 0.3 — O frame está chegando

```bash
cd ~/workspace/Controle_robo_web && source install/setup.bash
ros2 launch robot_nav robot.launch.py use_flow:=false
```
Noutra aba, **as duas** — a nova e a que já existia:
```bash
ros2 topic hz /imu2/data     # BNO055 — ~50 Hz
ros2 topic hz /imu/data      # MPU    — tem que continuar viva
```

> **Este é o teste do barramento compartilhado.** As duas IMUs dividem o mesmo
> par SDA/SCL (endereços 0x68 e 0x28) — junto com o AK8963 em 0x0C, que é o
> magnetômetro dentro da própria MPU e já divide esse barramento desde sempre.
> Se compartilhar quebrasse alguma coisa, **as duas caem juntas** e você vê na
> hora. As duas publicando = barramento saudável, provado no hardware.

| resultado | leitura |
|---|---|
| **as duas a ~50 Hz** | ✅ passou, siga |
| `/imu2/data` mudo, `/imu/data` viva | a MEGA não vê a BNO055. Confira GND comum, endereço (0x28/0x29 — o firmware tenta os dois) e se o VIN tem tensão |
| **as duas caíram** | aí sim é o barramento: fio trocado, curto mecânico na emenda, ou alimentação afundando. Desligue a BNO055 e confirme que a MPU volta |
| muito abaixo de 50 Hz | fio ruim / contato intermitente na derivação |

- [x] `/imu2/data` a ~50 Hz. **Se não passar, pule pra "Plano B" no fim e rode o dia sem a IMU nova.**
      2026-09-03: medido **49,2 Hz** na serial (frame `0x85`), com a MPU a 49,2 Hz
      no mesmo barramento. `sensor_flags = 0x05` → `imu2_ok=True`. O tópico
      `/imu2/data` em si ainda NÃO existe: o `mega_bridge.py` da Pi é de 22/07 e
      não decodifica o frame — a validação saiu por sonda serial avulsa.

### 0.4 — SINAL 🔴 (o mais importante do dia)

```bash
python3 ros2_packages/robot_nav/tools/imu2_check.py
```

**Gire o robô PRA ESQUERDA** (na mão, rodas no ar). Olhe `gz1` e `gz2`:

> ⚠️ Os `gz` que o `imu2_check.py` imprime já vêm **corrigidos** pelos sinais de
> montagem (`gz1*imu_yaw_sign`, `gz2*imu2_yaw_sign`) — é sobre eles que a tabela
> vale. **Não use os valores crus** de `ros2 topic echo /imu/data` e
> `/imu2/data`: o MPU deste robô está de ponta-cabeça (`imu_yaw_sign=-1.0`),
> então os crus saem **opostos justamente quando as duas estão certas**.

| resultado | ação |
|---|---|
| **mesmo sinal** | ✅ passou |
| **sinais opostos** | a BNO055 está montada girada. Suba com `imu2_yaw_sign:=-1.0` (não precisa reflashear) e repita |

**Medido no robô em 2026-09-03** (giro único de ~90° na mão, 87 amostras acima
do limiar): pico `gz1 = +1.268` contra `gz2 = -1.328` rad/s — crus opostos,
módulos a 5% um do outro, ou seja **as duas mediram o mesmo giro**. Corrigidos,
concordam. Sinal usado: **`imu2_yaw_sign = 1.0` (o default — nada a passar)**.

> Enquanto discordarem, o `pose_estimator` **ignora** a BNO055 e loga erro —
> isso é proposital, não é bug. Com o sinal trocado a média das duas daria
> **zero**: o robô giraria no chão sem girar no mapa.

```bash
# se precisar do sinal invertido, derrube e suba assim:
ros2 launch robot_nav robot.launch.py imu2_yaw_sign:=-1.0 use_flow:=false
```

- [x] `gz1` e `gz2` com o mesmo sinal.  Sinal usado: `imu2_yaw_sign = 1.0 (default)`

### 0.5 — MAGNITUDE

Gire o robô **90° reais** (marque no chão / use um canto de mesa como referência).

| `yaw_abs` andou | leitura |
|---|---|
| **~90°**, no mesmo sentido do `yaw_odom` | ✅ passou |
| ~45° ou ~180° | **a montagem não está plana.** Volte e endireite o chip |

- [x] 90° reais → `yaw_abs` andou `-94.0°` (giro pra DIREITA; horário = negativo,
      que é a convenção certa do ROS). 4% de erro contra um 90° marcado a olho.

### 0.6 — CALIBRAÇÃO do magnetômetro

Mova o robô em **∞ (oito) NO AR**, uns 20 s, até `mag` chegar em **3**.

- `|B|` deve ficar entre **~25 e 65 µT** (campo da Terra). Muito acima ou
  instável = ferro perto do sensor ou EMI.
- Enquanto `mag < 2` o heading absoluto **não é usado** — é o esperado, o gate
  de calibração está funcionando.

- [ ] `mag` chegou a 3.  `|B|` = `________ µT`

### 0.7 — EMI: o teste que decide o heading

**Ligue os motores** (ainda com as rodas no ar — dirija um pouco no controle) e
olhe o `mag`.

| resultado | ação |
|---|---|
| `mag` **se segura** em 2-3 | ✅ o heading absoluto vale aqui |
| `mag` **despenca** pra <2 com motor ligado | **EMI deste ambiente.** Rode com `use_imu2_heading:=false` — mantém a 2ª taxa de giro, desliga só a âncora magnética |

- [ ] `mag` com motores ligados: `________`  → `use_imu2_heading` = `________`

---

### ✅ Critério de passagem da Fase 0

**Só siga pro chão se 0.3, 0.4 e 0.5 passaram.** 0.6 e 0.7 podem falhar sem
bloquear — só definem se o heading entra ou não.

**Anote a linha de comando que vai usar o resto do dia:**

```
ros2 launch robot_nav robot.launch.py use_flow:=false \
    imu2_yaw_sign:=____  use_imu2_heading:=____
```

---

# FASE 1 — A reta, com a IMU nova DESLIGADA
### 🛞 Robô no chão. ~4 m livres à frente. **Controle DESLIGADO.**

> Por que desligada: `use_imu2:=false` devolve a pose **exatamente** ao que era
> antes (tem teste unitário garantindo). Esta é a linha de base conhecida. Sem
> ela, qualquer coisa estranha na Fase 2 não tem com o que ser comparada.

> **Controle desligado é obrigatório:** `joy_vel` tem prioridade 100 e sobrepõe
> o `key_vel` que os scripts usam. Com o controle ligado o script parece não
> funcionar.

```bash
ros2 launch robot_nav robot.launch.py use_imu2:=false use_flow:=false
```
Noutra aba, uma reta pura de 2 m (`--wz 0` = sem giro comandado):
```bash
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0 --vx 0.5 --duration 4
```

**Marque com giz** o ponto de partida e a linha de mira. Ao fim, meça com trena.

| rep | `chord` (odom) | distância real (trena) | desvio lateral no fim (trena) | `girou` (°) |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |

**O que olhar:** `girou` deveria ser ~0°. Se ele puxa consistentemente pro mesmo
lado, é assimetria física (roda/motor), não sensor — e vai contaminar tudo
depois.

- [ ] 3 repetições feitas e anotadas.

---

# FASE 2 — A mesma reta, com a IMU nova LIGADA

Exatamente o mesmo comando, mesma marcação de giz, trocando só o launch:

```bash
ros2 launch robot_nav robot.launch.py imu2_yaw_sign:=____ use_imu2_heading:=____ use_flow:=false
```
```bash
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0 --vx 0.5 --duration 4
```

| rep | `chord` (odom) | distância real (trena) | desvio lateral (trena) | `girou` (°) |
|---|---|---|---|---|
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |

### A leitura desta fase

| o que aconteceu | veredito |
|---|---|
| trena igual à Fase 1, `girou` igual ou **menor** | ✅ a IMU nova entrou limpa |
| `girou` (odom) mudou mas a **trena não** | a IMU mudou a **crença**, não o robô. Quem está mais perto da trena está certo — provavelmente a nova |
| a **trena** piorou | o robô está andando diferente. Algo está realimentando: pare e investigue antes de seguir |
| discrepância grande e inexplicada | volte pro `imu2_check.py` e reconfira 0.4 e 0.5 |

- [ ] Decisão: rodar o resto do dia com `use_imu2` = `________`

---

# FASE 3 — O giro no lugar

```bash
python3 ros2_packages/robot_nav/scripts/spin_calib.py --speeds 3
```

Mede **esquerda × direita** na mesma velocidade e mostra o rad/s efetivo.

**O que já sabemos e queremos confirmar:** a deriva assimétrica conhecida é
**+13,4 cm à direita × −3,1 cm à esquerda**, e foi medida a **24°/s**. Hoje o
trekking gira a **83°/s** — e a odometria **não enxerga** essa deriva.

| velocidade | girou ESQ (°) | girou DIR (°) | rad/s efetivo ESQ | rad/s efetivo DIR | deslocamento do centro (trena) |
|---|---|---|---|---|---|
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 6 |  |  |  |  |  |

Rode a varredura completa quando o 3 passar:
```bash
python3 ros2_packages/robot_nav/scripts/spin_calib.py --speeds 6,4,3,2
```

**Meça o deslocamento do centro com trena.** Marque o ponto do centro do robô
antes, gire, marque depois. É a deriva que a odometria não vê — o número que
mais importa aqui, e o único que a `/odom` não te dá.

- [ ] Varredura feita. Assimetria esq/dir: `________`

---

# FASE 4 — Reta + giro juntos (o quadrante que falta) ⭐

> **Esta é a fase que mais destrava coisa.** A conclusão "o robô não arqueia"
> veio toda de `vx=0,25`, onde **todo** arco testado tinha raio ≤ 0,83 m num
> robô de 0,5 m — nunca testou curva, só pirueta. O quadrante útil
> (**vx 0,8-1,2 com wz 0,2-0,6 = raio 2-5 m**) está **vazio**. Você afirma que
> ele arqueia com velocidade. Se arquear, os ~20% do tempo que o trekking ainda
> gasta parado viram **zero**.

**Um arco por execução.** Precisa de espaço: o arco anda pra frente. Posicione,
rode, ele para, traga de volta, rode o próximo.

```bash
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0.3 --vx 1.0
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0.5 --vx 1.0
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0.2 --vx 0.8
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz 0.6 --vx 1.2
# wz NEGATIVO = arco pra DIREITA — repita os mesmos pra checar simetria
python3 ros2_packages/robot_nav/scripts/arc_calib.py --wz -0.3 --vx 1.0
```

O script já imprime o veredito e anexa uma linha em `/tmp/arc_calib.csv`:

| `ratio` | significado |
|---|---|
| **> 70%** | ✅ **ELE ARQUEIA** nessa wz — vale reescrever o trekking pra usar arco |
| 25-70% | arco parcial: vira menos do que pede |
| **< 25%** | não arqueia nessa wz |

| vx | wz | ratio | raio comandado | raio efetivo |
|---|---|---|---|---|
| 1.0 | +0.3 |  |  |  |
| 1.0 | +0.5 |  |  |  |
| 0.8 | +0.2 |  |  |  |
| 1.2 | +0.6 |  |  |  |
| 1.0 | −0.3 |  |  |  |

- [ ] Puxe o `/tmp/arc_calib.csv` no fim do dia: `scp robo@robo-desktop.local:/tmp/arc_calib.csv .`

---

# FASE 5 — Trekking curto

Só aqui o `launch.sh` entra (precisa do `cone_detector` + `trekking_runner` +
web, que o `robot.launch.py` sozinho não sobe):

```bash
./launch.sh --trekking
```

> ⚠️ **Aqui está o furo conhecido:** o `launch.sh:485` chama o
> `robot.launch.py` **sem repassar argumento nenhum**. Se você precisou de
> `imu2_yaw_sign:=-1.0` ou `use_imu2_heading:=false` nas fases anteriores,
> **esses valores não chegam** nesta fase. Ou você edita o `robot.launch.py` na
> mão na Pi, ou pula a Fase 5 hoje. (Tem um patch de ~15 linhas pendente pra
> resolver isso — peça antes de sair, se quiser a Fase 5.)

1. Grave uma rota **curta**: uma reta e um canto, 2 waypoints, sem cone.
2. Dê Play e assista.

> ⚠️ **Corrigido em 2026-08-27 — a instrução anterior estava invertida pro
> robô real, e apontava o arquivo errado.**
>
> **Onde `rot_min` mora:** só no `DriveConfig` do `trekking_runner.py`
> (default 4,0). **Não** está no `trekking.launch.py` e não tem argumento de
> launch. Os params são lidos **uma vez no init**, sem callback — `ros2 param
> set /trekking_runner rot_min ...` com o nó no ar **não faz nada**. Pra mudar
> em campo: subir o nó com `-p rot_min:=X` ou editar o `DriveConfig`.
>
> **Confira em qual checkout você está antes de mexer.** A `main` tem 4,0; o
> checkout da Pi (branch `seguir-pessoa`, congelado em 06-12) tem **2,4** — e
> naquela versão o `_control_tick` nem usa piso de giro.

**Se aparecer giro extra ou traçado torto, `rot_min` NÃO é o primeiro knob a
baixar.** Medido no robô real em 2026-08-26 (Fase 3): a zona-morta do skid é
**~1,7 rad/s** e o comando 3,0 entrega só 0,43 rad/s efetivos (24°/s). Baixar o
piso joga a correção **dentro** da zona-morta: o controlador pede giro e a roda
não sai do lugar — o robô segue reto achando que está corrigindo. Foi assim que
ele empurrou o cone por 100 s com `wz=-0,476` congelado.

O sim mostrou um **penhasco entre 4,0 e 4,5** (4,5 dá 35 cm de desvio e um giro
extra), então 4,5 pra cima é o que se evita. Entre 4,0 e o piso útil real a
margem é estreita: **desça no máximo até 3,4 (61°/s) e pare aí.** Abaixo disso,
suspeite de outra coisa antes — `turn_enter` (20°) alto demais pro canto,
`arrival_tolerance` (25 cm) apertada, ou odometria escorregando sem cone pra
ancorar.

| tentativa | `rot_min` | nº de giros | traçado | tempo |
|---|---|---|---|---|
| 1 | 4.0 |  |  |  |
| 2 |  |  |  |  |

---

## Plano B — se a BNO055 não colaborar

Nada disso trava o dia. O sistema roda exatamente como antes:

```bash
ros2 launch robot_nav robot.launch.py use_imu2:=false use_flow:=false     # corta os dois caminhos
ros2 launch robot_nav robot.launch.py use_imu2_heading:=false use_flow:=false   # corta só a âncora magnética
```

`use_imu2:=false` volta a pose a ser **exatamente** a de antes (tem teste). As
Fases 3, 4 e 5 valem por si — o `arc_calib` no quadrante vazio é útil com ou sem
IMU nova.

---

## Regras de segurança / bom senso

- **Controle DESLIGADO** durante `spin_calib` e `arc_calib` (prio 100 sobrepõe).
- **Ctrl-C para o robô** nos dois scripts — é tratado, não é kill seco.
- **Dead-man no teleop:** segure LB (Xbox) ou L1 (PS4). Soltou, para.
- **Antes de tirar conclusão de uma corrida, confira se ela foi válida.**
  Reaproveitar stack sem reset já invalidou uma medição inteira em 08-25 (o robô
  começou de onde a anterior parou e saiu 6,7 m fora da rota).
- **Sintoma de movimento estranho → primeira coisa é `ps` pelos seus próprios
  processos.** Um script de validação esquecido rodando já foi confundido com
  bug do robô.
- **Uma mudança por vez.** A/B de duas coisas juntas não mede nenhuma.

---

## No fim do dia

- [ ] `scp robo@robo-desktop.local:/tmp/arc_calib.csv .`
- [ ] `scp robo@robo-desktop.local:/tmp/spin_calib*.csv .`
- [ ] Fotografe as tabelas preenchidas.
- [ ] Anote **o que te surpreendeu** — é o que vale mais que os números.
