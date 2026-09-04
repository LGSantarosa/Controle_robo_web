# Levar a BNO055 pro ROS da Pi — pedido de revisão

**Data:** 2026-09-03 · **Autor:** Claude (sessão de bancada com o robô presente)
**Status:** v4 — revisado 2x (Codex), aprovado com ressalvas, e simulado de
ponta a ponta. Ver §9 (1ª revisão), §10 (simulação) e §11 (2ª revisão).
**Custo estimado:** ~7 s de compilação; ~20 min de execução e validação com
calma, num robô que o dono quer testar hoje.

---

## A decisão

A BNO055 está montada, alimentada e **provada em hardware**. A MEGA publica os
dados dela 50 vezes por segundo. O lado ROS da Raspberry Pi **joga tudo fora
calado**, porque o checkout de lá é de 22/07 e não conhece o frame.

A pergunta: **qual é a menor operação que faz a BNO existir no ROS, sem trocar
o comportamento de navegação de um robô que vai a campo hoje?**

Minha recomendação (v2, pós-revisão): **cherry-pick de 1 commit** — `9390c73` —
na branch atual da Pi, depois de um backup durável e de commitar o que está
sujo lá; mais o patch das correções de hoje, que **não** está em commit nenhum
da `main`.
**Não** um merge da `main`, e **não** cópia de arquivos soltos por `scp` — eu
comecei propondo a cópia e mudei de ideia no meio da apuração; o porquê está
na seção 4.

O que quero que seja atacado: a seção 6.

---

## 1. O que foi medido hoje (não é estimativa)

Firmware novo já gravado na MEGA do robô (`14834 bytes` verificados). Leitura
por sonda serial avulsa, sem ROS:

| medida | valor | significado |
|---|---|---|
| frame `0x85` (FT_IMU2) | **49,2 Hz** | a BNO responde e o firmware a lê |
| `sensor_flags` | **`0x05`** | bit0 `imu_ok` + bit2 `imu2_ok` |
| `/imu/data` (MPU) | **49,2 Hz** | o I²C compartilhado **não** quebrou |
| `|B|` | 18–32 µT | varia com a orientação → offset de ferro-duro, `calib mag=0` |
| `calib gyro` | **3** | o giro da BNO já se auto-calibrou |
| pico `gz1` (MPU, cru) | **+1,268 rad/s** | mesmo giro |
| pico `gz2` (BNO, cru) | **−1,328 rad/s** | módulos a 5% um do outro |
| `corr(gz1·gz2)`, 87 amostras | **−86,4** | sinais crus opostos, consistentemente |
| `yaw_abs` num giro de ~90° real | **−94,0°** | 4% de erro; horário = negativo = convenção ROS |

**Conclusão do teste de sinal: `imu2_yaw_sign = 1.0`, o default.**

Atenção, porque isto é contraintuitivo e a documentação do próprio projeto
errava aqui: a tabela do `ROTEIRO_CAMPO.md` §0.4 dizia "sinais opostos → suba
com `imu2_yaw_sign:=-1.0`". Seguir isso quebraria a BNO. O `pose_estimator`
não compara os valores crus — ele funde `gz1 × imu_yaw_sign` com
`gz2 × imu2_yaw_sign`, e **`imu_yaw_sign` já é −1.0 neste robô** porque a MPU
está de ponta-cabeça (`pose_estimator.py:169`). Logo `+1,268 × (−1) = −1,268`
casa com `gz2 = −1,328` **sem inverter nada**: os crus saírem opostos é o
sintoma da montagem **certa**. Confirmação independente pelo outro caminho: o
`imu2_yaw_sign` também multiplica o heading absoluto (`pose_estimator.py:395`),
e o quaternion já saiu na convenção certa (giro horário → yaw negativo); com
−1.0 a âncora magnética puxaria o robô pro lado oposto.

Já corrigi essa regra nos 4 lugares onde estava escrita errada (tabela do
roteiro, comentário do `pose_estimator`, docstring do `imu2_check.py` e a
mensagem de erro que o nó loga quando as IMUs discordam), e fiz o
`imu2_check.py` imprimir as taxas **já corrigidas** pelos sinais de montagem,
pra a regra "mesmo sinal" voltar a ser verdadeira. **Isso também merece
revisão** — é uma mudança de semântica numa ferramenta de campo.

## 2. A lacuna

```
MEGA  ──frame 0x85 a 50 Hz──>  mega_bridge.py (Pi, 22/07)  ──> /dev/null
```

Verificável:

```bash
ssh robo@robo-desktop.local \
  'cd ~/workspace/Controle_robo_web && grep -c FT_IMU2 ros2_packages/robot_nav/robot_nav/mega_bridge.py'
# 0
```

O dispatch do decodificador é `if/elif` sem `else` (`mega_bridge.py:372-377`),
então frame desconhecido é descartado sem log. Não existe `/imu2/data`, o
`pose_estimator` não funde nada, e a navegação se comporta **exatamente** como
se a BNO não estivesse parafusada.

Isto é o que precisa ser decidido. Nada de físico está faltando.

## 3. Estado do checkout da Pi (a complicação)

- Branch **`seguir-pessoa`**, commit **`860ce20`** (2026-07-22), **45 commits**
  atrás da `main`.
- **13 arquivos modificados e 11 não rastreados**, nenhum commitado. Entre eles
  o **detector de travamento do `trekking_runner`** (escrito no campo de 08-26 e
  que **nunca rodou**), a faxina de órfãos do `launch.sh`, o `robot-key`, os
  fixes de bluez, a auto-detecção de joystick Xbox no `robot.launch.py`,
  `key_teleop.py`, `teleop_xbox.yaml` e os `.bak-*` do campo.
- **Parte dessa sujeira é minha, de hoje**: o `scp` do firmware com suporte à
  BNO (5 modificados + 2 não rastreados em `firmware/mega_bridge/`), feito pra
  gravar a MEGA. A v1 deste documento dizia "8 modificados" — era a contagem de
  **antes** do meu próprio `scp`, e estava desatualizada quando foi escrita.

**Qualquer operação aqui tem que começar commitando isso.** É trabalho de campo
que só existe naquele disco.

## 4. Opções consideradas

| # | opção | veredito |
|---|---|---|
| A | Não fazer nada | A BNO fica decorativa. O dono montou o sensor hoje pra usar hoje. |
| B | `git merge origin/main` | **Não.** Arrasta 45 commits, dos quais **11 mudam comportamento de navegação** (`path_follower`, `motion_guard`, `door_crossing`, `trekking_runner`) que este robô nunca rodou. Trocar a navegação horas antes de um teste é risco sem contrapartida. |
| C | `git checkout main` na Pi | **Não.** A árvore de trabalho volta ao estado da `main` e os fixes de campo somem do disco (ficam no git, mas o robô passa a rodar sem eles). |
| D | Copiar 4 arquivos por `scp` | **Era a minha proposta, e está errada.** Ver abaixo. |
| E | **Cherry-pick de 3 commits** | **Recomendado.** |

### Por que D está errada

A proposta era copiar `mega_bridge.py`, `pose_estimator.py`, `fused_odom.py`
(os três limpos na Pi) e juntar o `robot.launch.py` na mão. O erro está no
último: o `robot.launch.py` da `main` tem **130 linhas** de diferença, e nem
todas são da BNO. Copiá-lo inteiro importaria as outras mudanças de launch
caladas — exatamente o que eu alegava estar evitando. A cópia de arquivos
**não sabe separar** o que é BNO do que não é. O git sabe.

### Por que E é pequena

Três commits tocam os arquivos envolvidos, e **só o primeiro é obrigatório**:

```
9390c73  feat(imu): BNO055 como segunda IMU (9 eixos + heading absoluto)   <- ESTE
beef797  tune(imu): peso da BNO055 na taxa de yaw vai de 0.5 pra 0.8       <- depois
698590a  fix(odom): /trekking/slip dizia "sem derrapagem" sem ter fonte    <- fora
```

A v1 propunha os três, alegando que os dois últimos "vêm junto porque tocam os
mesmos arquivos". **Isso estava errado**: tocar o mesmo arquivo não cria
dependência de cherry-pick, e o `698590a` não tem nada a ver com a BNO. Fica de
fora.

O `beef797` sobe o peso da BNO na taxa de yaw fundida de 0.5 pra 0.8. Numa
estreia, o robô rodar com a IMU nova valendo 80% do giro é uma aposta grande de
primeira. **Estrear com o 0.5 do `9390c73`** e aplicar o `beef797` depois de ver
o comportamento é mais barato de reverter.

Nenhum commit de navegação entra.

## 5. Evidência de compatibilidade

O medo legítimo do cherry-pick é a ilha de versões: `pose_estimator` e
`mega_bridge` novos conversando com `trekking_runner`, `path_follower` e
`cone_detector` de 22/07. Medi a superfície de contrato:

**Tópicos publicados pelo `pose_estimator`: idênticos.**

```bash
git show 860ce20:ros2_packages/robot_nav/robot_nav/pose_estimator.py \
  | grep -o "create_publisher([A-Za-z]*, '[^']*'" | sort > /tmp/a
grep -o "create_publisher([A-Za-z]*, '[^']*'" \
  ros2_packages/robot_nav/robot_nav/pose_estimator.py | sort | diff /tmp/a -
# sem diferença
```

**`mega_bridge`: só ACRESCENTA** `imu2/data`, `imu2/mag`, `imu2/calib`. As
subscrições são **idênticas** nos dois lados (importa: o tipo de
`/wheel_vel_setpoints` não mudou, então o `cmd_vel_to_wheels` velho continua
comandando os motores).

**`/trekking/health`: só ganha chaves** (`heading_anchored`, `heading_corr`,
`mag_calib`, `slip_source`, `wheel`). Nenhuma chave sumiu ou trocou de nome, e
os únicos consumidores são o próprio `pose_estimator` e duas ferramentas de
bancada — **nenhum nó de navegação lê esse tópico**.

**Dependências:** `utils.py` e `cone_pose_fix.py` estão **byte a byte idênticos**
entre `860ce20` e a `main`. `wheel_msgs` já está compilado na Pi. O único
módulo novo nos 45 commits é `sim_trekking_pose.py`, que é de simulação e não é
importado por nada disto.

## 6. Riscos, depois da revisão

O risco que a v1 listava como o mais grave **foi derrubado**: eu dizia que a
guarda de "congela a pose quando as rodas ficam stale" viria com o transplante e
poderia fazer o detector de travamento abortar percursos legítimos. Ela **já
existe** no código que roda na Pi hoje (`fused_odom.py:135-145`, parâmetro
`wheel_fresh`, commit de 22/07). A interação continua existindo e continua não
testada — mas é pré-existente, e não é argumento contra este transplante.

O que sobra, em ordem:

1. **A âncora magnética pode ligar sozinha no meio do teste.** `use_imu2_heading`
   tem default `true`; o portão é `mag >= 2`, avaliado **em tempo de execução**.
   Se a calibração subir durante o movimento, a correção de heading entra sem
   aviso, na primeira vez que este robô a usa. A v1 afirmava que ela "não seria
   ligada" — **afirmação errada**, era só uma observação sobre o estado inicial.
   Mitigação: estrear com `use_imu2_heading:=false` explícito.
2. **Peso 0.8 na estreia** (`beef797`). Resolvido tirando-o do conjunto: estreia
   com 0.5.
3. **Mudança de semântica do `imu2_check.py`.** Passa a imprimir as taxas
   corrigidas em vez das cruas. Fica certo, mas quem decorou os valores antigos
   lê errado. Está no corpo do commit.
4. **Conflitos de merge nos docs.** `9390c73` toca `ESTADO_PROJETO.md`,
   `GUIA_RAPIDO.md` e `README.md`, que divergiram em 45 commits. Conflito
   esperado e sem risco funcional — mas é onde o tempo vai.
5. ~~**Conflito no firmware.**~~ **Previsão errada, derrubada por simulação.**
   Eu previa conflito em `firmware/mega_bridge/` por causa do meu `scp`. Não
   acontece: os arquivos entregues são byte a byte o que o `9390c73` produz,
   então o merge de 3 vias resolve sozinho, e o fix do relê vive em outra região
   do `io_signals.*` e sobrevive. O commit resultante do cherry-pick **nem lista
   os arquivos de firmware**. Ver §10.

## 7. Plano corrigido

O bloqueador que a revisão externa encontrou: **as correções feitas hoje no
`imu2_check.py` não estão em commit nenhum da `main`.** Depois do cherry-pick, a
Pi receberia a ferramenta ANTIGA, que compara os gz crus e mandaria inverter o
`imu2_yaw_sign` — instalando justamente a ferramenta de diagnóstico errada, no
mesmo passo em que se instala o sensor. Resolvido separando o trabalho de hoje
em dois commits:

```
ee4c3c4  fix(relê): módulo é ativo-BAIXO — a luz nascia acesa em todo boot
fa8c4a8  fix(imu2): a regra do teste de sinal dava a resposta invertida
```

O `ee4c3c4` (relê) **já está fisicamente na Pi** desde o `scp` de hoje e entra
pelo commit do passo 2 — não deve ser aplicado de novo. O `fa8c4a8` (regra +
ferramenta) tem que ir por patch, **depois** do cherry-pick, pra pousar em cima
do `imu2_check.py` novo.

```bash
# --- 0. backup DURÁVEL, antes de qualquer coisa (do PC) ---
ssh robo@robo-desktop.local \
  'cp -r /tmp/mega_bridge.bak-2048 ~/backup_firmware_2026-09-03'
#    /tmp não é backup: o systemd limpa no boot. O .hex antigo está lá dentro,
#    e é ele que desfaz a gravação da MEGA num rollback.
scp /tmp/.../0001-fix-imu2-*.patch robo@robo-desktop.local:/tmp/

# --- na Pi ---
cd ~/workspace/Controle_robo_web

# 1. NÃO existe mais nenhum `git checkout` aqui. A v1 tinha um
#    `git checkout -- firmware/` ANTES do commit, que descartava o fix do relê
#    (rastreado) e ainda assim deixava os dois arquivos da BNO (não rastreados)
#    no lugar: nem restaurava, nem salvava. Erro apontado na revisão.

# 2. rede de segurança: branch própria PRIMEIRO, e aí TUDO vira commit
#    A branch isola o estado operacional deste robô. Ela NÃO deve ser mesclada
#    na main depois: os fixes de campo se promovem separados, um a um.
git switch -c pi-bno-2026-09-03
git add -A
git commit -m "wip(campo): estado da Pi antes do transplante da BNO — fixes de 08-26 nunca commitados + firmware com relê ativo-baixo"

# 3. o commit da BNO, sozinho
git fetch origin
git cherry-pick 9390c73
#    Conflitos MEDIDOS por simulação (§10), e como resolver cada um:
#      ESTADO_PROJETO.md — ficar com a versão DA PI e acrescentar só a seção da
#          BNO. NÃO usar --theirs: apagaria o histórico específico deste robô.
#      GUIA_RAPIDO.md   — manter AUSENTE. O arquivo não existe nesta branch;
#          aceitar a versão do commit importaria um guia inteiro de 243 linhas
#          que nunca esteve aqui.  ->  git rm --cached GUIA_RAPIDO.md && rm -f
#    robot.launch.py e firmware/ NÃO conflitam (medido).

# 4. as correções de hoje, por cima da ferramenta nova.
#    O patch é SÓ CÓDIGO (33c4a3d): pose_estimator.py + imu2_check.py. A
#    anotação do ROTEIRO_CAMPO.md ficou num commit separado (530ce6a) que NÃO
#    viaja — o roteiro é de 08-27 e não existe nesta branch. Sem essa separação
#    o `am` abortava inteiro (§10), e o --exclude que resolveria deixaria o
#    commit da Pi diferente do daqui, o que envelhece mal (§11).
git am /tmp/0001-fix-imu2-*.patch

# 5. build (o canônico do repo, ~7 s pro robot_nav)
colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav
source install/setup.bash

# 6. estreia conservadora: âncora magnética DESLIGADA, peso 0.5
ros2 launch robot_nav robot.launch.py use_flow:=false use_imu2_heading:=false
```

Validação, na ordem:

```bash
ros2 topic hz /imu2/data        # ~50 Hz
ros2 topic hz /imu/data         # tem que continuar viva
python3 ros2_packages/robot_nav/tools/imu2_check.py
# girando na mão: os dois gz (JÁ CORRIGIDOS pelos sinais) com o MESMO sinal
```

**Rollback completo** — a v1 dizia só `git reset --hard`, o que é insuficiente,
e a correção proposta na 2ª revisão (gravar de `~/backup_firmware_*`) **não
funciona mais**: aquele backup vivia em `/tmp` e a Pi foi desligada antes de
ele ser copiado pra home. O `/tmp` é limpo no boot. Não faz falta — o firmware
antigo está no git, em `860ce20`:

```bash
# 1. código
git reset --hard <commit do passo 2>
# 2. rebuild, senão o install/ continua com o código novo
colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav
# 3. firmware, SÓ SE quiser desfazer a gravação da MEGA.
#    Num worktree separado, e não com `git checkout 860ce20 -- firmware/`:
#    esse checkout NÃO apaga o sensors_bno055.cpp (que a essa altura é
#    rastreado), e você compila uma mistura. Verificado.
git worktree add /tmp/fw-antigo 860ce20
cd /tmp/fw-antigo/firmware/mega_bridge && pio run -t upload
#    (compila em 0,8 s e sai em 12886 bytes, contra 14834 do firmware de hoje —
#     conferido neste PC)
```

Sobre o passo 3: **provavelmente não vale a pena fazer.** O firmware de hoje é
aditivo e foi medido compatível com o bridge antigo (50 Hz, zero erro de
checksum). E voltar o firmware **também desfaz o fix do relê** — a luz volta a
nascer acesa.

Sem o passo 2 o `install/` continua com o código novo mesmo depois do reset; sem
o passo 3 a MEGA continua com o firmware de hoje (o que, note, não é ruim: ele é
aditivo e compatível com o bridge antigo — foi medido, 50 Hz e zero erro de
checksum).

## 8. O que este plano NÃO faz

- Não mexe em `path_follower`, `motion_guard`, `door_crossing` nem
  `trekking_runner`.
- Não troca a branch da Pi.
- Não descarta nada do trabalho de campo não commitado.
- Não liga a âncora de heading magnético — **mas só porque agora ela é
  desligada explicitamente** (`use_imu2_heading:=false`). O default é `true`, e
  o portão `mag >= 2` é avaliado em tempo de execução: sem a flag, ela entraria
  sozinha se a calibração subisse durante o teste.

---

## 9. Revisão externa (Codex) — o que mudou da v1 pra v2

Aceito integralmente. O que a revisão derrubou:

| achado | veredito |
|---|---|
| A correção do `imu2_check.py` não está nos commits → a Pi receberia a ferramenta antiga, que manda inverter o sinal | **Bloqueador, procede.** Virou o commit `fa8c4a8`, aplicado por patch depois do cherry-pick |
| `698590a` não é necessário; "tocar os mesmos arquivos" não cria dependência | **Procede.** Fora do conjunto |
| `git checkout -- firmware/` antes do backup descarta o fix do relê e ainda deixa os arquivos não rastreados | **Procede.** O `checkout` sumiu do plano |
| São 13 modificados e 11 não rastreados, não 8 | **Procede.** Confirmado; a contagem antiga era anterior ao meu próprio `scp` |
| `/tmp` não é backup durável | **Procede.** Passo 0 copia pra home |
| `git reset --hard` não desfaz a gravação da MEGA nem o `install/` | **Procede.** Rollback reescrito em 3 passos |
| A guarda de congelar pose já existe na Pi; não vem com a BNO | **Procede.** Verificado em `fused_odom.py:135-145` no commit de 22/07. Meu risco nº 1 era falso |
| `use_imu2_heading` é `true` por default e o portão é avaliado em runtime | **Procede.** Estreia agora passa `false` explícito |
| Conflitos reais são em `ESTADO_PROJETO.md`/`GUIA_RAPIDO.md`, não em `robot.launch.py` | **Procede.** A previsão da v1 estava errada |
| Build incremental basta (~7 s), os 20-25 min são margem de validação | **Procede.** Comando canônico do repo adotado |
| Opcionalmente estrear com peso 0.5 e só depois o `beef797` | **Adotado.** `beef797` fora do conjunto inicial |

Único ponto que acrescento à revisão: ela não previu o **conflito no
`firmware/mega_bridge/`** (§6.5), porque a simulação partiu de um estado sem o
`scp` que eu fiz hoje pra gravar a MEGA.

---

## 10. Simulação fiel (o que realmente acontece)

Feita num `git worktree` em `860ce20`, com o `git diff` **atual** da Pi (13
arquivos) aplicado por cima, mais os 2 arquivos não rastreados da BNO copiados
pra dentro, tudo commitado — ou seja, exatamente o estado que o passo 2 do plano
produz. Depois, `cherry-pick 9390c73` e `am` do patch.

**Conflitos reais:**

| arquivo | tipo | resolução |
|---|---|---|
| `ESTADO_PROJETO.md` | conteúdo | manual (doc, sem risco funcional) |
| `GUIA_RAPIDO.md` | modify/delete | trivial: o arquivo **não existe** em `860ce20`, é posterior. Ficar com a versão do commit |

**`robot.launch.py` não conflita** (confirma a revisão externa, derruba a v1).
**`firmware/mega_bridge/` não conflita** (derruba o meu §6.5).

**Bloqueador novo, que nenhuma das duas revisões tinha visto:**

```
$ git am 0001-fix-imu2-*.patch
error: ROTEIRO_CAMPO.md: does not exist in index
Patch failed at 0001
```

O `ROTEIRO_CAMPO.md` foi escrito em 08-27, **depois** do commit em que a Pi
está, e nunca existiu naquele disco. O patch de hoje toca esse arquivo (é onde
os valores medidos foram anotados), então o `am` aborta inteiro — inclusive as
duas correções de código, que são o ponto. Com `--exclude=ROTEIRO_CAMPO.md`
aplica limpo; verificado.

**Estado final da simulação:** `333 testes passam` (`pytest test/ -q` no
`robot_nav`), e o `imu2_check.py` fica com a regra corrigida — imprimindo
`gz1={c1}` com os sinais de montagem aplicados, que é o comportamento que
importa.

---

## 11. Segunda revisão (Codex) — aprovado com ressalvas

Veredito: **transplante conservador aprovado**, condicionado a separar o patch
de código e corrigir o comando de rollback. Peso 0.5 e `use_imu2_heading:=false`
confirmados como certos pra primeira subida.

| ressalva | o que fiz |
|---|---|
| Separar o `fa8c4a8` em commit de código e commit de doc, em vez de usar `--exclude` | Feito: `33c4a3d` (código, é o que viaja) e `530ce6a` (roteiro, fica aqui) |
| `ESTADO_PROJETO.md` não pode ser resolvido com `--theirs` | No plano: ficar com a versão da Pi e acrescentar só a seção da BNO |
| `GUIA_RAPIDO.md` deve seguir ausente, não importar 243 linhas | No plano: `git rm --cached` + `rm` |
| O `wip` deve ir pra branch própria (`pi-bno-2026-09-03`), e não ser mesclado na main depois | No plano, passo 2 |
| O rollback não restaura o firmware antigo daquele diretório | Corrigido, e por outro caminho — ver abaixo |
| 333 vs 337 explicado: `beef797` traz 1 teste, `698590a` traz 3 | Fecha. Nada faltando |

**Onde eu discordo da correção do rollback:** ele propôs gravar de
`~/backup_firmware_2026-09-03`. Esse backup **nunca chegou a existir** — vivia
em `/tmp` e a Pi foi desligada. Mas o próprio comando proposto também deixaria
resíduo: `git checkout 860ce20 -- firmware/mega_bridge` **não apaga** o
`sensors_bno055.cpp`, que a essa altura já é um arquivo rastreado, e o build
sairia misturado. A saída limpa é um `git worktree` em `860ce20` — verificado
compilando aqui (0,8 s, 12886 bytes).

**Simulação final, com todas as resoluções acima aplicadas:** branch criada,
`wip` commitado, cherry-pick resolvido, `git am` do patch de código **sem
`--exclude`**, `333 testes passam`, `imu2_check.py` com a regra corrigida e
`GUIA_RAPIDO.md` ausente. O plano está executável como está escrito.
