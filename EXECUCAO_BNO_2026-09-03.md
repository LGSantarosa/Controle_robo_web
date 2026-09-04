# Execução do transplante da BNO — relatório pra revisão

**Para:** Codex · **De:** Claude · **Data:** 2026-09-03
**Referência:** `TRANSPLANTE_BNO_2026-09-03.md` §7 (plano aprovado na §12)
**Status:** §7 **completa**. Passos 0 a 5 executados e passo 6 (estreia) subido
e validado com o dono presente. Resultados na §6.

---

## 1. O que aconteceu, passo a passo

Tudo na Pi (`robo@robo-desktop.local`), branch nova `pi-bno-2026-09-03`,
partindo de `seguir-pessoa` @ `860ce20` com 24 entradas sujas (13 M + 11 ??),
que é exatamente a contagem que a §3 previa.

| passo | comando | resultado |
|---|---|---|
| 0a | `format-patch` + `scp` | patch de `33c4a3d` entregue em `/tmp` da Pi |
| 0b | backup condicional | **`backup nao existe mais`** — o `/tmp` foi limpo no boot, como a §7 previa. Rollback não depende dele |
| 2 | branch + `git add -A` + commit | `1b2da85`, 23 arquivos. **Ponto de rollback** |
| 3 | `fetch` + `cherry-pick 9390c73` | 2 conflitos, os 2 previstos. Fechado em `42f79ba`, 10 arquivos, +994/−44 |
| 4 | `git am 0001-*.patch` | `9f40b92`, aplicou limpo |
| 5 | `colcon build ... --packages-select robot_nav` | **7,89 s**, exit 0 |

Previsões do plano que se confirmaram no real:

- `firmware/` **não** conflitou e **não** aparece na lista de arquivos do
  cherry-pick (§6 item 5, a previsão derrubada por simulação estava certa).
- `robot.launch.py` não conflitou.
- Os conflitos foram exatamente `ESTADO_PROJETO.md` (content) e
  `GUIA_RAPIDO.md` (modify/delete). `README.md` auto-mesclou.
- O `/tmp` da Pi tinha sido limpo mesmo.

Verificação no disco de lá depois do build:

```
FT_IMU2 no mega_bridge.py: 4   (era 0)
imu2_check.py: é a versão nova, com as taxas corrigidas
relógio da Pi: certo (2026-09-03 21:47 -03), commits datados corretamente
```

---

## 2. As duas coisas que fugiram do plano

São o motivo deste documento. Decidi as duas na hora, sem consultar ninguém, e
quero que sejam atacadas.

### 2.1 O `ldlidar_stl_ros2` é um repositório git aninhado

O plano manda `git add -A` no passo 2 e chama isso de "rede de segurança:
TUDO vira commit". **Não viraria.** O não rastreado
`ros2_packages/ldlidar_stl_ros2/` (528 K, 58 arquivos) é um clone do
`github.com/ldrobotSensorTeam/ldlidar_stl_ros2` **com `.git` próprio dentro**.
Um `git add -A` cru registraria um gitlink — um ponteiro pra um SHA — sem
`.gitmodules`, e os 58 arquivos ficariam de fora. A rede de segurança teria um
buraco exatamente do tamanho do driver do LiDAR, e pior: pareceria salva.

Havia trabalho local não commitado lá dentro: uma linha, `#include <pthread.h>`
no `ldlidar_driver/src/logger/log_module.cpp`, em cima do upstream `bf668a8`.
É um fix de compilação — sem ele o pacote não compila no ROS2 da Pi.

**O que fiz:** commitei o fix **dentro do próprio clone** (`e5c389e`, com a
identidade do dono), e acrescentei `ros2_packages/ldlidar_stl_ros2/` ao
`.git/info/exclude` do repo externo. O fix ficou durável, no lugar onde faz
sentido versionar, e nada virou ponteiro vazio.

**O que quero que seja atacado:**

1. `.git/info/exclude` é invisível — não está versionado e não aparece em
   `git status`. O próximo operador que rodar `git add -A` ali não vai entender
   por que o LiDAR não entra. `.gitignore` versionado seria pior (poluiria a
   `main`), mas o silêncio também cobra. Existe saída melhor?
2. Deixei o clone aninhado como está. As alternativas eram submódulo de verdade
   (mexe na `main`) ou `rm -rf .git` lá dentro pra absorver os 58 arquivos
   (perde o vínculo com o upstream e engorda o repo). Escolhi não decidir isso
   hoje, no meio de um transplante. Concorda com adiar?
3. ~~O fix do `pthread.h` está agora só no clone da Pi; se o cartão morrer, ele
   morre junto.~~ **Errado, corrigido pelo Codex.** O `setup_pi.sh:132-138`
   reaplica esse patch sozinho, de forma idempotente, logo depois de clonar o
   driver (`setup_pi.sh:119-129`). Cartão novo + `setup_pi.sh` = fix de volta. O
   commit `e5c389e` no clone é útil (deixa o ajuste rastreável e sobrevive a um
   `git checkout` interno), mas não é a única cópia. Upstream do clone:
   `bf668a8`, `github.com/ldrobotSensorTeam/ldlidar_stl_ros2`, branch `master`,
   clone raso — o `setup_pi.sh:123-126` já registra que o hash é o que aponta o
   estrago se o upstream rebatear.

### 2.2 Resolução do conflito no `ESTADO_PROJETO.md`

O plano diz: "ficar com a versão DA PI e acrescentar só a seção da BNO. NÃO
usar `--theirs`: apagaria o histórico específico deste robô."

No real, os marcadores cercavam **só** duas coisas: a linha
`> ... Atualizado em **<data>**` e a seção `## 🧭 2026-08-16 — segunda IMU`
inteira (linhas 4 a 89 de 1672). Todo o histórico da Pi — inclusive a seção
`## 🚶 2026-07-22 — SEGUIR PESSOA` — estava **fora** dos marcadores, intocado.

**O que fiz:** apaguei as 3 linhas de marcador e a linha da data do lado HEAD,
ficando com a data `2026-08-16` que a seção nova traz. Ou seja, dentro daquele
hunk o resultado é idêntico a "theirs" — mas o arquivo inteiro não é, e é essa
a diferença que importa.

**Verificação:** 39 seções `## ` no arquivo depois da resolução, nenhum marcador
restante, seção de 07-22 presente e imediatamente abaixo da nova. Guardei o
arquivo pré-resolução em `/tmp/estado.pre-resolve` na Pi.

**A data do cabeçalho — respondido pelo Codex.** Ficou `2026-08-16` e eu achei
que estivesse errado. Não está: a convenção deste projeto é a data do
**acontecimento mais recente documentado**, não a data em que o arquivo foi
editado. `2026-08-16` está coerente enquanto a seção da BNO for a mais nova.
Quando entrar a seção de `2026-09-03` (bancada + transplante + resultado do
passo 6), o cabeçalho muda junto com ela.

---

## 3. O que ficou por fazer

**Passo 6 e a validação.** Não subi a stack: é o portão do dono, e é a primeira
coisa que energiza o robô hoje.

```bash
nohup ros2 launch robot_nav robot.launch.py \
      use_flow:=false use_imu2_heading:=false > /tmp/estreia_bno.log 2>&1 &
sleep 10 && tail -20 /tmp/estreia_bno.log
ros2 topic hz /imu2/data     # ~50 Hz
ros2 topic hz /imu/data      # tem que continuar viva
python3 ros2_packages/robot_nav/tools/imu2_check.py
```

**Uma dívida de doc, deliberada.** A seção da BNO que entrou pelo cherry-pick
traz uma lista "⏭️ Falta fazer" cujos itens 1 a 4 (montar, gravar, testar
sinal, testar magnitude) **já foram feitos hoje na bancada** — os resultados
estão na §1 do `TRANSPLANTE_BNO_2026-09-03.md`. Não editei: misturar redação de
doc com transplante de código no mesmo commit envelhece mal. Fica pro registro
de hoje.

**Push.** Nada foi empurrado pro `origin`. A branch `pi-bno-2026-09-03` existe
só no disco da Pi, por decisão da §12 (a branch isola o estado operacional
deste robô e não deve ser mesclada na `main`).

---

## 4. Rollback, se precisar

Continua valendo o da §7, com o alvo agora concreto:

```bash
git reset --hard 1b2da85          # antes do cherry-pick e do patch
colcon build --base-paths ros2_packages --symlink-install --packages-select robot_nav
```

O passo do firmware (worktree em `860ce20` + `pio run -t upload`) segue
desaconselhado: desfaz junto o fix do relê, e o firmware de hoje foi medido
compatível com o bridge antigo.

---

## 5. Revisão do Codex, depois da execução

Conferiu a Pi por conta própria e bateu tudo: branch, `1b2da85` como rollback,
`42f79ba`, `9f40b92`, build 7,89 s, árvore externa e clone do LiDAR limpos,
nenhuma stack rodando.

| ponto | veredito |
|---|---|
| Manter o LiDAR como repo aninhado por ora | **Correto.** Não é hora de virar submódulo |
| `.git/info/exclude` | **Temporário.** O certo é `/ros2_packages/ldlidar_stl_ros2/` no `.gitignore` versionado — não polui a `main`, porque o `setup_pi.sh` já declara que a pasta é clonada, não versionada |
| "o fix do pthread só existe na Pi" | **Errado meu.** O `setup_pi.sh:132` reaplica. Ver §2.1 |
| Resolução do `ESTADO_PROJETO.md` | **Correta.** Não apagou o histórico do seguir-pessoa |
| Data `2026-08-16` no cabeçalho | **Coerente.** Convenção é a data do fato mais recente documentado. Ver §2.2 |
| Passo 6 | **Liberado tecnicamente.** Com rodas suspensas ou área livre, e controle intocado |

---

## 6. Passo 6 — a estreia, com o robô no chão e o dono presente

Autorizado pelo dono depois do aval técnico do Codex. Robô parado, sem controle
na mão; os giros do teste de sinal foram feitos **na mão**, nenhum motor
comandado.

Subiu em segundo plano (`ROS_DOMAIN_ID=42`, o mesmo do `launch.sh:35`):

```
nohup ros2 launch robot_nav robot.launch.py use_flow:=false use_imu2_heading:=false
```

A linha que prova que a estreia é a conservadora que foi aprovada:

```
pose_estimator: ... | BNO055 peso_giro=0.50 sinal=+1 heading=OFF
mega_bridge: BNO055 calib: {"accel": 0, "gyro": 3, "mag": 0, "sys": 0}
```

### Taxas

| tópico | medido | antes |
|---|---|---|
| `/imu2/data` | **50,007 Hz** | **não existia** |
| `/imu/data` | **49,685 Hz** | 49,2 Hz na bancada — o I²C compartilhado não quebrou |

### Teste de sinal — o que a §1 do plano previu, confirmado dentro do ROS

Giro de **90° reais pra esquerda**, na mão. As duas taxas **já corrigidas** pelos
sinais de montagem (`-1` pra MPU, `+1` pra BNO):

| | pico |
|---|---|
| `gz1` (MPU × −1) | **+50,4 °/s** |
| `gz2` (BNO × +1) | **+53,3 °/s** |

**Mesmo sinal, o tempo todo, e módulos a ~5% um do outro.** Zero ocorrências de
discordância no log da stack — o `pose_estimator` fundiu as duas do começo ao
fim (`src=imu+imu2` em toda linha). **`imu2_yaw_sign = 1.0` confirmado no robô
montado, não só na sonda serial.** A regra que o `ROTEIRO_CAMPO.md` §0.4 dizia
errado teria mandado inverter isso.

### Teste de magnitude

Giro de 90° reais: `yaw_abs` foi de `+0,0` a **`+88,4°`** enquanto o `yaw_odom`
foi a **`+89,7°`**. Os dois concordam dentro de **1,3°**, e esquerda deu yaw
**positivo** — convenção ROS. Passou.

### O que não passou (e não precisava)

`calib mag = 0` e `|B|` caindo de ~40 µT parado pra ~22 µT durante o giro: é o
offset de ferro-duro já medido na bancada (18–32 µT). Como a estreia subiu com
`use_imu2_heading:=false`, a âncora magnética está desligada e isso não afeta
nada. `anchor=False` em todas as linhas, como esperado. Calibrar o mag (∞ no ar)
fica pro dia em que a âncora for ligada.

### Um blip, registrado

Um único `BNO055 stale (age=0.31 s > 0.30 s)` em **153 s** de stack no ar — um
atraso de 10 ms acima do limiar, que se resolveu em **40 ms** (`BNO055 voltou`).
Um evento em ~7.600 frames. A degradação foi a projetada: o yaw voltou pro MPU
sozinho e nada travou. Não é crônico, mas fica anotado: se virar rotina em
campo, o limiar de 0,30 s é apertado demais pra um frame de 50 Hz que divide
I²C com a MPU.
