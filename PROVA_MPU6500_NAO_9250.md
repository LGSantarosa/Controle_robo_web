# Laudo técnico — módulo vendido como "MPU-9250" é, na verdade, um MPU-6500 (sem magnetômetro)

**Data do teste:** 2026-06-29
**Equipamento sob teste:** módulo IMU em formato GY-9250/6500, conectado por I²C
(SDA/SCL) a um Arduino MEGA 2560.
**Conclusão:** o chip soldado no módulo é um **InvenSense MPU-6500** (3 eixos de
giroscópio + 3 eixos de acelerômetro). **NÃO é um MPU-9250 e NÃO possui
magnetômetro.** O módulo está fisicamente em formato de placa GY-9250 (que é o
mesmo PCB da GY-6500), mas o componente populado é o 6500.

---

## 1. Resumo da prova

Um MPU-9250 verdadeiro se distingue de um MPU-6500 por **dois fatos objetivos e
mensuráveis**, ambos definidos pela própria InvenSense (fabricante do chip):

| Característica | MPU-9250 (genuíno) | MPU-6500 | O que este módulo respondeu |
|---|---|---|---|
| `WHO_AM_I` (registrador 0x75) | **`0x71`** | **`0x70`** | **`0x70`** ❌ |
| Magnetômetro AK8963 no I²C `0x0C` | **presente, responde `0x48`** | **não existe** | **não responde** ❌ |

Os dois testes deram resultado de **MPU-6500**. São independentes entre si e
foram confirmados por dois programas distintos (ver Seção 4). Não há configuração
de software, fiação ou alimentação que faça um magnetômetro aparecer se o die do
AK8963 não está fisicamente dentro do encapsulamento.

---

## 2. Evidência bruta (saída do teste)

Saída literal de um *I²C scanner* — o programa de diagnóstico padrão do ecossistema
Arduino, que percorre os 127 endereços do barramento e lista os que respondem
(dão ACK). Foi executado **antes e depois** de habilitar o modo *bypass*
(pass-through), que é o modo pelo qual o host acessa o magnetômetro de um 9250:

```
=== DIAG MAG (sketch de referencia) ===
ANTES do bypass:  enderecos que respondem: 0x68
MPU em 0x68  WHO_AM_I(0x75)=0x70
   -> 0x71=MPU9250(com mag) | 0x73=9255 | 0x70=MPU6500(sem mag)
DEPOIS do bypass: enderecos que respondem: 0x68
AK8963 em 0x0C  WIA(0x00)=0xFF
   -> 0x48 = magnetometro PRESENTE
=== FIM ===
```

Leitura dos resultados:

- **`WHO_AM_I = 0x70`** → assinatura do **MPU-6500**. Um MPU-9250 retornaria `0x71`.
- **No barramento só responde o `0x68`** (o próprio chip inercial), antes e depois
  do bypass. O endereço **`0x0C` do magnetômetro nunca aparece** → não há AK8963
  no módulo.
- **`AK8963 WIA = 0xFF`** → ausência total de resposta no endereço do magnetômetro
  (um magnetômetro presente retornaria `0x48`).

O barramento I²C está comprovadamente sadio: o chip inercial responde, é lido e
fornece dados de giroscópio/acelerômetro normalmente nos mesmos fios SDA/SCL. A
ausência do `0x0C` não é problema de ligação — é ausência do componente.

---

## 3. Fundamentação no datasheet oficial da InvenSense

**a) Um MPU-9250 genuíno contém o magnetômetro AK8963 internamente.**
Documento *Product Specification* **PS-MPU-9250A-01** (InvenSense Inc.), Seção 1.3
"Product Overview":

> "MPU-9250 is a multi-chip module (MCM) consisting of two dies integrated into a
> single QFN package. One die houses the 3-Axis gyroscope and the 3-Axis
> accelerometer. **The other die houses the AK8963 3-Axis magnetometer** from Asahi
> Kasei Microdevices Corporation."

**b) Esse magnetômetro é acessível no endereço I²C 0x0C.**
Mesmo documento, seção de *Pass-Through mode*:

> "Pass-Through mode is also used to access the AK8963 magnetometer directly from
> the host. **In this configuration the slave address for the AK8963 is 0X0C or 12
> decimal.**"

**c) Os valores de `WHO_AM_I` que identificam cada chip** estão nos *Register Map*
oficiais da InvenSense:
- *RM-MPU-9250A-00* (Register Map do MPU-9250): registrador 117 (0x75) `WHO_AM_I`,
  valor de reset = **`0x71`**.
- *Register Map do MPU-6500*: registrador 0x75 `WHO_AM_I` = **`0x70`**.

Cruzando: o datasheet diz que um 9250 **tem** o AK8963 em `0x0C` e responde `0x71`
no `WHO_AM_I`. Este módulo **não tem** nada em `0x0C` e responde `0x70`. Logo, **não
é um MPU-9250** — é um MPU-6500.

---

## 4. Método (reprodutível — o fornecedor pode repetir)

1. Módulo ligado por I²C a um microcontrolador (SDA, SCL, VCC, GND). Nenhuma
   conexão extra é necessária: o magnetômetro de um 9250 é **interno**, no mesmo
   barramento, não precisa de fio adicional.
2. Ler o registrador `0x75` (`WHO_AM_I`) do chip inercial (endereço I²C `0x68`).
3. Habilitar o *bypass* para expor o magnetômetro ao host, conforme o datasheet:
   - `PWR_MGMT_1 (0x6B) = 0x00` (acorda o chip)
   - `USER_CTRL (0x6A) = 0x00` (desliga o mestre I²C interno)
   - `INT_PIN_CFG (0x37) = 0x02` (liga `BYPASS_EN`)
4. Varrer o barramento I²C (scanner padrão) e ler o `WIA (0x00)` do AK8963 em `0x0C`.

Esse resultado foi obtido por **dois programas independentes**: (i) o I²C scanner
de referência mostrado acima, e (ii) o firmware próprio do robô (driver direto por
registrador). **Ambos retornaram exatamente os mesmos valores** (`WHO_AM_I = 0x70`,
sem resposta em `0x0C`), descartando erro de implementação de software.

---

## 5. Pedido

O componente entregue é um **MPU-6500** (sem magnetômetro), enquanto o pedido era
um **MPU-9250** (com magnetômetro AK8963). Provavelmente houve **envio do SKU
errado** — a GY-9250 e a GY-6500 compartilham o mesmo PCB e o mesmo silk
"9250/6500", então a placa tem aparência idêntica, mas o chip populado é o 6500.

**Solicito a troca por um módulo MPU-9250 genuíno**, cujo `WHO_AM_I` (registrador
0x75) retorne **`0x71`** e cujo magnetômetro AK8963 responda no endereço I²C
**`0x0C`** (`WIA = 0x48`).
