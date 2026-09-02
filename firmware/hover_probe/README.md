# hover_probe — sonda de bancada da placa de hoverboard

Diagnóstico de uma placa de hover **fora** do robô: MEGA no PC, placa com bateria
própria, sem Pi e sem ROS.

Existe porque as duas ferramentas que já tínhamos param cedo demais:

- o `mega_bridge` só repassa RPM e bateria no frame STATE — `cmd1`/`cmd2` e
  `boardTemp`, que é onde mora o diagnóstico, ficam invisíveis;
- o `test_mega.py` diz *"placa não responde"* e para aí, sem dizer por quê.

## Uso

```bash
pio run -t upload --upload-port /dev/ttyACM0
pio device monitor -b 115200 --port /dev/ttyACM0
```

Neste PC não existe `/dev/mega` (o `setup_udev.sh` nunca rodou aqui), por isso o
`--upload-port` é obrigatório.

**Parado, a sonda não move nada**: manda `speed=0` a 50 Hz e imprime o que a placa
responde. Movimento só com tecla:

| tecla | ação |
|---|---|
| `g` | pulso de 0,5 s a `speed=+300` |
| `r` | pulso de 0,5 s a `speed=-300` |
| `s` | imprime uma linha de status agora |

> O sketch **não aciona no boot**, de propósito. Uma versão sem esse gatilho faria
> o robô sair andando ao ser flasheado — `pio run -t upload` reseta a MEGA e o
> `setup()` roda em seguida. Aconteceu duas vezes na bancada de 2026-09-01.
> **Se for mexer neste arquivo, mantenha o gatilho.**

## Como ler a saída

| sinal | significa |
|---|---|
| `bat = 0,00 V` | a placa não responde: cabo, GND, ou desligada |
| feedback ecoa o que você mandou | curto TX↔RX; a placa não está na conversa |
| `cmd2` acompanha o `speed` | a placa **recebeu e aceitou** o comando |
| `cmd2` fica em 0 | a placa ignora serial (firmware sem `CONTROL_SERIAL_USART2`) |
| `bat` e `temp` **imóveis** | corrente zero — não energizou |
| `bat` afunda / `temp` sobe | corrente circulou: fase conectada |
| `spdR`/`spdL` mexem girando na mão | hall bom, e diz **em qual canal** a roda está |

O modo parado já é o teste de hall: gire a roda com a mão e olhe `spdR`/`spdL`.

## A placa não gira mesmo aceitando o comando?

É trava de segurança do firmware dela, não defeito. Precisa das **duas rodas**
plugadas *e* de **girá-las com a mão** (o `errCode` de hall não limpa com a roda
parada). O beep é o indicador: constante = travada, mudou = armou.

Receita completa, com os testes e os números: `BANCADA_HOVER_2026-09-01.md`.
