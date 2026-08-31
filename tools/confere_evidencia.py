#!/usr/bin/env python3
"""Confere as pastas de evidência de `docs/baselines/`.

Por que existe (2026-08-31): a evidência arquivada já quebrou TRÊS vezes, sempre
do mesmo jeito e sempre achada por outra pessoa, não por mim:

  BO 31 — `colisao.log` fora do git (`*.log` no .gitignore) e CSV com CRLF;
  BO 45 — REINCIDÊNCIA: `probe.log` idem, num README que eu tinha acabado de
          escrever depois de LER a lição do 31;
  BO 50 — REINCIDÊNCIA do CRLF: `csv.writer` termina linha com \\r\\n por
          DEFAULT, e o `newline=''` que a doc do módulo manda usar preserva.

A lição escrita não protegeu nada nas duas reincidências. Um passo executável
protege. Uso:

    python3 tools/confere_evidencia.py            # confere tudo
    python3 tools/confere_evidencia.py --autoteste

Sai 0 se está tudo certo, 1 se achou problema (imprime o quê e onde).
"""
import os
import re
import subprocess
import sys

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join('docs', 'baselines')
# extensões que valem como "arquivo de evidência citado" num README
CITAVEL = re.compile(r'\.(csv|json|txt|md|log|yaml|sdf)$')


def rastreados():
    out = subprocess.run(['git', 'ls-files'], cwd=RAIZ,
                         capture_output=True, text=True).stdout
    return set(out.split('\n'))


def citados(texto):
    """Nomes de arquivo que o README AFIRMA estarem arquivados aqui.

    Só conta o que está entre crases DENTRO DA TABELA de evidência (linhas que
    começam com `|`). Fora da tabela o README fala de arquivo que ele
    deliberadamente NÃO arquiva ("não versionado aqui: `nav2.log`"), de
    referência cruzada (`DIARIO_ARENA.md`) e da história de um rename — nada
    disso é promessa de arquivo presente. A primeira versão desta função lia o
    texto inteiro e reprovou 7 coisas certas na primeira execução: validador que
    grita à toa é validador que se aprende a ignorar.

    Um `{a,b}` de shell não é nome de arquivo: o README tem que listar os três.
    """
    nomes = set()
    for linha in texto.split('\n'):
        if not linha.lstrip().startswith('|'):
            continue
        # PRIMEIRA coluna = o nome do arquivo. A coluna de descrição conta a
        # história do arquivo ("Era `colisao.log` e não entrava no git") e citar
        # o nome velho lá não é promessa de que ele existe.
        celulas = [c.strip() for c in linha.strip().strip('|').split('|')]
        if not celulas:
            continue
        nomes |= {c for c in re.findall(r'`([^`]+)`', celulas[0])
                  if CITAVEL.search(c) and '/' not in c
                  and '{' not in c and '*' not in c}
    return nomes


def confere(pasta, trk, problemas):
    rd = os.path.join(pasta, 'README.md')
    if not os.path.exists(os.path.join(RAIZ, rd)):
        problemas.append('%s: sem README.md' % pasta)
        return
    texto = open(os.path.join(RAIZ, rd), encoding='utf-8').read()
    cit = citados(texto)

    for nome in sorted(cit):
        rel = os.path.join(pasta, nome)
        if not os.path.exists(os.path.join(RAIZ, rel)):
            problemas.append('%s: README cita, mas NAO EXISTE' % rel)
        elif rel not in trk:
            problemas.append('%s: README cita, mas NAO ESTA NO GIT '
                             '(some num clone limpo)' % rel)

    for nome in sorted(os.listdir(os.path.join(RAIZ, pasta))):
        rel = os.path.join(pasta, nome)
        if nome != 'README.md' and nome not in cit:
            problemas.append('%s: existe, mas o README nao cita' % rel)
        if rel in trk or os.path.exists(os.path.join(RAIZ, rel)):
            dados = open(os.path.join(RAIZ, rel), 'rb').read()
            if b'\r\n' in dados:
                problemas.append('%s: CRLF (use lineterminator="\\n" no csv.writer)' % rel)
            if any(l.rstrip(b'\r\n') != l.rstrip() for l in dados.split(b'\n') if l):
                problemas.append('%s: espaco no fim de linha' % rel)


def _e2e(tmp, problemas_esperados, trk=None):
    """Roda o `confere()` de verdade contra uma pasta montada na hora.

    `trk` permite injetar um conjunto de rastreados FALSO. Sem isso o caso feliz
    não é feliz: um arquivo temporário nunca está no git, e a primeira versão
    deste teste filtrava justamente esse erro pra fingir que passava — o revisor
    pegou. Filtrar o erro que o teste deveria provar ausente é teste decorativo.
    """
    achados = []
    confere(os.path.relpath(tmp, RAIZ),
            rastreados() if trk is None else trk, achados)
    faltou = [e for e in problemas_esperados
              if not any(e in a for a in achados)]
    return achados, faltou


def autoteste_confere():
    """PONTA-A-PONTA: monta pastas de mentira e roda o `confere()`.

    A primeira versão deste autoteste cobria só o `citados()` — a extração de
    nomes — e eu ainda assim chamei de "ponta-a-ponta" (o revisor pegou). É o
    BO 20 outra vez: autoteste que cobre a metade fácil não protege a metade que
    falha. Cada caso abaixo é uma das promessas do script.
    """
    import shutil
    import tempfile
    base = tempfile.mkdtemp(dir=os.path.join(RAIZ, BASE), prefix='_autoteste_')
    ruim = 0
    try:
        casos = [
            ('arquivo citado que NAO existe',
             {'README.md': '| arquivo | x |\n|---|---|\n| `sumiu.csv` | y |\n'},
             ['sumiu.csv: README cita, mas NAO EXISTE']),
            ('arquivo citado que existe mas o git IGNORA (BO 31/45)',
             {'README.md': '| arquivo | x |\n|---|---|\n| `x.log` | y |\n',
              'x.log': 'sou ignorado pelo *.log\n'},
             ['x.log: README cita, mas NAO ESTA NO GIT']),
            ('CRLF (BO 31/50)',
             {'README.md': '| arquivo | x |\n|---|---|\n| `a.csv` | y |\n',
              'a.csv': 'a,b\r\n1,2\r\n'},
             ['a.csv: CRLF']),
            ('espaco no fim de linha',
             {'README.md': '| arquivo | x |\n|---|---|\n| `b.csv` | y |\n',
              'b.csv': 'a,b \n1,2\n'},
             ['b.csv: espaco no fim de linha']),
            ('arquivo na pasta que o README NAO cita',
             {'README.md': '| arquivo | x |\n|---|---|\n',
              'orfao.csv': 'a\n'},
             ['orfao.csv: existe, mas o README nao cita']),
            ('sem README',
             {'so_isso.csv': 'a\n'},
             ['sem README.md']),
        ]
        for nome, arquivos, esperado in casos:
            d = tempfile.mkdtemp(dir=base)
            for f, conteudo in arquivos.items():
                with open(os.path.join(d, f), 'w', newline='') as fh:
                    fh.write(conteudo)
            achados, faltou = _e2e(d, esperado)
            ok = not faltou
            ruim += not ok
            print('%s e2e: %-52s %s' % ('ok  ' if ok else 'FALHA', nome,
                                        '' if ok else 'NAO pegou: %s' % faltou))
        # o caso feliz, agora PURO: pasta correta com os arquivos declarados
        # rastreados -> ZERO achado, sem filtrar nada.
        d = tempfile.mkdtemp(dir=base)
        open(os.path.join(d, 'README.md'), 'w').write(
            '| arquivo | x |\n|---|---|\n| `ok.csv` | y |\n')
        open(os.path.join(d, 'ok.csv'), 'w').write('a,b\n1,2\n')
        rel = os.path.relpath(d, RAIZ)
        trk_falso = {os.path.join(rel, 'README.md'), os.path.join(rel, 'ok.csv')}
        achados, _ = _e2e(d, [], trk=trk_falso)
        ruim += bool(achados)
        print('%s e2e: %-52s %s' % ('ok  ' if not achados else 'FALHA',
                                    'pasta correta: ZERO achado (trk injetado)',
                                    '' if not achados else achados))
    finally:
        shutil.rmtree(base, ignore_errors=True)
    return ruim


def autoteste():
    """Prova que o conferidor REPROVA o que ele promete pegar — senão ele é
    decoração. Cada caso é o BO que ele existe pra impedir."""
    casos = [
        ('| `a.csv` | o que sustenta |', {'a.csv'}, 'linha de tabela: conta'),
        ('| `x.log` | idem |', {'x.log'}, 'tabela com .log (o BO 31/45): conta'),
        ('| `novo.txt` | **Era `velho.log`** e nao entrava no git |', {'novo.txt'},
         'nome VELHO na coluna de descricao nao e promessa de arquivo'),
        ('Nao versionado aqui: `nav2.log`.', set(),
         'PROSA nao conta — o README diz que NAO esta aqui'),
        ('Ver `DIARIO_ARENA.md` para a analise.', set(),
         'PROSA: referencia cruzada nao e arquivo desta pasta'),
        ('| `r{1,2}.json` | x |', set(), 'chave de shell NAO e nome de arquivo'),
        ('| `*.log` | x |', set(), 'glob NAO e nome de arquivo'),
        ('| `docs/x/a.csv` | x |', set(), 'caminho com barra e de outra pasta'),
        ('| `nada` | x |', set(), 'palavra sem extensao nao e arquivo'),
    ]
    ruim = 0
    for texto, esperado, porque in casos:
        got = citados(texto)
        ok = got == esperado
        ruim += not ok
        print('%s %-42s %r' % ('ok  ' if ok else 'FALHA', porque, sorted(got)))
    return ruim


def main():
    if '--autoteste' in sys.argv:
        return 1 if (autoteste() + autoteste_confere()) else 0
    base = os.path.join(RAIZ, BASE)
    if not os.path.isdir(base):
        print('sem %s — nada a conferir' % BASE)
        return 0
    trk, problemas = rastreados(), []
    pastas = sorted(d for d in os.listdir(base)
                    if os.path.isdir(os.path.join(base, d)))
    for d in pastas:
        confere(os.path.join(BASE, d), trk, problemas)
    for p in problemas:
        print('✗ %s' % p)
    print('%d pasta(s) conferida(s), %d problema(s)' % (len(pastas), len(problemas)))
    return 1 if problemas else 0


if __name__ == '__main__':
    sys.exit(main())
