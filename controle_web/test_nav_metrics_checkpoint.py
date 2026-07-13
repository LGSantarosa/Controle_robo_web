# Checkpoint do nav_metrics: a tentativa EM ANDAMENTO sobrevive a apagão seco
# de bateria (Pi morre sem shutdown). Testa só a camada de persistência —
# o coletor é instanciado via __new__ pra não subir nó ROS.

import csv
import json
import os
import threading

import pytest

from nav_metrics import (NavMetricsCollector, NavAttempt, CSV_FIELDS,
                         CKPT_NAME, CKPT_INTERVAL_S)


@pytest.fixture
def col(tmp_path):
    c = NavMetricsCollector.__new__(NavMetricsCollector)
    c._log_dir = str(tmp_path)
    c._csv_lock = threading.Lock()
    c._ckpt_path = os.path.join(str(tmp_path), CKPT_NAME)
    c._ckpt_last = 0.0
    c._attempt = None
    c._last_odom_pose = (1.5, -2.0, 0.7)
    c._now = lambda: 1000.0
    yield c


def attempt(**kw):
    kw.setdefault('nav_id', 'abcd1234')
    kw.setdefault('start_ts', 990.0)
    a = NavAttempt(**kw)
    a.distance_traveled_m = 3.25
    return a


def read_csv(path):
    with open(path) as f:
        return list(csv.reader(f))


def test_checkpoint_escrito_com_goal_ativo(col):
    col._attempt = attempt()
    col._maybe_checkpoint(now=1000.0)
    with open(col._ckpt_path) as f:
        data = json.load(f)
    row = data['row']
    assert row[0] == 'abcd1234'
    assert row[CSV_FIELDS.index('status')] == 'POWERLOSS'
    assert row[CSV_FIELDS.index('distance_traveled_m')] == '3.250'
    # pose provisória = onde o robô estava no checkpoint
    assert row[CSV_FIELDS.index('end_x')] == '1.500'


def test_checkpoint_respeita_intervalo(col):
    col._attempt = attempt()
    col._maybe_checkpoint(now=1000.0)
    os.unlink(col._ckpt_path)
    col._maybe_checkpoint(now=1000.0 + CKPT_INTERVAL_S / 2)   # cedo demais
    assert not os.path.exists(col._ckpt_path)
    col._maybe_checkpoint(now=1000.0 + CKPT_INTERVAL_S + 0.1)
    assert os.path.exists(col._ckpt_path)


def test_sem_goal_nao_escreve(col):
    col._maybe_checkpoint(now=1000.0)
    assert not os.path.exists(col._ckpt_path)


def test_flush_limpo_apaga_checkpoint(col):
    col._attempt = attempt()
    col._maybe_checkpoint(now=1000.0)
    a = col._attempt
    a.status = 4   # SUCCEEDED
    a.end_ts = 1005.0
    col._flush_attempt(a)
    assert not os.path.exists(col._ckpt_path)
    [csv_file] = [f for f in os.listdir(col._log_dir) if f.endswith('.csv')]
    rows = read_csv(os.path.join(col._log_dir, csv_file))
    assert len(rows) == 2   # header + linha terminal (POWERLOSS não vazou)
    assert rows[1][CSV_FIELDS.index('status')] == 'SUCCEEDED'


def test_recover_apos_apagao_vira_linha_powerloss(col):
    # sessão 1: goal ativo, checkpoint no disco, "bateria morre"
    col._attempt = attempt()
    col._maybe_checkpoint(now=1000.0)
    # sessão 2: boot recupera
    col._recover_checkpoint()
    assert not os.path.exists(col._ckpt_path)
    [csv_file] = [f for f in os.listdir(col._log_dir) if f.endswith('.csv')]
    rows = read_csv(os.path.join(col._log_dir, csv_file))
    assert rows[0] == CSV_FIELDS
    assert rows[1][CSV_FIELDS.index('status')] == 'POWERLOSS'
    assert rows[1][0] == 'abcd1234'
    # duração até o último checkpoint (1000.0 − 990.0)
    assert rows[1][CSV_FIELDS.index('duration_s')] == '10.00'


def test_recover_sem_checkpoint_e_noop(col, tmp_path):
    col._recover_checkpoint()
    assert [f for f in os.listdir(str(tmp_path)) if f.endswith('.csv')] == []


def test_recover_checkpoint_corrompido_descarta(col):
    with open(col._ckpt_path, 'w') as f:
        f.write('{lixo')
    col._recover_checkpoint()
    assert not os.path.exists(col._ckpt_path)
    assert [f for f in os.listdir(col._log_dir) if f.endswith('.csv')] == []
