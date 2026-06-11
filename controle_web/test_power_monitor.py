"""
Testes do power_monitor (detector de eventos elétricos + CSV).

Tudo lógica pura — sem rclpy. O objetivo do monitor é distinguir, no desarme
do BMS do hoverboard:
  * stall/rotor bloqueado → SAG gradual (com STALL junto) ANTES do TRIP
  * mau contato           → TRIP seco, sem SAG nem STALL antes
"""
import csv
import glob
import os

from power_monitor import PowerCsvLogger, PowerEventDetector


def make_detector(**kw):
    return PowerEventDetector(**kw)


def run_ticks(det, rows):
    """rows = lista de (t, v_front, v_rear, (set_l, set_r), (fl, fr, rl, rr)).
    Retorna lista achatada de eventos na ordem em que dispararam."""
    out = []
    for t, vf, vr, sets, meas in rows:
        out.extend(det.update(t, vf, vr, sets, meas))
    return out


PARADO = ((0.0, 0.0), (0.0, 0.0, 0.0, 0.0))


def test_nominal_sem_eventos():
    det = make_detector()
    rows = [(i * 0.1, 39.0, 39.0, *PARADO) for i in range(30)]
    assert run_ticks(det, rows) == []
    assert det.front_ok and det.rear_ok
    assert not det.stall_active


def test_sag_gradual_dispara_antes_do_trip():
    # Tensão da frente caindo 1V/tick (10 Hz): sag (queda >3V em <1s) tem que
    # vir antes do trip (<30V) — é a assinatura de stall/sobrecarga.
    det = make_detector()
    rows = [(i * 0.1, max(39.0 - i, 0.0), 39.0, *PARADO) for i in range(20)]
    events = run_ticks(det, rows)
    assert events == ['sag_front', 'trip_front']


def test_queda_instantanea_e_so_trip():
    # 39V → 0V num tick (mau contato / BMS cortou): trip seco, sem sag.
    det = make_detector()
    rows = [(0.0, 39.0, 39.0, *PARADO),
            (0.1, 39.0, 39.0, *PARADO),
            (0.2, 0.0, 39.0, *PARADO)]
    assert run_ticks(det, rows) == ['trip_front']
    assert not det.front_ok
    assert det.rear_ok


def test_trip_nao_repete_enquanto_morto_e_rearma_apos_recuperar():
    det = make_detector()
    rows = [(0.0, 39.0, 39.0, *PARADO)]
    rows += [(0.1 + i * 0.1, 0.0, 39.0, *PARADO) for i in range(20)]   # morto 2s
    rows += [(2.2 + i * 0.1, 39.0, 39.0, *PARADO) for i in range(10)]  # religou
    rows += [(3.3, 0.0, 39.0, *PARADO)]                                # cai de novo
    assert run_ticks(det, rows) == ['trip_front', 'trip_front']


def test_placa_morta_desde_o_inicio_nao_e_trip():
    # MEGA ligada mas hoverboard desligado: tensão 0 desde o boot — não há
    # transição viva→morta, então não é evento.
    det = make_detector()
    rows = [(i * 0.1, 0.0, 0.0, *PARADO) for i in range(20)]
    assert run_ticks(det, rows) == []
    assert not det.front_ok and not det.rear_ok


def test_stall_dispara_apos_hold_e_nao_repete():
    # Comando forte (200 units) com roda parada por >0.5s = stall. Persistindo,
    # o evento não repete (stall_active fica True).
    det = make_detector()
    rows = [(i * 0.1, 39.0, 39.0, (200.0, 0.0), (0.0, 0.0, 0.0, 0.0))
            for i in range(15)]
    events = run_ticks(det, rows)
    assert events == ['stall']
    assert det.stall_active


def test_stall_limpa_quando_roda_gira_e_pode_rearmar():
    det = make_detector()
    stall_rows = [(i * 0.1, 39.0, 39.0, (200.0, 0.0), (0.0, 0.0, 0.0, 0.0))
                  for i in range(10)]
    free_rows = [(1.0 + i * 0.1, 39.0, 39.0, (200.0, 0.0), (30.0, 0.0, 30.0, 0.0))
                 for i in range(5)]
    stall2_rows = [(1.5 + i * 0.1, 39.0, 39.0, (200.0, 0.0), (0.0, 0.0, 0.0, 0.0))
                   for i in range(10)]
    events = run_ticks(det, stall_rows + free_rows + stall2_rows)
    assert events == ['stall', 'stall']


def test_rodando_livre_nao_e_stall():
    det = make_detector()
    rows = [(i * 0.1, 39.0, 39.0, (200.0, 200.0), (30.0, 30.0, 30.0, 30.0))
            for i in range(20)]
    assert run_ticks(det, rows) == []
    assert not det.stall_active


def test_comando_zero_com_roda_parada_nao_e_stall():
    det = make_detector()
    rows = [(i * 0.1, 39.0, 39.0, *PARADO) for i in range(20)]
    run_ticks(det, rows)
    assert not det.stall_active


def test_csv_logger_escreve_header_e_linhas(tmp_path):
    logger = PowerCsvLogger(str(tmp_path))
    logger.log(ts=1.0, v_front=39.0, v_rear=38.9,
               setpoints=(100.0, -100.0), measured=(10.0, -10.0, 9.5, -9.5),
               stall=False, events=[])
    logger.log(ts=1.1, v_front=33.0, v_rear=38.9,
               setpoints=(200.0, 200.0), measured=(0.0, 0.0, 0.0, 0.0),
               stall=True, events=['sag_front', 'stall'])
    logger.close()

    files = glob.glob(os.path.join(str(tmp_path), 'power_*.csv'))
    assert len(files) == 1
    with open(files[0]) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert rows[0]['v_front'] == '39.00'
    assert rows[0]['event'] == ''
    assert rows[1]['event'] == 'sag_front|stall'
    assert rows[1]['stall'] == '1'
    assert rows[1]['set_left'] == '200.0'
    assert rows[1]['meas_fl'] == '0.0'
