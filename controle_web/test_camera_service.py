# Testes do camera_service — parser MJPEG e máquina de estados da gravação.
# Nada aqui toca hardware: o serviço sobe com autostart=False e os frames
# são injetados direto em _handle_frame com relógio falso.
import os

import pytest

from camera_service import CameraService, MjpegSplitter


def jpeg(payload: bytes = b'x') -> bytes:
    return b'\xff\xd8' + payload + b'\xff\xd9'


# ---------------- MjpegSplitter ----------------

def test_splitter_frame_inteiro():
    s = MjpegSplitter()
    assert s.feed(jpeg(b'abc')) == [jpeg(b'abc')]


def test_splitter_frame_partido_em_chunks():
    s = MjpegSplitter()
    f = jpeg(b'0123456789')
    assert s.feed(f[:3]) == []
    assert s.feed(f[3:7]) == []
    assert s.feed(f[7:]) == [f]


def test_splitter_varios_frames_num_chunk():
    s = MjpegSplitter()
    fs = [jpeg(b'a'), jpeg(b'b'), jpeg(b'c')]
    assert s.feed(b''.join(fs)) == fs


def test_splitter_lixo_antes_do_soi():
    s = MjpegSplitter()
    assert s.feed(b'\x00\x01lixo' + jpeg(b'ok')) == [jpeg(b'ok')]


def test_splitter_soi_partido_na_borda_do_chunk():
    # 0xFF no fim de um chunk, 0xD8 no começo do próximo
    s = MjpegSplitter()
    f = jpeg(b'zz')
    assert s.feed(b'lixo\xff') == []
    assert s.feed(f[1:]) == [f]


def test_splitter_descarta_frame_gigante():
    s = MjpegSplitter(max_buffer=64)
    assert s.feed(b'\xff\xd8' + b'\x00' * 100) == []   # nunca fecha → descarta
    assert s.feed(jpeg(b'novo')) == [jpeg(b'novo')]    # ressincroniza


# ---------------- Gravação ----------------

@pytest.fixture
def cam(tmp_path):
    # rec_fps = fps → stride 1 (grava todo frame); a decimação tem teste próprio
    c = CameraService(log_dir=str(tmp_path), rec_fps=30, idle_grace_s=8.0,
                      max_rec_s=1800.0, autostart=False)
    c._available = True                  # finge câmera viva
    c._remux = lambda path, fps: None    # JPEGs falsos não remuxam — mantém o .mjpeg
    yield c


def rec_files(cam):
    return [f for f in os.listdir(cam._log_dir) if f.endswith('.mjpeg')]


def test_sem_nav_nao_grava(cam):
    cam._handle_frame(jpeg(), now=100.0)
    assert cam.status()['recording'] is False
    assert rec_files(cam) == []


def test_nav_active_inicia_gravacao(cam):
    cam.nav_active('rota')
    cam._handle_frame(jpeg(b'f1'), now=100.0)
    cam._handle_frame(jpeg(b'f2'), now=100.1)
    st = cam.status()
    assert st['recording'] is True
    assert 'rota' in st['file']
    cam.stop_recording('teste')
    [f] = rec_files(cam)
    with open(os.path.join(cam._log_dir, f), 'rb') as fh:
        assert fh.read() == jpeg(b'f1') + jpeg(b'f2')


def test_gravacao_decimada_15fps(tmp_path):
    # default rec_fps=15 com câmera a 30 → grava frame sim, frame não
    c = CameraService(log_dir=str(tmp_path), autostart=False)
    c._available = True
    c._remux = lambda path, fps: None
    c.nav_active('rota')
    for i in range(4):
        c._handle_frame(jpeg(bytes([i])), now=100.0 + i / 30)
    c.stop_recording('teste')
    [f] = [f for f in os.listdir(str(tmp_path)) if f.endswith('.mjpeg')]
    with open(os.path.join(str(tmp_path), f), 'rb') as fh:
        assert fh.read() == jpeg(b'\x00') + jpeg(b'\x02')


def test_remux_recebe_fps_real_da_gravacao(cam, monkeypatch):
    # câmera entregou 2 fps reais (20 frames em 10 s) — o remux tem que
    # carimbar 2.0, não o nominal, senão o vídeo fica acelerado
    remuxes = []
    cam._remux = lambda path, fps: remuxes.append(fps)
    t = [100.0]
    monkeypatch.setattr('camera_service.time.time', lambda: t[0])
    cam.nav_active('rota')                 # _rec_started = 100.0
    for i in range(20):
        cam._handle_frame(jpeg(bytes([i])), now=100.0 + i * 0.5)
    t[0] = 110.0
    cam.stop_recording('teste')
    assert remuxes == [pytest.approx(2.0)]


def test_nav_ended_respeita_debounce(cam):
    cam.nav_active('goal')
    cam._handle_frame(jpeg(), now=100.0)
    cam.nav_ended()
    # dentro do grace (8 s) continua gravando — não picota entre waypoints
    cam._handle_frame(jpeg(), now=105.0)
    assert cam.status()['recording'] is True


def test_nav_ended_para_depois_do_grace(cam, monkeypatch):
    monkeypatch.setattr('camera_service.time.time', lambda: 100.0)
    cam.nav_active('goal')
    cam._handle_frame(jpeg(), now=100.0)
    cam.nav_ended()   # _idle_since = 100.0
    cam._handle_frame(jpeg(), now=109.0)   # 9 s > grace de 8 s
    assert cam.status()['recording'] is False


def test_novo_goal_cancela_debounce(cam, monkeypatch):
    monkeypatch.setattr('camera_service.time.time', lambda: 100.0)
    cam.nav_active('rota')
    cam._handle_frame(jpeg(), now=100.0)
    cam.nav_ended()             # fim do waypoint 1
    cam.nav_active('rota')      # waypoint 2 aceito → cancela o debounce
    cam._handle_frame(jpeg(), now=120.0)
    assert cam.status()['recording'] is True


def test_teto_de_duracao(cam, monkeypatch):
    monkeypatch.setattr('camera_service.time.time', lambda: 100.0)
    cam.nav_active('goal')
    cam._handle_frame(jpeg(), now=100.0)
    cam._handle_frame(jpeg(), now=100.0 + 1801.0)
    assert cam.status()['recording'] is False


def test_stop_imediato_e_arquivo_vazio_removido(cam):
    cam.nav_active('goal')
    # ■ Parar antes de qualquer frame → arquivo vazio não fica de lixo
    cam.stop_recording('parar')
    assert cam.status()['recording'] is False
    assert rec_files(cam) == []


def test_nav_active_sem_camera_e_noop(tmp_path):
    c = CameraService(log_dir=str(tmp_path), autostart=False)   # available=False
    c.nav_active('rota')
    assert c.status()['recording'] is False


def test_wait_frame_entrega_e_respeita_seq(cam):
    cam._handle_frame(jpeg(b'a'), now=100.0)
    seq, frame = cam.wait_frame(0, timeout=0.1)
    assert frame == jpeg(b'a')
    # mesmo seq → sem frame novo → None após timeout
    assert cam.wait_frame(seq, timeout=0.05) is None
