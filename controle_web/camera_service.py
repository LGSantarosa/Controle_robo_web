# Câmera USB (Logitech C922) — POV do robô: live view na GUI + gravação do percurso.
#
# Um único processo ffmpeg é dono do /dev/video0 e copia o MJPEG que a câmera
# já entrega comprimido (-c:v copy) — a Pi não re-encoda nada, só faz memcpy.
# Uma thread separa os frames JPEG do pipe e:
#   * guarda sempre o último frame  → rota /camera/stream (live view na GUI);
#   * quando "gravando", appenda cada frame num .mjpeg em logs/pov/ na Pi.
#
# Gravação segue a navegação (mesma direção da run):
#   * nav_active()  — chamado no play (start_waypoints / nav_goal) e a cada
#     goal aceito pelo Nav2 (hook do nav_metrics). Inicia se estava parado.
#   * nav_ended()   — goal terminou. NÃO para na hora: arma um debounce
#     (idle_grace_s) pra não picotar o vídeo entre waypoints da mesma rota.
#   * stop_recording() — parada imediata (■ Parar na GUI, shutdown).
# Teto de max_rec_s corta gravação esquecida (ex.: goal via porta usa
# navigate_through_poses, que o nav_metrics não rastreia → sem nav_ended).
#
# Controle manual pela GUI (ex.: teste de bateria com live view mas sem encher
# o SD): set_auto_record(False) faz os hooks de navegação virarem no-op;
# start_manual() grava até ⏹/teto/câmera cair — nav_ended não encerra manual.
#
# Ao fechar um arquivo, um remux ffmpeg em background embrulha o MJPEG cru em
# .mkv com timestamps (assistível em qualquer player); o cru é apagado se der
# certo. Sem câmera plugada o serviço fica quieto tentando de novo a cada 5 s
# (mesma filosofia do retry do LD06 no launch.sh) — nada derruba o app.

import logging
import os
import shutil
import subprocess
import threading
import time

log = logging.getLogger(__name__)

_SOI = b'\xff\xd8'   # início de JPEG
_EOI = b'\xff\xd9'   # fim de JPEG


class MjpegSplitter:
    """Separa frames JPEG completos de um stream MJPEG concatenado.

    Nos dados comprimidos do scan todo 0xFF é byte-stuffed (0xFF00), então
    procurar SOI/EOI direto no stream é seguro pra MJPEG de webcam (sem
    thumbnail EXIF embutida).
    """

    def __init__(self, max_buffer: int = 8 * 1024 * 1024):
        self._buf = bytearray()
        self._max = max_buffer

    def feed(self, chunk: bytes) -> list:
        self._buf.extend(chunk)
        frames = []
        while True:
            start = self._buf.find(_SOI)
            if start < 0:
                # lixo sem SOI — mantém só o último byte (pode ser um 0xFF
                # de um SOI partido entre chunks)
                del self._buf[:max(0, len(self._buf) - 1)]
                break
            end = self._buf.find(_EOI, start + 2)
            if end < 0:
                if start > 0:
                    del self._buf[:start]
                if len(self._buf) > self._max:
                    # frame impossível de tão grande — descarta e ressincroniza
                    self._buf.clear()
                break
            frames.append(bytes(self._buf[start:end + 2]))
            del self._buf[:end + 2]
        return frames


class CameraService:
    """Dona da câmera USB: live view + gravação do percurso (POV)."""

    def __init__(self, log_dir: str, socketio=None, device: str = '/dev/video0',
                 width: int = 1280, height: int = 720, fps: int = 30,
                 rec_fps: int = 15, idle_grace_s: float = 8.0,
                 max_rec_s: float = 1800.0, autostart: bool = True):
        self._log_dir = log_dir
        self._sock = socketio
        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        # Gravação em disco decimada (frame sim, frame não a 30→15): a C922
        # manda MJPEG ~165 KB/frame — 30 fps daria ~4,7 GB por run de 30 min.
        # O live view segue no fps cheio; só a escrita pula frames.
        self._rec_stride = max(1, round(fps / rec_fps))
        self._rec_fps = fps / self._rec_stride
        self._idle_grace_s = idle_grace_s
        self._max_rec_s = max_rec_s
        os.makedirs(log_dir, exist_ok=True)

        # Último frame pro live view (protegido por condition — clientes do
        # /camera/stream esperam nela em vez de fazer polling)
        self._cond = threading.Condition()
        self._last_frame: bytes = b''
        self._seq = 0

        # Estado de gravação (protegido por _rec_lock)
        self._rec_lock = threading.Lock()
        self._rec_file = None
        self._rec_path = ''
        self._rec_started = 0.0
        self._rec_frames = 0
        self._rec_manual = False  # gravação manual só para no ⏹/teto/câmera
        self._idle_since = None   # ts do último nav_ended sem novo goal
        self._auto_record = True  # hooks de navegação disparam gravação?

        self._available = False
        self._running = True
        self._proc = None
        self._thread = None
        if autostart:
            self._thread = threading.Thread(
                target=self._capture_loop, daemon=True, name='camera_capture')
            self._thread.start()

    # ---------------- API pública ----------------

    @property
    def available(self) -> bool:
        return self._available

    def status(self) -> dict:
        with self._rec_lock:
            return {
                'available': self._available,
                'recording': self._rec_file is not None,
                'manual': self._rec_manual,
                'auto_record': self._auto_record,
                'file': os.path.basename(self._rec_path) if self._rec_file else '',
            }

    def wait_frame(self, last_seq: int, timeout: float = 2.0):
        """Bloqueia até haver frame mais novo que last_seq. → (seq, jpeg) | None."""
        with self._cond:
            if self._seq <= last_seq:
                self._cond.wait(timeout)
            if self._seq <= last_seq or not self._last_frame:
                return None
            return self._seq, self._last_frame

    def nav_active(self, label: str = 'nav'):
        """Navegação (re)começou — inicia gravação se parada, cancela debounce."""
        with self._rec_lock:
            if not self._auto_record:
                return
            self._idle_since = None
            if self._rec_file is not None:
                return
            if not self._available:
                log.info(f"[Camera] nav_active({label}) sem câmera — gravação pulada")
                return
            self._start_recording_locked(label)
        self._emit_status()

    def nav_ended(self):
        """Goal terminou — arma o debounce; para de vez se nada novo chegar."""
        with self._rec_lock:
            if (self._rec_file is not None and not self._rec_manual
                    and self._idle_since is None):
                self._idle_since = time.time()

    def set_auto_record(self, on: bool):
        """Liga/desliga a gravação automática nos hooks de navegação.

        Não mexe numa gravação em andamento (auto some pelo debounce/■;
        manual só pelo ⏹)."""
        with self._rec_lock:
            self._auto_record = bool(on)
        self._emit_status()

    def start_manual(self):
        """⏺ da GUI — grava até o ⏹/teto/câmera cair (nav_ended não encerra).

        Se já havia gravação automática rolando, ela vira manual (adota) —
        não picota o arquivo."""
        with self._rec_lock:
            if self._rec_file is not None:
                self._rec_manual = True
                self._idle_since = None
            elif self._available:
                self._start_recording_locked('manual', manual=True)
            else:
                log.info("[Camera] ⏺ manual sem câmera — ignorado")
        self._emit_status()

    def stop_recording(self, reason: str = 'stop'):
        """Parada imediata (■ Parar, shutdown, câmera caiu)."""
        with self._rec_lock:
            self._stop_recording_locked(reason)
        self._emit_status()

    def shutdown(self):
        self._running = False
        self.stop_recording('shutdown')
        proc = self._proc
        if proc is not None:
            try:
                proc.terminate()
            except Exception:
                pass
        # Acorda clientes do stream pra encerrarem
        with self._cond:
            self._cond.notify_all()

    # ---------------- Gravação (interno) ----------------

    def _start_recording_locked(self, label: str, manual: bool = False):
        ts = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._rec_path = os.path.join(self._log_dir, f'pov_{ts}_{label}.mjpeg')
        try:
            self._rec_file = open(self._rec_path, 'wb')
        except OSError as e:
            log.warning(f"[Camera] não abriu {self._rec_path}: {e}")
            self._rec_file = None
            return
        self._rec_started = time.time()
        self._rec_frames = 0
        self._rec_seen = 0
        self._rec_manual = manual
        self._idle_since = None
        log.info(f"[Camera] ● gravando POV → {os.path.basename(self._rec_path)}")

    def _stop_recording_locked(self, reason: str):
        if self._rec_file is None:
            return
        f, path, frames = self._rec_file, self._rec_path, self._rec_frames
        self._rec_file = None
        self._rec_manual = False
        self._idle_since = None
        try:
            f.close()
        except Exception:
            pass
        dur = time.time() - self._rec_started
        log.info(f"[Camera] ■ POV parado ({reason}): {frames} frames, "
                 f"{dur:.0f}s → {os.path.basename(path)}")
        if frames == 0:
            # run cancelada antes do 1º frame — não deixa lixo vazio
            try:
                os.unlink(path)
            except OSError:
                pass
            return
        # fps REAL da gravação, não o nominal: com pouca luz a C922 entrega
        # menos que 30 fps (exposição automática) — carimbar 15 fixo no remux
        # deixava o vídeo acelerado (~1,5x nas runs de campo).
        real_fps = frames / dur if dur > 0.5 else self._rec_fps
        threading.Thread(target=self._remux, args=(path, real_fps), daemon=True,
                         name='camera_remux').start()

    def _remux(self, raw_path: str, fps: float):
        """Embrulha o MJPEG cru em .mkv (cópia, sem re-encode). Falhou → mantém o cru."""
        if not shutil.which('ffmpeg'):
            log.warning("[Camera] ffmpeg ausente — mantendo .mjpeg cru")
            return
        out = raw_path.rsplit('.', 1)[0] + '.mkv'
        try:
            rc = subprocess.run(
                ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                 '-f', 'mjpeg', '-framerate', f'{fps:.3f}', '-i', raw_path,
                 '-c:v', 'copy', out],
                timeout=300).returncode
        except Exception as e:
            log.warning(f"[Camera] remux falhou ({e}) — mantendo {raw_path}")
            return
        if rc == 0:
            os.unlink(raw_path)
            log.info(f"[Camera] POV pronto: {out}")
        else:
            log.warning(f"[Camera] remux rc={rc} — mantendo {raw_path}")

    def _handle_frame(self, jpeg: bytes, now: float = None):
        """Um frame chegou: publica pro live view e grava/faz housekeeping."""
        if now is None:
            now = time.time()
        with self._cond:
            self._last_frame = jpeg
            self._seq += 1
            self._cond.notify_all()
        stopped = False
        with self._rec_lock:
            if self._rec_file is not None:
                if self._idle_since is not None and now - self._idle_since > self._idle_grace_s:
                    self._stop_recording_locked('nav idle')
                    stopped = True
                elif now - self._rec_started > self._max_rec_s:
                    self._stop_recording_locked('teto de duração')
                    stopped = True
                else:
                    try:
                        if self._rec_seen % self._rec_stride == 0:
                            self._rec_file.write(jpeg)
                            self._rec_frames += 1
                        self._rec_seen += 1
                    except OSError as e:
                        log.warning(f"[Camera] erro gravando ({e}) — parando")
                        self._stop_recording_locked('erro de escrita')
                        stopped = True
        if stopped:
            self._emit_status()

    # ---------------- Captura (interno) ----------------

    def _capture_loop(self):
        if not shutil.which('ffmpeg'):
            log.warning("[Camera] ffmpeg não instalado (sudo apt install ffmpeg) "
                        "— POV desabilitado")
            return
        first = True
        while self._running:
            if not os.path.exists(self._device):
                if first:
                    log.info(f"[Camera] {self._device} ausente — POV desabilitado "
                             f"(replug ativa sozinho)")
                    first = False
                self._set_available(False)
                time.sleep(5.0)
                continue
            first = False
            try:
                self._run_ffmpeg()
            except Exception as e:
                log.warning(f"[Camera] captura caiu: {e}")
            self._set_available(False)
            self.stop_recording('câmera caiu')
            if self._running:
                time.sleep(5.0)

    def _run_ffmpeg(self):
        cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error',
            '-f', 'v4l2', '-input_format', 'mjpeg',
            '-video_size', f'{self._width}x{self._height}',
            '-framerate', str(self._fps),
            '-i', self._device,
            '-c:v', 'copy', '-f', 'mjpeg', 'pipe:1',
        ]
        log.info(f"[Camera] abrindo {self._device} "
                 f"({self._width}x{self._height}@{self._fps}, MJPEG copy)")
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            bufsize=0)
        splitter = MjpegSplitter()
        got_frame = False
        try:
            while self._running:
                chunk = self._proc.stdout.read(65536)
                if not chunk:
                    break
                for jpeg in splitter.feed(chunk):
                    if not got_frame:
                        got_frame = True
                        self._set_available(True)
                        log.info("[Camera] C922 viva — live view disponível")
                    self._handle_frame(jpeg)
        finally:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=3)
            except Exception:
                pass
            self._proc = None

    def _set_available(self, val: bool):
        if val != self._available:
            self._available = val
            self._emit_status()

    def _emit_status(self):
        if self._sock is None:
            return
        try:
            self._sock.emit('camera_status', self.status(), namespace='/')
        except Exception:
            pass
