"""
Ponte ROS2 → WebSocket para a câmera RGB do robô.

Subscreve `/camera/image` (sensor_msgs/Image), converte cada frame em JPEG
(qualidade ajustável) e emite via Socket.IO no evento `camera_frame`.
Limita a taxa de envio pra não saturar a banda do WebSocket — a câmera
publica a 15 Hz, mandamos no máximo 5 Hz pra UI.

Roda em thread daemon com executor próprio, igual o MapBridge.
"""
from __future__ import annotations

import base64
import logging
import os
import threading
import time
from io import BytesIO
from typing import Optional

import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Image


log = logging.getLogger(__name__)


def _img_msg_to_jpeg_b64(msg: Image, quality: int = 60) -> Optional[str]:
    """Converte sensor_msgs/Image (rgb8) em JPEG base64."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        log.debug(f"[CameraBridge] encoding inesperado: {msg.encoding}")
        return None
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if arr.size != msg.height * msg.width * 3:
        log.warning("[CameraBridge] tamanho do buffer não bate")
        return None
    arr = arr.reshape((msg.height, msg.width, 3))
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]  # BGR → RGB
    pil = PILImage.fromarray(arr, mode='RGB')
    buf = BytesIO()
    pil.save(buf, format='JPEG', quality=quality, optimize=False)
    return base64.b64encode(buf.getvalue()).decode('ascii')


class CameraBridge:
    """Captura /camera/image e faz stream JPEG via Socket.IO."""

    PUBLISH_HZ = 5.0   # taxa máxima de emissão pro web

    def __init__(self, socketio, topic: str = '/camera/image'):
        self._sock = socketio
        self._topic = topic
        self._period = 1.0 / self.PUBLISH_HZ
        self._last_emit_ts: float = 0.0

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node('web_camera_bridge')

        if os.environ.get('ROBOT_SIM', 'false').lower() == 'true':
            from rclpy.parameter import Parameter
            self._node.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])

        self._node.create_subscription(
            Image, topic, self._on_image, 10
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='camera_bridge_spin'
        )
        self._spin_thread.start()

        log.info(f"[CameraBridge] inicializado (topic={topic})")

    def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                log.debug(f"[CameraBridge] spin: {e}")

    def _on_image(self, msg: Image):
        # Throttle pra ~5 Hz mesmo se ROS publicar a 15 Hz
        now = time.time()
        if now - self._last_emit_ts < self._period:
            return
        try:
            jpeg_b64 = _img_msg_to_jpeg_b64(msg, quality=60)
            if jpeg_b64 is None:
                return
            self._sock.emit(
                'camera_frame',
                {'jpeg_b64': jpeg_b64, 'w': msg.width, 'h': msg.height, 'ts': now},
                namespace='/',
            )
            self._last_emit_ts = now
        except Exception as e:
            log.debug(f"[CameraBridge] erro emitindo frame: {e}")

    def shutdown(self):
        self._running = False
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:
            pass
