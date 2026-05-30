"""Capture microphone audio and publish utterances as Float32 buffers.

We use `sounddevice` for cross-platform mic access, and a simple energy-based
VAD: an utterance starts when the rolling RMS crosses `start_rms` and ends
after `silence_ms` of below-threshold audio. Each completed utterance is
published as a `std_msgs/Float32MultiArray` on `/chess/voice/audio` along with
its sample rate as the first dimension label.

This node is intentionally decoupled from ASR (`whisper_asr_node`) so it can
be replaced or mocked (file-based replay, push-to-talk) without changing the
rest of the pipeline.
"""

from __future__ import annotations

import collections
import threading
from typing import Deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class AudioCapture(Node):

    def __init__(self) -> None:
        super().__init__("chess_audio_capture")
        self.declare_parameter("sample_rate",      16000)
        self.declare_parameter("block_ms",         30)
        self.declare_parameter("start_rms",        0.02)
        self.declare_parameter("end_rms",          0.012)
        self.declare_parameter("silence_ms",       700)
        self.declare_parameter("max_utterance_s",  12.0)
        self.declare_parameter("device",           "")

        self._sr      = int(self.get_parameter("sample_rate").value)
        self._block   = int(self._sr * self.get_parameter("block_ms").value / 1000)
        self._start   = float(self.get_parameter("start_rms").value)
        self._end     = float(self.get_parameter("end_rms").value)
        self._silence = int(self.get_parameter("silence_ms").value
                            / self.get_parameter("block_ms").value)
        self._max     = int(self.get_parameter("max_utterance_s").value
                            * self._sr / self._block)
        device = self.get_parameter("device").get_parameter_value().string_value or None

        self._pub = self.create_publisher(Float32MultiArray, "/chess/voice/audio", 1)
        self._buffer: Deque[np.ndarray] = collections.deque()
        self._silent_blocks = 0
        self._in_utterance = False
        self._running = False
        self._reader: threading.Thread | None = None

        # Blocking read in a dedicated thread (NOT a PortAudio callback): under
        # WSL2 the callback path spawns a real-time-scheduled thread that fails
        # with `paTimedOut [-9987]`; a blocking InputStream.read() avoids it.
        # `device` should be "pulse" on WSL2 (the only backend wired to the WSLg
        # microphone); other ALSA devices open but capture silence.
        try:
            import sounddevice as sd
            self._stream = sd.InputStream(
                samplerate=self._sr, channels=1, blocksize=self._block,
                dtype="float32", device=device)
            self._stream.start()
        except Exception as exc:        # noqa: BLE001 -- top-level user feedback
            self.get_logger().error(
                f"Failed to open microphone (device={device!r}): {exc}")
            raise

        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f"Mic streaming @ {self._sr} Hz (device={device!r}, blocking read), "
            f"start_rms={self._start}")

    def _read_loop(self) -> None:
        while self._running:
            try:
                indata, _overflowed = self._stream.read(self._block)
            except Exception as exc:        # noqa: BLE001
                self.get_logger().error(f"Mic read error: {exc}")
                break
            self._process(indata[:, 0].copy())

    def stop(self) -> None:
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:               # noqa: BLE001
            pass

    def _process(self, block) -> None:
        rms = float(np.sqrt(np.mean(block * block) + 1e-12))

        if not self._in_utterance:
            if rms > self._start:
                self._in_utterance = True
                self._buffer.clear()
                self._silent_blocks = 0
                self._buffer.append(block)
            return

        self._buffer.append(block)
        self._silent_blocks = self._silent_blocks + 1 if rms < self._end else 0
        too_long = len(self._buffer) >= self._max
        if self._silent_blocks >= self._silence or too_long:
            self._emit()
            self._in_utterance = False

    def _emit(self) -> None:
        audio = np.concatenate(list(self._buffer)).astype(np.float32)
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = f"sample_rate={self._sr}"
        dim.size = audio.size
        dim.stride = audio.size
        msg.layout.dim = [dim]
        msg.data = audio.tolist()
        self._pub.publish(msg)
        self.get_logger().info(f"Published utterance, {audio.size/self._sr:.2f}s")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    try:
        node = AudioCapture()
    except Exception:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
