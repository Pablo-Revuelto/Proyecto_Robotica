"""Speech-to-text via HuggingFace Whisper.

Lazily loads the `transformers` pipeline so other nodes do not pay the import
cost at startup. The default model `openai/whisper-small` is a good balance
between Spanish ASR accuracy and latency; override with the `model` parameter
to use `whisper-tiny`, `whisper-medium`, etc.

Input:  /chess/voice/audio   (std_msgs/Float32MultiArray, mono float32)
Output: /chess/voice/utterance (std_msgs/String)
"""

from __future__ import annotations

import re
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


_SR_RE = re.compile(r"sample_rate=(\d+)")


class WhisperAsrNode(Node):

    def __init__(self) -> None:
        super().__init__("chess_whisper_asr")
        self.declare_parameter("model",    "openai/whisper-small")
        self.declare_parameter("language", "spanish")
        self.declare_parameter("device",   "cpu")

        model = self.get_parameter("model").get_parameter_value().string_value
        self._lang   = self.get_parameter("language").get_parameter_value().string_value
        self._device = self.get_parameter("device").get_parameter_value().string_value

        self.get_logger().info(f"Loading Whisper model {model!r} on {self._device}...")
        self._asr = self._load_pipeline(model)
        self.get_logger().info("Whisper loaded.")

        self._pub = self.create_publisher(String, "/chess/voice/utterance", 5)
        self._sub = self.create_subscription(
            Float32MultiArray, "/chess/voice/audio", self._on_audio, 5)

    def _load_pipeline(self, model: str):
        from transformers import pipeline
        device_index = 0 if self._device == "cuda" else -1
        return pipeline(
            "automatic-speech-recognition", model=model, device=device_index,
            generate_kwargs={"language": self._lang, "task": "transcribe"},
        )

    def _on_audio(self, msg: Float32MultiArray) -> None:
        sr = self._extract_sr(msg)
        if sr is None:
            self.get_logger().warn("Audio buffer missing sample_rate label.")
            return
        audio = np.asarray(msg.data, dtype=np.float32)
        if audio.size == 0:
            return
        try:
            result = self._asr({"array": audio, "sampling_rate": sr})
        except Exception as exc:        # noqa: BLE001
            self.get_logger().error(f"ASR failed: {exc}")
            return
        text = (result.get("text") or "").strip()
        if not text:
            return
        self.get_logger().info(f"Transcribed: {text!r}")
        out = String()
        out.data = text
        self._pub.publish(out)

    @staticmethod
    def _extract_sr(msg: Float32MultiArray) -> Optional[int]:
        for dim in msg.layout.dim:
            m = _SR_RE.search(dim.label)
            if m:
                return int(m.group(1))
        return None


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = WhisperAsrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
