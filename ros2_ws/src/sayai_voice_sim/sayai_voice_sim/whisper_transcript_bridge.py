#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from whisper_msgs.msg import Transcription


class WhisperTranscriptBridge(Node):
    """Republish Whisper transcription text as the voice manager input topic."""

    COMMAND_KEYWORDS = {
        "action",
        "back",
        "backward",
        "bag",
        "bottle",
        "bring",
        "close",
        "curtain",
        "execute",
        "fetch",
        "forward",
        "get",
        "go",
        "move",
        "open",
        "run",
        "start",
        "stop",
    }

    STOP_KEYWORDS = {
        "cancel",
        "halt",
        "stop",
    }

    def __init__(self):
        super().__init__("whisper_transcript_bridge")

        self.declare_parameter("input_topic", "/whisper/transcription")
        self.declare_parameter("output_topic", "/voice/transcript")
        self.declare_parameter("ignore_empty_text", True)
        self.declare_parameter("forward_only_command_like_text", True)
        self.declare_parameter("require_wake_word", True)
        self.declare_parameter(
            "wake_words",
            [
                "hey robot",
                "hay robot",
                "hai robot",
                "hi robot",
                "hello robot",
                "hey mobi",
                "hi mobi",
                "ok robot",
                "okay robot",
                "mobi",
                "moby",
                "mobile",
                "movie",
                "robot",
                "robo",
                "robert",
                "row bot",
            ],
        )
        self.declare_parameter("wake_window_sec", 30.0)

        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._ignore_empty_text = self.get_parameter("ignore_empty_text").value
        self._forward_only_command_like_text = self.get_parameter(
            "forward_only_command_like_text"
        ).value
        self._require_wake_word = self.get_parameter("require_wake_word").value
        self._wake_words = [
            self._normalize(wake_word)
            for wake_word in self.get_parameter("wake_words").value
        ]
        self._wake_window_sec = float(self.get_parameter("wake_window_sec").value)
        self._wake_active_until = self.get_clock().now()

        self._publisher = self.create_publisher(String, self._output_topic, 10)
        self._subscription = self.create_subscription(
            Transcription,
            self._input_topic,
            self._transcription_callback,
            10,
        )

        self.get_logger().info(
            "WhisperTranscriptBridge ready: %s -> %s"
            % (self._input_topic, self._output_topic)
        )

    def _transcription_callback(self, msg: Transcription) -> None:
        text = msg.text.strip()

        if self._ignore_empty_text and not text:
            self.get_logger().warn("Ignoring empty Whisper transcription")
            return

        text_to_publish = self._apply_wake_gate(text)
        if not text_to_publish:
            return

        if (
            self._forward_only_command_like_text
            and not self._looks_like_command(text_to_publish)
        ):
            self.get_logger().info(
                "Ignoring non-command Whisper text: '%s'" % text_to_publish
            )
            return

        out_msg = String()
        out_msg.data = text_to_publish
        self._publisher.publish(out_msg)
        self.get_logger().info("Republished Whisper text: '%s'" % text_to_publish)

    def _apply_wake_gate(self, text: str) -> str:
        if not self._require_wake_word:
            return text

        normalized = self._normalize(text)

        if self._contains_stop_keyword(normalized):
            self.get_logger().warn("Safety command bypassed wake gate: '%s'" % text)
            return text

        wake_word, remainder = self._extract_wake_word(normalized)
        if wake_word:
            self._wake_active_until = self.get_clock().now() + Duration(
                seconds=self._wake_window_sec
            )
            self.get_logger().info(
                "Wake word detected: '%s'. Listening for %.1f seconds"
                % (wake_word, self._wake_window_sec)
            )
            if remainder:
                self.get_logger().info("Wake command remainder: '%s'" % remainder)
                return remainder
            return ""

        wake_word, remainder = self._extract_misheard_wake_command(normalized)
        if wake_word:
            self._wake_active_until = self.get_clock().now() + Duration(
                seconds=self._wake_window_sec
            )
            self.get_logger().warn(
                "Possible wake word detected as '%s'. Treating remainder as command: '%s'"
                % (wake_word, remainder)
            )
            return remainder

        if self.get_clock().now() <= self._wake_active_until:
            return text

        if self._looks_like_command(text):
            self.get_logger().info(
                "Ignoring command-like text because wake word is required: '%s'" % text
            )
        else:
            self.get_logger().debug(
                "Ignoring non-command text because wake word is required: '%s'" % text
            )
        return ""

    def _looks_like_command(self, text: str) -> bool:
        words = set(self._normalize(text).split())
        return bool(words.intersection(self.COMMAND_KEYWORDS))

    def _contains_stop_keyword(self, normalized_text: str) -> bool:
        words = set(normalized_text.split())
        return bool(words.intersection(self.STOP_KEYWORDS))

    def _extract_wake_word(self, normalized_text: str):
        for wake_word in self._wake_words:
            if normalized_text == wake_word:
                return wake_word, ""
            if normalized_text.startswith(wake_word + " "):
                return wake_word, normalized_text[len(wake_word) :].strip()
            marker = " " + wake_word + " "
            if marker in normalized_text:
                before, _, after = normalized_text.partition(marker)
                del before
                return wake_word, after.strip()
            if normalized_text.endswith(" " + wake_word):
                return wake_word, ""
        return "", ""

    def _extract_misheard_wake_command(self, normalized_text: str):
        words = normalized_text.split()
        if len(words) < 2:
            return "", ""

        first_word = words[0]
        if first_word not in {"be", "bee", "b", "baby", "maybe"}:
            return "", ""

        remainder = " ".join(words[1:])
        if self._looks_like_command(remainder):
            return first_word, remainder
        return "", ""

    def _normalize(self, text: str) -> str:
        punctuation = ".,!?;:-_()[]{}\"'"
        cleaned_words = [word.strip(punctuation).lower() for word in text.split()]
        return " ".join(word for word in cleaned_words if word)


def main(args=None):
    rclpy.init(args=args)
    node = WhisperTranscriptBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
