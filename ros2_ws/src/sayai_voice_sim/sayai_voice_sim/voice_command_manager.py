#!/usr/bin/env python3

import os
import re
import string
import subprocess
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class CommandIntent:
    name: str
    service: str
    require_confirmation: bool
    canonical_phrases: frozenset


class VoiceCommandManager(Node):
    """Match recognized text commands to simulated robot task services."""

    FILLER_WORDS = {
        "a",
        "an",
        "can",
        "could",
        "me",
        "please",
        "the",
        "to",
        "you",
        "would",
    }

    KEYWORD_INTENTS = (
        ("stop", {"stop", "cancel", "halt"}),
        ("open_curtain", {"open"}),
        ("close_curtain", {"close"}),
        ("bring_bag", {"bag", "back"}),
        ("bring_bottle", {"bottle"}),
        ("move_forward", {"forward"}),
        ("move_backward", {"backward", "back"}),
        ("go_home", {"home"}),
    )

    def __init__(self):
        super().__init__("voice_command_manager")

        default_config = os.path.join(
            get_package_share_directory("sayai_voice_sim"),
            "config",
            "voice_commands.yaml",
        )

        self.declare_parameter("commands_file", default_config)
        self.declare_parameter("cooldown_seconds", 2.0)

        self._commands_file = (
            self.get_parameter("commands_file").get_parameter_value().string_value
        )
        self._cooldown_seconds = (
            self.get_parameter("cooldown_seconds").get_parameter_value().double_value
        )

        self._intents = self._load_intents(self._commands_file)
        self._task_clients: Dict[str, rclpy.client.Client] = {
            intent.service: self.create_client(Trigger, intent.service)
            for intent in self._intents.values()
        }
        self._last_accepted_key = ""
        self._last_accepted_time = self.get_clock().now()

        self._subscription = self.create_subscription(
            String,
            "/voice/transcript",
            self._transcript_callback,
            10,
        )

        self.get_logger().info(
            "VoiceCommandManager ready. Loaded %d intents from %s"
            % (len(self._intents), self._commands_file)
        )

    def _load_intents(self, commands_file: str) -> Dict[str, CommandIntent]:
        with open(commands_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        commands = data.get("commands", {})
        intents: Dict[str, CommandIntent] = {}

        for name, config in commands.items():
            phrases = config.get("phrases", [])
            canonical_phrases = frozenset(self._canonicalize(phrase) for phrase in phrases)
            service = config["service"]
            require_confirmation = bool(config.get("require_confirmation", False))
            intents[name] = CommandIntent(
                name=name,
                service=service,
                require_confirmation=require_confirmation,
                canonical_phrases=canonical_phrases,
            )

        return intents

    def _transcript_callback(self, msg: String) -> None:
        raw_text = msg.data
        normalized_text = self._normalize(raw_text)
        canonical_text = self._canonicalize(raw_text)

        self.get_logger().info("Raw text: '%s'" % raw_text)
        self.get_logger().info("Normalized text: '%s'" % normalized_text)

        incomplete_reason = self._get_incomplete_reason(canonical_text)
        if incomplete_reason:
            self.get_logger().warn("Rejected command: incomplete command (%s)" % incomplete_reason)
            return

        intent = self._match_intent(canonical_text)
        if intent is None:
            self.get_logger().warn("Rejected command: command not recognized")
            return

        if self._is_in_cooldown(canonical_text):
            self.get_logger().warn(
                "Rejected command: duplicate command within %.1f second cooldown"
                % self._cooldown_seconds
            )
            return

        self.get_logger().info("Detected intent: %s" % intent.name)
        self.get_logger().info(
            "Require confirmation: %s"
            % ("true" if intent.require_confirmation else "false")
        )
        subprocess.Popen(["spd-say", "OK"])
        self._call_task_service(intent)
        self._last_accepted_key = canonical_text
        self._last_accepted_time = self.get_clock().now()

    def _normalize(self, text: str) -> str:
        text = text.lower().strip()
        text = text.translate(str.maketrans("", "", string.punctuation))
        text = re.sub(r"\s+", " ", text)
        return text.strip()

    def _canonicalize(self, text: str) -> str:
        normalized = self._normalize(text)
        words = []

        for word in normalized.split():
            if word in self.FILLER_WORDS:
                continue
            if len(word) > 3 and word.endswith("s"):
                word = word[:-1]
            words.append(word)

        return " ".join(words)

    def _get_incomplete_reason(self, canonical_text: str) -> Optional[str]:
        words = set(canonical_text.split())

        if words.intersection({"bring", "fetch", "get"}) and not words.intersection(
            {"bag", "bottle"}
        ):
            return "bring/fetch/get command is missing the object"

        return None

    def _match_intent(self, canonical_text: str) -> Optional[CommandIntent]:
        words = set(canonical_text.split())

        for intent_name, keywords in self.KEYWORD_INTENTS:
            if intent_name not in self._intents:
                continue

            for keyword in keywords:
                if " " in keyword and keyword in canonical_text:
                    return self._intents[intent_name]
                if keyword in words:
                    return self._intents[intent_name]

        for intent in self._intents.values():
            if canonical_text in intent.canonical_phrases:
                return intent
        return None

    def _is_in_cooldown(self, canonical_text: str) -> bool:
        elapsed = self.get_clock().now() - self._last_accepted_time
        elapsed_seconds = elapsed.nanoseconds / 1_000_000_000.0
        return (
            canonical_text == self._last_accepted_key
            and elapsed_seconds < self._cooldown_seconds
        )

    def _call_task_service(self, intent: CommandIntent) -> None:
        client = self._task_clients[intent.service]

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available: %s" % intent.service)
            return

        self.get_logger().info("Calling service: %s" % intent.service)
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda response_future: self._handle_service_response(
                intent.service,
                response_future,
            )
        )

    def _handle_service_response(self, service_name: str, response_future) -> None:
        try:
            response = response_future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error("Service call failed: %s (%s)" % (service_name, exc))
            return

        self.get_logger().info(
            "Service response from %s: success=%s, message='%s'"
            % (service_name, response.success, response.message)
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandManager()

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
