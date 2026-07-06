"""Text-input smoke test for the robot_ai voice assistant."""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import dashscope
import yaml

from .voice_chat_node import _pick_device, chat, speak


def _default_params_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_ai")) / "config" / "params.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "params.yaml"


def _load_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("robot_ai_node", {}).get("ros__parameters", {})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Send one text prompt through robot_ai LLM/TTS."
    )
    parser.add_argument("text", nargs="?", default="你好", help="Text to send.")
    parser.add_argument(
        "--params-file",
        default=str(_default_params_file()),
        help="robot_ai params.yaml path.",
    )
    parser.add_argument("--no-tts", action="store_true", help="Skip TTS playback.")
    args = parser.parse_args(argv)

    params = _load_params(Path(args.params_file))
    api_key = params.get("dashscope_api_key") or os.environ.get("DASHSCOPE_API_KEY", "")
    if not api_key:
        print("ERROR: dashscope_api_key is empty and DASHSCOPE_API_KEY is not set.")
        return 2
    dashscope.api_key = api_key

    llm_model = params.get("llm_model", "qwen3-235b-a22b-instruct-2507")
    max_tokens = int(params.get("max_tokens", 512))
    output_device = params.get("output_device", "UAC")
    tts_model = params.get("tts_model", "cosyvoice-v1")
    tts_voice = params.get("tts_voice", "longxiaochun")
    tts_play_rate = int(params.get("tts_play_rate", 22050))

    def log(msg: str) -> None:
        print(msg, flush=True)

    print(f"[text] {args.text}", flush=True)
    reply = chat([], args.text, "", llm_model, max_tokens, log)
    print(f"[reply] {reply}", flush=True)

    if args.no_tts:
        return 0

    out_idx = _pick_device(output_device, want_input=False, sample_rate=tts_play_rate)
    if out_idx is None:
        print(f"ERROR: output device matching {output_device!r} was not found.")
        return 3
    print(f"[audio] output_device={output_device!r} index={out_idx}", flush=True)
    speak(reply, out_idx, tts_model, tts_voice, tts_play_rate, log)
    print("[done]", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
