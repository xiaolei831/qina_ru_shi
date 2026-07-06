"""One-shot microphone test for the robot_ai voice assistant."""
from __future__ import annotations

import argparse
import os
import wave
from pathlib import Path

import dashscope
import numpy as np
import sounddevice as sd

from .hello_text_test import _default_params_file, _load_params
from .voice_chat_node import (
    _is_noise_transcript,
    _pick_device,
    _rms,
    asr,
    chat,
    omni_asr,
    record_utterance,
    speak,
)


def _save_wav(path: Path, pcm: bytes, sample_rate: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(sample_rate)
        f.writeframes(pcm)


def _rms_probe(in_idx: int, sample_rate: int, seconds: float) -> None:
    block = int(sample_rate * 0.03)
    count = max(1, int(seconds / 0.03))
    values = []
    with sd.InputStream(
        samplerate=sample_rate,
        blocksize=block,
        dtype="int16",
        channels=1,
        device=in_idx,
    ) as stream:
        for _ in range(count):
            data, _ = stream.read(block)
            values.append(_rms(data))
    values.sort()
    p95 = values[min(len(values) - 1, int(len(values) * 0.95))]
    print(
        f"[rms] min={values[0]} mean={int(np.mean(values))} "
        f"p95={p95} max={values[-1]}",
        flush=True,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Record one microphone utterance and optionally run ASR/LLM/TTS."
    )
    parser.add_argument(
        "--params-file",
        default=str(_default_params_file()),
        help="robot_ai params.yaml path.",
    )
    parser.add_argument("--rms-only", action="store_true", help="Only print mic RMS.")
    parser.add_argument("--asr-only", action="store_true", help="Skip LLM and TTS.")
    parser.add_argument("--no-tts", action="store_true", help="Skip TTS playback.")
    parser.add_argument("--rms-seconds", type=float, default=3.0)
    parser.add_argument(
        "--save-wav",
        default="/tmp/robot_ai_mic_test.wav",
        help="Path for captured PCM WAV. Use '' to disable.",
    )
    args = parser.parse_args(argv)

    params = _load_params(Path(args.params_file))
    sample_rate = int(params.get("sample_rate", 16000))
    input_device = params.get("input_device", "UAC")
    output_device = params.get("output_device", "UAC")

    in_idx = _pick_device(input_device, want_input=True, sample_rate=sample_rate)
    if in_idx is None:
        print(f"ERROR: input device matching {input_device!r} was not found.")
        return 3
    print(f"[audio] input_device={input_device!r} index={in_idx}", flush=True)

    if args.rms_only:
        _rms_probe(in_idx, sample_rate, args.rms_seconds)
        return 0

    api_key = params.get("dashscope_api_key") or os.environ.get("DASHSCOPE_API_KEY", "")
    if not api_key:
        print("ERROR: dashscope_api_key is empty and DASHSCOPE_API_KEY is not set.")
        return 2
    dashscope.api_key = api_key

    def log(msg: str) -> None:
        print(msg, flush=True)

    print("[listen] 请在提示后说一句话。", flush=True)
    pcm = record_utterance(
        in_idx,
        sample_rate,
        int(params.get("silence_threshold", 110)),
        int(params.get("speech_threshold", 170)),
        float(params.get("silence_end_s", 1.0)),
        float(params.get("pre_speech_pad_s", 0.3)),
        float(params.get("min_utterance_s", 0.3)),
        float(params.get("max_utterance_s", 12.0)),
        int(params.get("speech_start_blocks", 4)),
        int(params.get("min_recorded_rms", 120)),
        log,
    )
    if not pcm:
        print("[record] no valid speech captured", flush=True)
        return 4

    print(f"[record] bytes={len(pcm)} rms={_rms(np.frombuffer(pcm, dtype=np.int16))}")
    if args.save_wav:
        _save_wav(Path(args.save_wav), pcm, sample_rate)
        print(f"[record] saved={args.save_wav}", flush=True)

    asr_backend = params.get("asr_backend", "omni")
    if asr_backend == "omni":
        text = omni_asr(
            pcm,
            params.get("omni_asr_model", "qwen3-omni-flash-realtime"),
            params.get("omni_voice", "Cherry"),
            sample_rate,
            int(params.get("max_tokens", 512)),
            log,
        )
    else:
        text = asr(
            pcm,
            params.get("asr_model", "paraformer-realtime-v2"),
            sample_rate,
            log,
        )
    print(f"[asr] {text}", flush=True)
    if not text or _is_noise_transcript(text):
        return 5
    if args.asr_only:
        return 0

    reply = chat(
        [],
        text,
        "",
        params.get("llm_model", "qwen3-235b-a22b-instruct-2507"),
        int(params.get("max_tokens", 512)),
        log,
    )
    print(f"[reply] {reply}", flush=True)
    if args.no_tts:
        return 0

    tts_play_rate = int(params.get("tts_play_rate", 22050))
    out_idx = _pick_device(output_device, want_input=False, sample_rate=tts_play_rate)
    if out_idx is None:
        print(f"ERROR: output device matching {output_device!r} was not found.")
        return 6
    print(f"[audio] output_device={output_device!r} index={out_idx}", flush=True)
    speak(
        reply,
        out_idx,
        params.get("tts_model", "cosyvoice-v1"),
        params.get("tts_voice", "longxiaochun"),
        tts_play_rate,
        log,
    )
    print("[done]", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
