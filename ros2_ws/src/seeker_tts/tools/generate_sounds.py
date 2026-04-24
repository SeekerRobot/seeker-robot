#!/usr/bin/env python3
"""Pre-render static WAV cues for SeekObject state transitions.

Uses the Fish Audio API (same endpoint as seeker_tts/tts_node.py) and writes
one WAV per cue into ros2_ws/src/seeker_tts/sounds/.

API keys are loaded automatically from docker/.env (relative to the repo
root). You can also override them with environment variables:
  FISH_API_KEY, FISH_REFERENCE_ID.

Run once; re-run only when canonical phrases change.

  python3 ros2_ws/src/seeker_tts/tools/generate_sounds.py
  python3 ros2_ws/src/seeker_tts/tools/generate_sounds.py --only searching
"""

import argparse
import os
import sys
import wave
from pathlib import Path

import requests


FISH_TTS_URL = "https://api.fish.audio/v1/tts"
SAMPLE_RATE = 16000
DEFAULT_MODEL = "s2-pro"

# Canonical text rendered for each pre-cached WAV.
# Voice persona: Hatsune Miku — energetic, upbeat, expressive.
STATIC_PHRASES: dict[str, str] = {
    "search_start":    "Let's find it! Here we go!",
    "object_spotted":  "I see it! I see it!",
    "object_reached":  "Yes! I made it!",
    "search_failed":   "Aww... I couldn't find it",
    "search_canceled": "Oh okay, stopping now!",
    "searching":       "Still searching... la la la...",
}

# Path: tools/ -> seeker_tts/ -> src/ -> ros2_ws/ -> repo_root/
_REPO_ROOT = Path(__file__).resolve().parents[4]
_DOCKER_ENV = _REPO_ROOT / "docker" / ".env"


def _load_dotenv(env_path: Path) -> dict[str, str]:
    """Minimal .env parser — no dependency on python-dotenv."""
    result: dict[str, str] = {}
    if not env_path.is_file():
        return result
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, val = line.partition("=")
        key = key.strip()
        val = val.strip().strip("'\"")
        if key:
            result[key] = val
    return result


def get_api_creds() -> tuple[str, str]:
    dotenv = _load_dotenv(_DOCKER_ENV)
    api_key = (
        os.environ.get("FISH_API_KEY", "").strip()
        or dotenv.get("FISH_API_KEY", "").strip()
    )
    ref_id = (
        os.environ.get("FISH_REFERENCE_ID", "").strip()
        or dotenv.get("FISH_REFERENCE_ID", "").strip()
    )
    return api_key, ref_id


def fish_tts(text: str, api_key: str, ref_id: str, model: str) -> bytes:
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "model": model,
    }
    body = {
        "text": text,
        "format": "pcm",
        "sample_rate": SAMPLE_RATE,
    }
    if ref_id:
        body["reference_id"] = ref_id

    resp = requests.post(FISH_TTS_URL, headers=headers, json=body,
                         stream=True, timeout=60)
    resp.raise_for_status()
    return b"".join(resp.iter_content(chunk_size=4096))


def write_wav(path: Path, pcm: bytes) -> None:
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--out-dir", type=Path,
        default=Path(__file__).resolve().parent.parent / "sounds",
        help="Destination directory for generated .wav files",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--only", nargs="*",
        help="Only regenerate the given event names (default: all)",
    )
    args = parser.parse_args()

    api_key, ref_id = get_api_creds()
    if not api_key:
        print(
            f"ERROR: FISH_API_KEY not set in environment or {_DOCKER_ENV}",
            file=sys.stderr,
        )
        return 1

    source = "docker/.env" if not os.environ.get("FISH_API_KEY") else "env"
    ref_note = f", ref_id={ref_id[:8]}..." if ref_id else " (no reference_id)"
    print(f"Using API key from {source}{ref_note}")

    args.out_dir.mkdir(parents=True, exist_ok=True)

    events = list(STATIC_PHRASES.keys())
    if args.only:
        unknown = [e for e in args.only if e not in STATIC_PHRASES]
        if unknown:
            print(f"ERROR: unknown events: {unknown}", file=sys.stderr)
            print(f"Available: {list(STATIC_PHRASES.keys())}", file=sys.stderr)
            return 1
        events = args.only

    for event in events:
        text = STATIC_PHRASES[event]
        out_path = args.out_dir / f"{event}.wav"
        print(f"[{event}] {text!r} → {out_path.name}")
        try:
            pcm = fish_tts(text, api_key, ref_id, args.model)
        except requests.RequestException as e:
            print(f"  FAILED: {e}", file=sys.stderr)
            return 2
        write_wav(out_path, pcm)
        duration_s = len(pcm) / (SAMPLE_RATE * 2)
        print(f"  wrote {len(pcm)} bytes ({duration_s:.2f}s)")

    print(f"\nDone. {len(events)} WAV(s) in {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
