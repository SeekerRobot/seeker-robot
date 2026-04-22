"""Human-sounding audio cues for SeekObject state transitions.

All cues use pre-rendered WAVs via /audio_play_file for instant, API-free
playback. Dynamic Fish TTS (/audio_tts_input) is reserved for user voice
commands in command_node and only used here as a fallback if a WAV is missing.
"""

import os
import random

from std_msgs.msg import String


PHRASES: dict[str, list[str]] = {
    "search_start": [
        "Let's find the {thing}! Here we go!",
        "Okay! Miku is on the case! Looking for the {thing}!",
        "I'll find you, {thing}! Starting search now!",
    ],
    "object_spotted": [
        "I see it! I see it!",
        "There it is — found you!",
        "Ooh, I spot the {thing}! Moving in!",
        "Found something! Could it be...?",
    ],
    "object_reached": [
        "Yes! I made it!",
        "Got there! Wooo!",
        "Mission complete! The {thing} is right here!",
        "Ta-dah! We found it!",
    ],
    "search_failed": [
        "Aww... I couldn't find the {thing}",
        "Nooo, the {thing} was nowhere to be found...",
        "Search timed out. I tried my best!",
    ],
    "search_canceled": [
        "Oh okay, stopping now~",
        "Roger that! Pulling back!",
    ],
    "coverage_start": [
        "Hmm, let me try a wider sweep!",
        "Expanding my search area now!",
    ],
    "approach_start": [
        "Ooh, getting closer...!",
        "I see something — moving in!",
        "Almost there, almost there!",
    ],
    "searching": [
        "Still searching... la la la...",
    ],
    "periodic_dynamic": [
        "Where are you hiding, {thing}?",
        "Hmm, not here either...",
        "Keep looking, keep looking!",
        "I know you're out there somewhere, {thing}!",
        "Don't give up! Miku never gives up!",
        "Searching every corner...",
    ],
}

STATIC_WAVS: dict[str, str] = {
    "search_start":    "search_start.wav",
    "object_spotted":  "object_spotted.wav",
    "object_reached":  "object_reached.wav",
    "search_failed":   "search_failed.wav",
    "search_canceled": "search_canceled.wav",
    "searching":       "searching.wav",
}

PERIODIC_INTERVAL_MIN_S = 8.0
PERIODIC_INTERVAL_MAX_S = 15.0
MIN_GAP_AFTER_UTTERANCE_S = 3.0


class SearchVoice:
    def __init__(self, node, tts_pub, file_pub, sounds_dir: str):
        self._node = node
        self._tts_pub = tts_pub
        self._file_pub = file_pub
        self._sounds_dir = sounds_dir
        self._last_utterance_ns: int = 0
        self._next_periodic_ns: int = 0
        self._rng = random.Random()

    def say_event(self, event: str, thing: str | None = None) -> None:
        wav_name = STATIC_WAVS.get(event)
        if wav_name:
            wav_path = os.path.join(self._sounds_dir, wav_name)
            if os.path.isfile(wav_path):
                msg = String()
                msg.data = wav_path
                self._file_pub.publish(msg)
                self._mark_uttered()
                return
            self._node.get_logger().warn(
                f"search_voice: static WAV missing for '{event}' at {wav_path}; "
                f"falling back to dynamic TTS"
            )

        phrases = PHRASES.get(event)
        if not phrases:
            return
        text = self._rng.choice(phrases).format(thing=thing or "target")
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)
        self._mark_uttered()

    def maybe_periodic(self, now_ns: int, thing: str | None = None) -> bool:
        if self._next_periodic_ns == 0:
            self._schedule_next(now_ns)
            return False
        if now_ns < self._next_periodic_ns:
            return False
        if (now_ns - self._last_utterance_ns) * 1e-9 < MIN_GAP_AFTER_UTTERANCE_S:
            self._schedule_next(now_ns)
            return False

        self.say_event("searching")
        self._schedule_next(now_ns)
        return True

    def reset_periodic_timer(self, now_ns: int) -> None:
        self._schedule_next(now_ns)

    def _mark_uttered(self) -> None:
        try:
            self._last_utterance_ns = self._node.get_clock().now().nanoseconds
        except Exception:
            pass

    def _schedule_next(self, now_ns: int) -> None:
        interval_s = self._rng.uniform(
            PERIODIC_INTERVAL_MIN_S, PERIODIC_INTERVAL_MAX_S
        )
        self._next_periodic_ns = now_ns + int(interval_s * 1e9)
