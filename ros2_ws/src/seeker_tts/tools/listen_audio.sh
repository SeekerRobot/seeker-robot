#!/bin/bash
# Listen to the tts_node audio stream and play through host speakers.
#
# Run from the HOST machine (not inside the container).
# The container uses network_mode: host so port 8383 is on localhost.
#
# Usage:
#   ./listen_audio.sh
#   ./listen_audio.sh 8383        # custom port

PORT=${1:-8383}
TMP=/tmp/_seeker_audio.raw

if ! command -v ffplay &>/dev/null; then
  echo "ffplay not found — install it: sudo apt install ffmpeg"
  exit 1
fi

echo "Listening on http://localhost:${PORT}/audio_out  (Ctrl+C to stop)"

while true; do
  curl -s --max-time 30 -o "$TMP" "http://localhost:${PORT}/audio_out" \
    && [ -s "$TMP" ] \
    && ffplay -f s16le -ar 16000 -ac 1 -nodisp -autoexit "$TMP" 2>/dev/null
done
