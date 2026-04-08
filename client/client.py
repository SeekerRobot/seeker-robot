import requests
import numpy as np
from faster_whisper import WhisperModel

ESP32_IP = "http://192.168.8.50:81"
URL = f"{ESP32_IP}/audio"

SAMPLE_RATE = 16000
WINDOW_SECONDS = 2
BUFFER_THRESHOLD = SAMPLE_RATE * WINDOW_SECONDS

print("Loading Whisper model...")
model = WhisperModel("tiny.en", device = "cpu", compute_type = "int8")

def transcribe():
    audio_buffer = np.array([], dtype = np.float32)
    
    print(f"Connecting to {URL}")
    try:
        with requests.get(URL, stream = True, timeout = 10) as r:
            r.raise_for_status()
            print("Transcription started. Speak into the mic.")
            
            for chunk in r.iter_content(chunk_size = 2048):
                if not chunk:
                    break
                
                pcm_data = np.frombuffer(chunk, dtype = np.int16)
                audio_float = pcm_data.astype(np.float32) / 32768.0
                
                audio_buffer = np.append(audio_buffer, audio_float)
                
                if len(audio_buffer) >= BUFFER_THRESHOLD:
                    segments, _ = model.transcribe(audio_buffer, beam_size = 5)
                    
                    for segment in segments:
                        if segment.text.strip():
                            print(f"[Transcript]: {segment.text.strip()}")
                            
                    audio_buffer = np.array([], dtype = np.float32)
    
    except requests.exceptions.RequestException as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    transcribe()
