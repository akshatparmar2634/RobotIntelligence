# asr_node.py

import whisper
import sounddevice as sd
import numpy as np
import queue

# Set up audio capture
samplerate = 16000  # Whisper default
duration = 10        # seconds
channels = 1
model_size = "tiny"  # You can use "base", "small", etc.

q = queue.Queue()
model = whisper.load_model(model_size)

def audio_callback(indata, frames, time, status):
    if status:
        print(f"Audio Status: {status}")
    q.put(indata.copy())

def record_audio():
    print("🎙️ Listening... Speak now!")
    audio_frames = []

    with sd.InputStream(callback=audio_callback, channels=channels, samplerate=samplerate):
        for _ in range(0, int(samplerate / 1024 * duration)):
            audio_frames.append(q.get())
    
    audio = np.concatenate(audio_frames, axis=0)
    return audio.flatten()

def transcribe(audio):
    print("🔍 Transcribing...")
    result = model.transcribe(audio, fp16=False)
    return result['text']

if __name__ == "__main__":
    audio = record_audio()
    text = transcribe(audio)
    print("📝 You said:", text)
