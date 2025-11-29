import os
from io import BytesIO
from typing import IO, Optional

from dotenv import load_dotenv  # type: ignore[import]
from elevenlabs import VoiceSettings  # type: ignore[import]
from elevenlabs.client import ElevenLabs  # type: ignore[import]
import os
import sys
from contextlib import contextmanager

@contextmanager
def suppress_alsa_errors():
    # Redirect stderr to /dev/null
    stderr_fileno = sys.stderr.fileno()
    with open(os.devnull, 'w') as devnull:
        old_stderr = os.dup(stderr_fileno)
        os.dup2(devnull.fileno(), stderr_fileno)
        try:
            yield
        finally:
            os.dup2(old_stderr, stderr_fileno)

try:  # Optional playback dependency
    from pydub import AudioSegment, playback  # type: ignore[import]
    from pydub.playback import play as pydub_play  # type: ignore[import]
    playback._play_with_ffplay = True
    
except ImportError:  # pragma: no cover - optional dependency
    AudioSegment = None  # type: ignore[assignment]
    pydub_play = None  # type: ignore[assignment]

load_dotenv()

ELEVENLABS_API_KEY = os.getenv("ELEVENLABS_API_KEY")
if not ELEVENLABS_API_KEY:
    raise ValueError("ELEVENLABS_API_KEY environment variable not set")

client = ElevenLabs(api_key=ELEVENLABS_API_KEY)

DEFAULT_VOICE_ID = os.getenv("ELEVENLABS_VOICE_ID", "pNInz6obpgDQGcFmaJgB")
DEFAULT_MODEL_ID = os.getenv("ELEVENLABS_TTS_MODEL", "eleven_multilingual_v2")


def text_to_speech_stream(
    text: str,
    *,
    voice_id: str = DEFAULT_VOICE_ID,
    model_id: str = DEFAULT_MODEL_ID,
    optimize_streaming_latency: str = "0",
    output_format: str = "mp3_22050_32",
    voice_settings: Optional[VoiceSettings] = None,
) -> IO[bytes]:
    """Convert text to speech and return the audio as an in-memory byte stream."""
    response = client.text_to_speech.convert(
        voice_id=voice_id,
        optimize_streaming_latency=optimize_streaming_latency,
        output_format=output_format,
        text=text,
        model_id=model_id,
    )

    audio_stream = BytesIO()
    for chunk in response:
        if chunk:
            audio_stream.write(chunk)

    audio_stream.seek(0)
    return audio_stream


def text_to_speech_bytes(**kwargs) -> bytes:
    """Return synthesized speech as raw bytes for the given parameters."""
    stream = text_to_speech_stream(**kwargs)
    try:
        return stream.read()
    finally:
        stream.close()


def speak(text: str, **kwargs) -> None:
    """Synthesize text and play the generated audio directly from memory."""
    if AudioSegment is None or pydub_play is None:
        raise RuntimeError(
            "pydub (and an audio backend such as simpleaudio or pyaudio) is required for playback. "
            "Install with 'pip install pydub simpleaudio'."
        )

    stream = text_to_speech_stream(text, **kwargs)
    try:
        segment = AudioSegment.from_file(stream, format="mp3")
        with suppress_alsa_errors():
            pydub_play(segment)
    finally:
        stream.close()


__all__ = ["text_to_speech_stream", "text_to_speech_bytes", "speak"]


if __name__ == "__main__":
    speak("Testing streaming text-to-speech output from ElevenLabs.")