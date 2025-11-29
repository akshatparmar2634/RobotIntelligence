"""Utilities for capturing audio and transcribing it with ElevenLabs Speech-to-Text."""

from __future__ import annotations

import os
import tempfile
import uuid
import wave
from typing import Any, Optional

try:  # Optional convenience for local development
    from dotenv import load_dotenv  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    load_dotenv = None  # type: ignore

try:
    from elevenlabs.client import ElevenLabs  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    ElevenLabs = None  # type: ignore

if load_dotenv is not None:  # pragma: no cover - no effect in production deployments
    load_dotenv()


ELEVENLABS_API_KEY_ENV = "ELEVENLABS_API_KEY"
ELEVENLABS_STT_MODEL_ENV = "ELEVENLABS_STT_MODEL"
DEFAULT_STT_MODEL = "scribe_v1"

_ELEVENLABS_CLIENT: Any = None
_ELEVENLABS_CLIENT_KEY: Optional[str] = None


def _get_elevenlabs_client(api_key: Optional[str] = None) -> Any:
    """Instantiate (or reuse) an ElevenLabs client with the provided API key."""
    global _ELEVENLABS_CLIENT, _ELEVENLABS_CLIENT_KEY

    if ElevenLabs is None:
        raise RuntimeError(
            "The 'elevenlabs' package is required for speech-to-text. Install it with 'pip install elevenlabs'."
        )

    resolved_key = api_key or os.getenv(ELEVENLABS_API_KEY_ENV)
    if not resolved_key:
        raise RuntimeError(
            "ElevenLabs API key not set. Please export ELEVENLABS_API_KEY in the environment."
        )

    if _ELEVENLABS_CLIENT is None or resolved_key != _ELEVENLABS_CLIENT_KEY:
        _ELEVENLABS_CLIENT = ElevenLabs(api_key=resolved_key)
        _ELEVENLABS_CLIENT_KEY = resolved_key

    return _ELEVENLABS_CLIENT


def _require_sounddevice():
    try:
        import sounddevice  # noqa: F401  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on optional dependency
        raise RuntimeError(
            "The 'sounddevice' package is required for voice input. Install it with 'pip install sounddevice'."
        ) from exc


def record_audio(duration: float = 5.0, sample_rate: int = 16000, channels: int = 1) -> str:
    """Record audio from the default microphone and return a temporary WAV file path."""
    _require_sounddevice()

    import numpy as np
    import sounddevice as sd  # type: ignore

    frames = int(duration * sample_rate)
    sd.default.samplerate = sample_rate
    sd.default.channels = channels

    audio = sd.rec(frames, samplerate=sample_rate, channels=channels, dtype="int16")
    sd.wait()

    audio = np.squeeze(audio)

    tmp_dir = tempfile.gettempdir()
    file_path = os.path.join(tmp_dir, f"elevenlabs-recording-{uuid.uuid4().hex}.wav")

    with wave.open(file_path, "wb") as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(2)  # int16
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio.tobytes())

    return file_path


def transcribe_audio(
    audio_path: str,
    api_key: Optional[str] = None,
    model_id: Optional[str] = None,
    language_code: Optional[str] = None,
    tag_audio_events: Optional[bool] = None,
    diarize: Optional[bool] = None,
    **extra_options: Any,
) -> str:
    """Send an audio file to ElevenLabs Speech-to-Text and return the transcription."""
    if not os.path.exists(audio_path):
        raise FileNotFoundError(f"Audio file not found: {audio_path}")

    client = _get_elevenlabs_client(api_key)

    resolved_model = model_id or os.getenv(ELEVENLABS_STT_MODEL_ENV, DEFAULT_STT_MODEL)
    convert_kwargs: dict[str, Any] = {"model_id": resolved_model}

    if language_code:
        convert_kwargs["language_code"] = language_code
    if tag_audio_events is not None:
        convert_kwargs["tag_audio_events"] = tag_audio_events
    if diarize is not None:
        convert_kwargs["diarize"] = diarize
    if extra_options:
        convert_kwargs.update(extra_options)

    with open(audio_path, "rb") as audio_file:
        result = client.speech_to_text.convert(
            file=audio_file,
            **convert_kwargs,
        )

    text = _extract_transcription_text(result)
    if not text:
        raise RuntimeError(f"No transcription text found in ElevenLabs response: {result}")

    return text.strip()


def _extract_transcription_text(result: Any) -> Optional[str]:
    """Normalize various ElevenLabs SDK responses into a transcription string."""
    if result is None:
        return None

    if isinstance(result, str):
        return result

    if isinstance(result, dict):
        for key in ("text", "transcription", "transcribed_text", "output"):
            value = result.get(key)
            if isinstance(value, str) and value.strip():
                return value

    for attr in ("text", "transcription", "transcribed_text", "output"):
        value = getattr(result, attr, None)
        if isinstance(value, str) and value.strip():
            return value

    return None


def capture_and_transcribe(
    duration: float = 5.0,
    sample_rate: int = 16000,
    cleanup: bool = True,
    **kwargs,
) -> str:
    """Convenience helper that records audio and returns the transcription."""
    audio_path = record_audio(duration=duration, sample_rate=sample_rate)
    try:
        transcription = transcribe_audio(audio_path, language_code="en", **kwargs)
        return transcription
    finally:
        if cleanup and os.path.exists(audio_path):
            try:
                os.remove(audio_path)
            except OSError:
                pass
