"""Render the six approved alerts once with the official AISHELL3 VITS model.

Usage: python generate_alert_audio.py /path/to/vits-icefall-zh-aishell3
The model is needed only to build the WAV assets, not to run the GUI.
"""

import argparse
import wave
from pathlib import Path

import numpy as np
import sherpa_onnx

from voice_assets import ALERT_SPEECH, AUDIO_DIR


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model_dir", type=Path)
    parser.add_argument("--speaker", type=int, default=99)
    args = parser.parse_args()
    directory = args.model_dir
    config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(directory / "model.onnx"),
                lexicon=str(directory / "lexicon.txt"),
                tokens=str(directory / "tokens.txt"),
            ),
            num_threads=2,
        ),
        rule_fsts=",".join(str(directory / name) for name in ("phone.fst", "date.fst", "number.fst")),
        max_num_sentences=1,
    )
    if not config.validate():
        raise ValueError(f"Invalid TTS model directory: {directory}")
    tts = sherpa_onnx.OfflineTts(config)
    generation = sherpa_onnx.GenerationConfig()
    generation.sid = args.speaker
    generation.speed = 1.0
    generation.silence_scale = 0.2
    AUDIO_DIR.mkdir(exist_ok=True)
    for key, speech in ALERT_SPEECH.items():
        spoken = (speech.replace("95%", "百分之九十五")
                        .replace("88%", "百分之八十八")
                        .replace("35℃", "三十五摄氏度")
                        .replace("38℃", "三十八摄氏度"))
        recording = tts.generate(spoken, generation)
        if len(recording.samples) == 0:
            raise RuntimeError(f"No audio generated for {key}")
        samples = np.asarray(recording.samples, dtype=np.float32)
        peak = float(np.max(np.abs(samples)))
        if peak <= 0:
            raise RuntimeError(f"Silent audio generated for {key}")
        pcm = (np.clip(samples * (0.75 / peak), -1, 1) * 32767).astype("<i2")
        path = AUDIO_DIR / f"{key}.wav"
        with wave.open(str(path), "wb") as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(recording.sample_rate)
            output.writeframes(pcm.tobytes())
        print(f"{key}: {len(pcm) / recording.sample_rate:.1f}s -> {path}")


if __name__ == "__main__":
    main()
