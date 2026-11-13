"""Approved, prerecorded health announcements.

Only an exact match may be played. This keeps model-generated replies and
unreviewed health wording out of the automatic alert path.
"""

from pathlib import Path


ALERT_SPEECH = {
    "hr_rise": "心率较个人基线明显升高，请检查手环佩戴，并确认是否不适。",
    "hr_fall": "心率较个人基线明显下降，请检查手环佩戴，并确认是否头晕或不适。",
    "spo2_watch": "血氧持续低于95%，请检查手环佩戴，并确认是否不适。",
    "spo2_urgent": "血氧持续低于88%，请立即复测并尽快就医。",
    "temp_urgent": "手环体温读数持续低于35℃，请立即用体温计复测；如仍偏低或有不适，请及时就医。",
    "temp_high": "手环体温读数持续高于38℃，请立即用体温计复测；如确认发热或有不适，请及时就医。",
}

AUDIO_DIR = Path(__file__).resolve().parent / "audio"


def alert_audio_path(speech: str) -> Path | None:
    """Return the bundled audio only for an approved announcement."""
    for key, approved in ALERT_SPEECH.items():
        if speech == approved:
            return AUDIO_DIR / f"{key}.wav"
    return None
