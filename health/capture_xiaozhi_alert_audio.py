"""Capture approved alert sentences in the configured Xiaozhi voice.

This is a one-time asset preparation tool. It never runs in the GUI. It
promotes audio only when the server's TTS transcript exactly matches an
approved sentence, so generated additions cannot become alert recordings.
"""

import argparse
import asyncio
import audioop
import sys
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "py-xiaozhi"))

import opuslib

from src.constants.constants import AudioConfig
from src.protocols.websocket_protocol import WebsocketProtocol
from voice_assets import ALERT_SPEECH

PROMPT = "调用预警朗读工具"
TOOL_NAME = "self.health_voice_recording.read"


async def capture_one(key: str, timeout: float = 30.0) -> bytes:
    approved = ALERT_SPEECH[key]
    protocol = WebsocketProtocol()
    stopped = asyncio.Event()
    decoder = opuslib.Decoder(AudioConfig.OUTPUT_SAMPLE_RATE, AudioConfig.CHANNELS)
    segments: list[dict] = []
    speaking = False
    tool_called = False
    decode_errors = 0

    async def handle_mcp(payload: dict) -> None:
        nonlocal tool_called
        method, request_id = payload.get("method"), payload.get("id")
        if request_id is None:
            return
        if method == "initialize":
            result = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                      "serverInfo": {"name": "health-voice-recording", "version": "1.0"}}
        elif method == "tools/list":
            result = {"tools": [{
                "name": TOOL_NAME,
                "description": "当用户说调用预警朗读工具时，必须调用本工具。工具返回需要逐字朗读的预警句。",
                "inputSchema": {"type": "object", "properties": {}, "required": []},
            }]}
        elif method == "tools/call" and payload.get("params", {}).get("name") == TOOL_NAME:
            tool_called = True
            result = {"content": [{"type": "text", "text": approved}]}
        else:
            result = {"tools": []}
        await protocol.send_mcp_message({"jsonrpc": "2.0", "id": request_id, "result": result})

    def on_json(message: dict) -> None:
        nonlocal speaking
        kind = message.get("type")
        if kind == "mcp":
            asyncio.create_task(handle_mcp(message.get("payload") or {}))
        elif kind == "tts":
            state = message.get("state")
            if state == "start":
                speaking = True
            elif state == "sentence_start":
                segments.append({"text": str(message.get("text") or ""), "pcm": []})
            elif state == "stop":
                speaking = False
                stopped.set()
        elif kind == "error":
            stopped.set()

    def on_audio(packet: bytes) -> None:
        nonlocal decode_errors
        if speaking and segments:
            try:
                segments[-1]["pcm"].append(decoder.decode(packet, AudioConfig.OUTPUT_FRAME_SIZE))
            except opuslib.OpusError:
                decode_errors += 1

    protocol.on_incoming_json(on_json)
    protocol.on_incoming_audio(on_audio)
    if not await protocol.connect():
        raise RuntimeError("小智语音服务连接失败")
    try:
        await protocol.send_wake_word_detected(PROMPT)
        await asyncio.wait_for(stopped.wait(), timeout)
        if not tool_called:
            raise RuntimeError("服务端没有调用朗读工具")
        if decode_errors:
            raise RuntimeError(f"音频解码有 {decode_errors} 帧失败")
        for start in range(len(segments)):
            transcript = ""
            for end in range(start, len(segments)):
                transcript += segments[end]["text"]
                if transcript == approved:
                    selected = segments[start:end + 1]
                    if not all(item["pcm"] for item in selected):
                        break
                    audio = b"".join(frame for item in selected for frame in item["pcm"])
                    seconds = len(audio) / (2 * AudioConfig.OUTPUT_SAMPLE_RATE)
                    if not 2 <= seconds <= 20:
                        raise RuntimeError(f"异常音频时长: {seconds:.1f} 秒")
                    if audioop.rms(audio, 2) < 500:
                        raise RuntimeError("录音音量异常或静音")
                    return audio
                if len(transcript) >= len(approved):
                    break
        texts = [item["text"] for item in segments]
        raise RuntimeError(f"TTS 文案不匹配，拒绝保存: {texts}")
    finally:
        await protocol.close_audio_channel()


def write_wav(path: Path, pcm: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".wav.tmp")
    with wave.open(str(temporary), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(AudioConfig.OUTPUT_SAMPLE_RATE)
        output.writeframes(pcm)
    temporary.replace(path)


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("keys", nargs="*", metavar="ALERT_KEY")
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/massage-xiaozhi-alert-captures"))
    parser.add_argument("--attempts", type=int, default=3)
    args = parser.parse_args()
    for key in args.keys:
        if key not in ALERT_SPEECH:
            parser.error(f"unknown alert key: {key}")
    for key in args.keys or ALERT_SPEECH:
        for attempt in range(1, args.attempts + 1):
            try:
                audio = await capture_one(key)
                target = args.output_dir / f"{key}.wav"
                write_wav(target, audio)
                print(f"{key}: exact transcript; {len(audio) / (2 * AudioConfig.OUTPUT_SAMPLE_RATE):.2f}s; {target}")
                break
            except (RuntimeError, asyncio.TimeoutError, opuslib.OpusError) as exc:
                print(f"{key}: attempt {attempt} rejected: {exc}")
        else:
            raise SystemExit(f"{key}: no verified recording after {args.attempts} attempts")


if __name__ == "__main__":
    asyncio.run(main())
