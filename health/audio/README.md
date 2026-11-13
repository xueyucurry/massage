# 固定体征播报音频

这六段 WAV 与 `health/voice_assets.py` 的六句批准文案一一对应。运行 GUI 只读取 WAV，通过小智现有的 `AudioCodec` 输出流播放，不向云端请求生成式回复。

当前正式音频于 2026-11-13 使用设备已配置的小智服务一次性录制。`health/capture_xiaozhi_alert_audio.py` 向临时朗读工具提供六句批准文案；只在服务端实际调用工具且 TTS 文字与整句完全一致时保存对应音频。体温两句由服务端分段下发，脚本先拼接分段文字核对，再拼接对应音频。录音均为 24 kHz、单声道、16 位 PCM，时长约 4.5～8.3 秒。已通过实体扬声器播放一条候选录音；音色确认和完整六句逐句听测仍待用户反馈。

重录时执行 `py-xiaozhi/.venv-linux/bin/python health/capture_xiaozhi_alert_audio.py`，脚本仅写入 `/tmp/massage-xiaozhi-alert-captures/`；检查文字、时长和试听结果后再复制到本目录。若服务端改写文案，脚本拒绝保存。先前 AISHELL3 模型录音留在本机的 `previous_aishell3/` 备份目录，未纳入 Git；`health/generate_alert_audio.py` 可按[模型官方说明](https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html#aishell3-chinese-multi-speaker-174-speakers)重新生成，其音色与小智不同。
