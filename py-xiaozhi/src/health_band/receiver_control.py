"""Controlled lifecycle operations for the local health-band receiver."""

import subprocess
from pathlib import Path
from typing import Dict


HEALTH_DIR = Path(__file__).resolve().parents[3] / "health"
HEALTH_SCRIPT = HEALTH_DIR / "health"


class HealthReceiverController:
    """Run only fixed, local ``health`` lifecycle commands; never uses a shell."""

    def _run(self, action: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            [str(HEALTH_SCRIPT), action],
            cwd=HEALTH_DIR,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )

    def status(self) -> Dict[str, object]:
        if not HEALTH_SCRIPT.is_file():
            return {"running": False, "text": "未找到 health 接收器脚本"}
        try:
            result = self._run("receiver-status")
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"running": False, "text": f"无法检查接收器：{exc}"}
        text = (result.stdout or result.stderr).strip() or "接收器状态未知"
        return {"running": result.returncode == 0, "text": text}

    def start(self) -> Dict[str, object]:
        return self._change("start")

    def stop(self) -> Dict[str, object]:
        return self._change("stop")

    def _change(self, action: str) -> Dict[str, object]:
        try:
            result = self._run(action)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"success": False, "text": f"接收器{action}失败：{exc}", **self.status()}
        action_text = (result.stdout or result.stderr).strip()
        state = self.status()
        return {
            "success": result.returncode == 0,
            "text": action_text or str(state["text"]),
            **state,
        }
