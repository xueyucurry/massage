"""
使用 huggingface_hub 将 ArtVIP 数据集下载到本地。
默认会尝试使用镜像站（hf-mirror），可通过环境变量覆盖。
"""

import os
from pathlib import Path

from huggingface_hub import snapshot_download

# 配置：可根据需要修改或通过环境变量覆盖
REPO_ID = os.getenv("ARTVIP_REPO_ID", "x-humanoid-robomind/ArtVIP")
LOCAL_DIR = Path(os.getenv("ARTVIP_LOCAL_DIR", r"D:\ArtVIP"))
MAX_WORKERS = int(os.getenv("ARTVIP_MAX_WORKERS", "8"))
RESUME = os.getenv("ARTVIP_RESUME", "true").lower() != "false"


def main() -> None:
    # 默认启用国内镜像，可通过 HF_ENDPOINT 环境变量修改或清空
    os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
    # 启用更快的传输（若安装了 hf_transfer）
    os.environ.setdefault("HF_HUB_ENABLE_HF_TRANSFER", "1")

    LOCAL_DIR.mkdir(parents=True, exist_ok=True)

    print(f"开始下载 {REPO_ID} 到 {LOCAL_DIR} ...")
    snapshot_download(
        repo_id=REPO_ID,
        repo_type="dataset",
        local_dir=str(LOCAL_DIR),
        local_dir_use_symlinks=False,
        resume_download=RESUME,
        max_workers=MAX_WORKERS,
        token=os.getenv("HF_TOKEN"),  # 私有/受限数据需设置
    )
    print("下载完成")


if __name__ == "__main__":
    main()










