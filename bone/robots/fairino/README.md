# Fairino Partition

This directory contains Fairino-specific robot code and data.

Common entry points:

```bash
cd /home/franka/massage/robots/fairino
./run_fairino_compliance.sh --dry-run
./run_fairino_spring.sh --help
./run_demo.sh
```

The older `demo.py`/`lasttime.py` stack uses Fairino SDK imports and shared vision assets through symlinks in this directory.
