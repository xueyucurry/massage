import glob
import importlib.util
import os
import sys


_PKG_DIR = os.path.dirname(__file__)
_PROJECT_ROOT = os.path.dirname(_PKG_DIR)

_SDK_BASE_DIRS = [
    os.environ.get("FAIRINO_SDK_BASE", "").strip(),
    os.path.join(
        _PROJECT_ROOT,
        "fairino-python-sdk-master (1)",
        "fairino-python-sdk-master",
    ),
    os.path.join(
        _PROJECT_ROOT,
        "fairino-python-sdk-master",
        "fairino-python-sdk-master",
    ),
]

_LEGACY_ROOTS = [
    os.environ.get("FAIRINO_SDK_ROOT", "").strip(),
    "/home/franka/py-xiaozhi/src/user_functions/fairino",
]


def _iter_module_candidates():
    explicit_module = os.environ.get("FAIRINO_SDK_MODULE", "").strip()
    if explicit_module:
        yield explicit_module

    for base_dir in _SDK_BASE_DIRS:
        if not base_dir:
            continue

        lib_dir = os.path.join(base_dir, "linux", "libfairino")
        for module_path in sorted(glob.glob(os.path.join(lib_dir, "Robot*.so"))):
            yield module_path

        yield os.path.join(base_dir, "linux", "fairino", "Robot.py")

    for legacy_root in _LEGACY_ROOTS:
        if not legacy_root:
            continue
        yield os.path.join(legacy_root, "Robot.py")


def _load_robot_module(module_path):
    if not module_path:
        return None

    if not os.path.isfile(module_path):
        return None

    module_dir = os.path.dirname(module_path)
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

    spec = importlib.util.spec_from_file_location("fairino.Robot", module_path)
    if spec is None or spec.loader is None:
        return None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


Robot = None
SDK_ROOT = None
SDK_MODULE = None

for _candidate in _iter_module_candidates():
    _module = _load_robot_module(_candidate)
    if _module is not None:
        Robot = _module
        SDK_MODULE = _candidate
        SDK_ROOT = os.path.dirname(_candidate)
        break

if Robot is None:
    raise ImportError(
        "未找到可用的 FAIRINO Linux SDK，请确认 FAIRINO_SDK_MODULE / FAIRINO_SDK_BASE "
        "或项目内 fairino-python-sdk-master 路径可用"
    )

__all__ = ["Robot", "SDK_ROOT", "SDK_MODULE"]
