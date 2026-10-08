from __future__ import annotations

import sys
from pathlib import Path


# 让 scripts/ 下直接运行的脚本能导入 ETHERCAT_SEABOX 根目录下的正式包。
ROOT_DIR = Path(__file__).resolve().parents[1]
ROOT_STR = str(ROOT_DIR)
if ROOT_STR not in sys.path:
    sys.path.insert(0, ROOT_STR)
