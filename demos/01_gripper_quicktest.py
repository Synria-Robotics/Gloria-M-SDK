from __future__ import annotations

import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk.demo_support.quicktest_demo import main as _demo_main


def main() -> int:
    return _demo_main()


if __name__ == "__main__":
    raise SystemExit(main())

