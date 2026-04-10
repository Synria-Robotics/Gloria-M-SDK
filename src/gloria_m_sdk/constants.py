from __future__ import annotations

# 夹爪安全位置区间 [MIT_SAFE_Q_MIN, MIT_SAFE_Q_MAX]（rad）。
# 约定：0.0 为完全张开，沿正方向增大趋向闭合，最大闭合约 2.7 rad。
MIT_SAFE_Q_MIN: float = 0.0
MIT_SAFE_Q_MAX: float = 2.7
