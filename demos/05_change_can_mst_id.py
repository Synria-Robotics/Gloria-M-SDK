from __future__ import annotations

"""
更改电机 ESC_ID(CAN ID) 和 MST_ID 的 demo。于2026年3月31日修改。

协议要点（来自现有 SDK 的约定）：
- 目标电机 ID 通过 payload 的前两个字节 (can_id_l, can_id_h) 指定
- RID 写入：data[2] = 0x55，data[3] = rid，后续 4 bytes 为值（u32 或 float32）
- RID 读取：data[2] = 0x33，data[3] = rid
- 保存参数：data[2] = 0xAA
"""

import argparse
import os
import struct
import sys
import time

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk import SerialCanAdapter, Variable


def _parse_int(s: str) -> int:
    return int(s, 0)


def _is_u32_param(rid: int) -> bool:
    # 与 src/gloria_m_sdk/controller.py 保持一致
    return (7 <= rid <= 10) or (13 <= rid <= 16) or (35 <= rid <= 36)


def _send_read(adapter: SerialCanAdapter, *, target_id: int, rid: int) -> None:
    can_id_l = target_id & 0xFF
    can_id_h = (target_id >> 8) & 0xFF
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x33, rid & 0xFF, 0, 0, 0, 0]))


def _send_write_u32(adapter: SerialCanAdapter, *, target_id: int, rid: int, value: int) -> None:
    can_id_l = target_id & 0xFF
    can_id_h = (target_id >> 8) & 0xFF
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0x55, rid & 0xFF]) + struct.pack("<I", int(value) & 0xFFFFFFFF))


def _send_save(adapter: SerialCanAdapter, *, target_id: int) -> None:
    can_id_l = target_id & 0xFF
    can_id_h = (target_id >> 8) & 0xFF
    adapter.send(0x7FF, bytes([can_id_l, can_id_h, 0xAA, 0x00, 0, 0, 0, 0]))


def _wait_param(
    adapter: SerialCanAdapter,
    *,
    target_id: int,
    rid: int,
    timeout_s: float,
) -> int | float | None:
    t = int(target_id)
    can_id_l = t & 0xFF
    can_id_h = (t >> 8) & 0xFF
    # 参数回包可能使用 command_id 或 feedback_id(常见为 0x100 + id)
    expect_ids = {t, int(0x100 + t), 0x00}
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        for pkt in adapter.read_packets():
            # 参数回包的 cmd 在不同固件/适配器实现中可能不同，这里不强依赖 cmd
            if len(pkt.data) != 8:
                continue
            if int(pkt.can_id) not in expect_ids and not (pkt.data[0] == can_id_l and pkt.data[1] == can_id_h):
                continue
            if pkt.data[2] not in (0x33, 0x55):
                continue
            if int(pkt.data[3]) != int(rid & 0xFF):
                continue

            if _is_u32_param(rid):
                return struct.unpack("<I", pkt.data[4:8])[0]
            return struct.unpack("<f", pkt.data[4:8])[0]
        time.sleep(0.01)
    return None


def _read_param_strict_u32(
    adapter: SerialCanAdapter,
    *,
    target_id: int,
    rid: int,
    timeout_s: float,
    retries: int,
) -> int:
    for _ in range(max(1, int(retries))):
        _send_read(adapter, target_id=int(target_id), rid=int(rid))
        val = _wait_param(adapter, target_id=int(target_id), rid=int(rid), timeout_s=float(timeout_s))
        if val is not None:
            return int(val)
    raise RuntimeError(f"读取寄存器失败：rid={int(rid)} target_id={int(target_id)}（超时 {timeout_s}s，重试 {retries} 次）")


def main() -> int:
    ap = argparse.ArgumentParser(description="更改电机 ESC_ID(CAN ID) 与 MST_ID（写入后自动 SAVE 持久化）")
    ap.add_argument("--port", default="COM12", help="串口号。Windows 例如 COM8；Linux 例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("--id", type=_parse_int, default="0x01", help="当前电机 ESC_ID/CAN ID，默认 0x01")
    ap.add_argument("--new-can-id", type=_parse_int, default=None, help="要写入的新 ESC_ID/CAN ID，例如 0x02")
    ap.add_argument("--new-mst-id", type=_parse_int, default=None, help="要写入的新 MST_ID，例如 0x01")
    ap.add_argument("--timeout", type=float, default=1.0, help="等待回包超时（秒），默认 1.0")
    ap.add_argument("--retries", type=int, default=8, help="读回重试次数（每次都会重新发读指令），默认 8")
    args = ap.parse_args()

    cur_id = int(args.id)
    rid_mst = int(Variable.MST_ID)
    rid_esc = int(Variable.ESC_ID)

    with SerialCanAdapter(args.port, baudrate=int(args.baud), timeout=0.2) as adapter:
        # 强制读当前值（读不到就失败退出）
        mst0 = _read_param_strict_u32(
            adapter,
            target_id=cur_id,
            rid=rid_mst,
            timeout_s=float(args.timeout),
            retries=int(args.retries),
        )
        esc0 = _read_param_strict_u32(
            adapter,
            target_id=cur_id,
            rid=rid_esc,
            timeout_s=float(args.timeout),
            retries=int(args.retries),
        )
        print(f"[before] MST_ID={mst0}  ESC_ID(CAN)={esc0}")

        if args.new_mst_id is None and args.new_can_id is None:
            print("未指定 --new-mst-id / --new-can-id，仅做读取。")
            return 0

        # 先写 MST_ID（不影响通信），再写 ESC_ID（会改变电机 CAN ID）
        desired_mst = int(mst0 if args.new_mst_id is None else args.new_mst_id)
        desired_esc = int(esc0 if args.new_can_id is None else args.new_can_id)

        if args.new_mst_id is not None:
            new_mst = int(args.new_mst_id)
            print(f"[write] MST_ID <- {new_mst}")
            _send_write_u32(adapter, target_id=cur_id, rid=rid_mst, value=new_mst)
            time.sleep(0.05)

        new_id = cur_id
        if args.new_can_id is not None:
            new_id = int(args.new_can_id)
            print(f"[write] ESC_ID(CAN) <- {new_id}")
            _send_write_u32(adapter, target_id=cur_id, rid=rid_esc, value=new_id)
            time.sleep(0.1)

        print("[save] SAVE(0xAA)")
        _send_save(adapter, target_id=new_id)
        time.sleep(0.2)

        # 读取验证（优先按新 ID 读；失败则回退按旧 ID 再试一次）
        try:
            mst1 = _read_param_strict_u32(
                adapter,
                target_id=new_id,
                rid=rid_mst,
                timeout_s=float(args.timeout),
                retries=int(args.retries),
            )
            esc1 = _read_param_strict_u32(
                adapter,
                target_id=new_id,
                rid=rid_esc,
                timeout_s=float(args.timeout),
                retries=int(args.retries),
            )
        except RuntimeError:
            mst1 = _read_param_strict_u32(
                adapter,
                target_id=cur_id,
                rid=rid_mst,
                timeout_s=float(args.timeout),
                retries=int(args.retries),
            )
            esc1 = _read_param_strict_u32(
                adapter,
                target_id=cur_id,
                rid=rid_esc,
                timeout_s=float(args.timeout),
                retries=int(args.retries),
            )
        print(f"[after ] MST_ID={mst1}  ESC_ID(CAN)={esc1}")
        if int(mst1) != int(desired_mst) or int(esc1) != int(desired_esc):
            raise SystemExit(
                f"验证失败：期望 MST_ID={desired_mst}, ESC_ID(CAN)={desired_esc}，"
                f"实际 MST_ID={mst1}, ESC_ID(CAN)={esc1}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

