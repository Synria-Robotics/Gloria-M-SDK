## 项目说明

这是一个**标准 CAN 控制的 Python 示例工程**，将**可复用 API 层**与 **demos 示例**分离，便于快速上手与二次开发。

当前实现使用“串口转 CAN”适配器的封包格式（发送 30 字节、接收 16 字节），并提供：

- **MIT 模式 demo**：按 5 参数（kp/kd/pos/vel/tau_ff）控制
- **MIT 力控 demo**：使用额外力矩参数，并打印位置/速度/力矩反馈
- **PV 模式 demo**：速度追位置，打印目标位置与当前位置误差（考虑延时采样）

## 目录结构

- `src/gloria_m_sdk/`：库代码
- `demos/`：可运行示例

## 安装

建议使用虚拟环境：

```bash
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
pip install -e .
```

## 运行 demos（夹爪示例）

- 电机命令 ID：`0x01`
- 反馈 ID：`0x101`
- MIT 安全位置范围：\([MIT_SAFE_Q_MIN, 0.0]\)，其中 **0.0 为完全张开**
- `MIT_SAFE_Q_MIN` 定义在 `gloria_m_sdk` 中（`src/gloria_m_sdk/constants.py`），如需调整限幅下限，修改这一处即可。
- MIT 缩放上限（PMAX/VMAX/TMAX）：`[3.14, 10, 12]`

### 1) MIT 往返开合示例

```bash
python demos/01_mit_quickstart.py --port COM8
```

### 2) MIT 力控示例

```bash
python demos/03_mit_torque_control.py --port COM8
```

### 3) PV 位置精度示例

```bash
python demos/04_pv_position_accuracy.py --port COM8
```

### 4) 修改电机 CAN ID / MST_ID（并保存）

> 修改 `ESC_ID(CAN ID)` / `MST_ID` 后会自动发送 `SAVE(0xAA)` 持久化。

```bash
# 读取当前 MST_ID / ESC_ID
python demos/05_change_can_mst_id.py --port COM8 --id 0x01

# 修改 CAN ID 与 MST_ID（写入后自动保存）
python demos/05_change_can_mst_id.py --port COM12 --id 0x02 --new-can-id 0x01 --new-mst-id 0x101 
```

### 5) 06 连杆夹爪力控示例

脚本：`demos/06_mit_linkage_force_control.py`

用途：

- 基于 MIT 模式做连杆夹爪开合、接触判定和力保持
- 支持空载基线补偿，适合联调 `target-force`、`contact-force`、`radius-profile`
- 默认仍然保留 demo 风格的自动张开流程

默认电机参数：

- 夹爪命令 ID：`0x07`
- 夹爪反馈 ID：`0x207`
- 最大张开位置：`2.77 rad`
- 最小闭合位置：`0.003 rad`

建议先生成一份空载基线，再运行 `06`：

```bash
python demos/07_mit_close_test.py --port COM12 --close-tau -1.25 --timeout 3 --stop-force 0
python demos/06_mit_linkage_force_control.py --port COM12 --baseline-csv demos/output/close_baseline_xxx_binned.csv --target-force 8 --cycles 1
```

更详细的参数说明见：`demos/06_mit_linkage_force_control.md`

### 6) 07 夹爪闭合测试 / 空载基线标定

脚本：`demos/07_mit_close_test.py`

用途：

- 单独验证“负扭矩是否能让夹爪闭合”
- 记录闭合过程的位置、速度、反馈力矩
- 生成后续 `06/08` 可直接使用的空载基线 CSV

默认电机参数：

- 夹爪命令 ID：`0x07`
- 夹爪反馈 ID：`0x207`

基本用法：

```bash
python demos/07_mit_close_test.py --port COM12
```

推荐用于基线标定的命令：

```bash
python demos/07_mit_close_test.py --port COM12 --close-tau -1.25 --timeout 3 --stop-force 0
```

运行结束后，默认会在 `demos/output/` 下生成：

- `*_raw.csv`：原始采样数据
- `*_binned.csv`：按位置分桶后的空载基线

### 7) 08 跟随输入电机位置的夹爪控制示例

脚本：`demos/08_mit_linkage_follow_input_close_q.py`

用途：

- 持续读取另一台电机的位置，作为动态 `close_q` 输入
- 保留 `06` 的接触判定和力保持逻辑
- 去掉 demo 风格的自动张开循环，更适合实际交互控制

默认电机参数：

- 夹爪命令 ID：`0x07`
- 夹爪反馈 ID：`0x207`
- 输入电机命令 ID：`0x01`
- 输入电机反馈 ID：`0x201`
- 默认空载基线：`demos/output/close_baseline_20260407_114136_binned.csv`

基本用法：

```bash
python demos/08_mit_linkage_follow_input_close_q.py --port COM12
```

常用调试命令：

```bash
python demos/08_mit_linkage_follow_input_close_q.py --port COM12 --target-force 5 --contact-force 4
python demos/08_mit_linkage_follow_input_close_q.py --port COM12 --input-scale 1.0 --input-offset 0.0
```

说明：

- `0x01 / 0x201` 电机位置会实时映射为 `close_q_in`
- 正扭矩表示张开，负扭矩表示闭合
- 过力保护阈值会自动按 `max(target-force, contact-force) + 5` 计算

## 备注

- **发控制帧才会返回最新位置/速度/力矩/温度**。

