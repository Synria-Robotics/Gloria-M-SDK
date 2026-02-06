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
- MIT 安全位置范围：\([-2.9, 0.0]\)，其中 **0.0 为完全张开**
- MIT 缩放上限（PMAX/VMAX/TMAX）：`[3.14, 10, 12]`

### 1) MIT 往返开合示例

```bash
python demos/mit_quickstart.py --port COM8
```

### 2) MIT 力控示例

```bash
python demos/mit_torque_control.py --port COM8
```

### 3) PV 位置精度示例

```bash
python demos/pv_position_accuracy.py --port COM8
```

## 备注

- **发控制帧才会返回最新位置/速度/力矩**。

