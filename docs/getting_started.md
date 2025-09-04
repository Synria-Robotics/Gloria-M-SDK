# 快速开始

## 安装

```bash
pip install gloria-msdk
```

或从源码：

```bash
pip install -e .
```

## 基本使用

```python
from gloria_msdk import SerialManager, SerialConfig, Command

cfg = SerialConfig(port="COM3", baudrate=115200)
mgr = SerialManager(cfg)

mgr.on_status.register(lambda st: print("status", st))
mgr.on_ack.register(lambda c: print("ack", c))
mgr.on_error.register(lambda e: print("error", e))

mgr.open()
mgr.send_command(Command(code=0x01))

import time
time.sleep(2)
mgr.close()
```

## 协议简述

自定义帧格式：`Start(0x40) | Type | Length | Payload | CRC16(lo,hi)`。

CRC16 使用 Modbus (0xA001) 对 `Type + Length + Payload` 计算。
