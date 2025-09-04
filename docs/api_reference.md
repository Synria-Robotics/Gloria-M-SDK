# API 参考

## SerialManager

```python
SerialManager(config: SerialConfig, serial_cls: Optional[type]=None)
```

方法：
- `open()` 打开串口并启动读/写线程
- `close()` 关闭
- `send_command(command: Command, wait_ack=False, timeout=1.0)` 发送命令，可阻塞等待 ACK
- `is_open()` 查询状态

回调注册器：
- `on_status` -> `DeviceStatus`
- `on_ack` -> `Command`
- `on_error` -> `Exception | str`

## Command

字段：`code:int`, `params:bytes`, `seq:int|None`

## DeviceStatus

字段：`position:int`, `force:int`, `error:int`, `timestamp:float`

## 异常

`GloriaMError` 基类，以及：
`SerialConnectionError`, `ProtocolError`, `CRCMismatchError`, `AckTimeoutError` 等。
