# 06_mit_linkage_force_control

这个脚本是一个基于 MIT 模式的连杆夹爪力控示例，适合当前这套达妙电机夹爪做联调和参数验证。

当前版本的基本约定是：

- 正扭矩表示张开
- 负扭矩表示闭合
- 默认张开位置是 `2.77 rad`
- 默认闭合位置是 `0.003 rad`

## 当前控制流程

默认运行时，脚本按下面的状态机工作：

1. `opening`
   用正扭矩把夹爪张开到 `--open-q` 附近。
2. `approaching`
   用负扭矩让夹爪向 `--close-q` 方向闭合。
3. `holding`
   检测到接触、堵转，或者到达闭合极限后，进入保持阶段。
4. `releasing`
   默认保持 `--hold-seconds` 秒后，自动张开。
5. 回到 `opening`
   如果 `--cycles` 大于 1，会继续下一轮；如果 `--cycles 0`，则一直循环。

接触判定使用以下条件之一：

- 有效夹持力 `eff_force` 超过 `--contact-force`
- 闭合过程中速度很小，满足堵转条件
- 到达闭合位置 `--close-q`

## 力估算说明

脚本内部不是直接把电机反馈力矩当成真实夹持力，而是做了两层处理：

1. 用 `--radius-profile` 描述“电机角度 -> 等效力臂”的连杆映射
2. 如果加载了 `--baseline-csv`，则先扣掉空载基线，再计算有效夹持力

运行日志里常见的几个量含义如下：

- `tau_fb`：电机反馈力矩
- `tau_free`：空载基线插值力矩
- `delta_tau`：相对空载的增量闭合力矩
- `force_est`：基于原始反馈力矩估算的夹持力
- `eff_force`：基于 `delta_tau` 估算的有效夹持力
- `p_m`：电机寄存器位置
- `xout`：另一组实际位置相关寄存器值

## 位置模式

除了默认的力控流程，这个脚本还支持位置模式：

- `--position-only`
  只运动到目标闭合位置并保持，不走正常的接触后力保持逻辑。
- `--hold-q`
  指定保持位置；未设置时，`position-only` 模式默认使用 `--close-q`。
- `--hold-kp`
  位置保持刚度。

## 关键参数

- `--open-q`
  最大张开位置。
- `--close-q`
  无接触时最终闭合目标位置。
- `--open-tau`
  张开扭矩，必须为正。
- `--close-tau`
  闭合扭矩，必须为负。
- `--target-force`
  保持阶段目标夹持力。
- `--contact-force`
  接触判定阈值；加载基线后表示增量夹持力阈值。
- `--abort-force`
  过力保护阈值；加载基线后表示增量夹持力保护阈值。
- `--max-hold-tau`
  保持阶段允许输出的最大闭合扭矩。
- `--radius-profile`
  连杆等效力臂曲线，格式为 `q:radius_mm`。
- `--baseline-csv`
  空载基线文件，建议先用 `07_mit_close_test.py` 生成。
- `--hold-seconds`
  保持时间，时间到后会自动进入释放阶段。
- `--cycles`
  循环次数，`0` 表示一直循环。
- `--start-phase`
  起始阶段，可选 `auto`、`opening`、`approaching`。
- `--print-hz`
  状态打印频率。

## 推荐使用方式

先做一份空载基线：

```bash
python demos/07_mit_close_test.py --port COM12 --close-tau -1.25 --timeout 3 --stop-force 0
```

然后再带着基线跑 `06`：

```bash
python demos/06_mit_linkage_force_control.py --port COM12 --baseline-csv demos/output/close_baseline_xxx_binned.csv --target-force 8 --cycles 1
```

如果想从闭合阶段直接开始：

```bash
python demos/06_mit_linkage_force_control.py --port COM12 --baseline-csv demos/output/close_baseline_xxx_binned.csv --start-phase approaching --open-tau 1.25 --close-tau -1.25
```

如果只想运动到某个设定闭合位置并保持：

```bash
python demos/06_mit_linkage_force_control.py --port COM12 --close-q 1.80 --position-only --hold-kp 25 --start-phase approaching
```

## 调参建议

- 闭合太慢，先增大 `--close-tau` 的绝对值。
- 张开太慢，增大 `--open-tau`。
- 夹持不够紧，先增大 `--target-force`，再看是否需要增大 `--max-hold-tau`。
- 接触误判太多，适当增大 `--contact-force`，并优先使用空载基线。
- 连杆接近死点时要特别小心，建议保守设置 `--abort-force` 和 `--max-hold-tau`。

## 注意事项

- `radius-profile` 现在仍然依赖机构标定，默认值只是占位参数。
- 如果没有加载基线，日志里的“力”更适合作为相对指标，不建议直接当作真实牛顿值。
- 当前脚本是“示例/联调脚本”，默认仍然包含 `hold-seconds` 到时自动释放的逻辑。
