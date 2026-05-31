# FOC 设计结论

## 控制环调度

- FOC loop 使用 `interrupt executor`，由 ADC/PWM 采样节奏驱动。
- 普通 executor 只跑 CLI、WS2812、显示、日志、参数更新。
- ISR / DMA complete 只做短通知，不放复杂控制逻辑。
- FOC task 里执行：取电流采样 -> 取预测角度 -> Park/PI/SVPWM -> 更新 PWM。

## 电流采样

- 电流采样必须和 TIM1 PWM 同步，不能长期使用 ADC continuous 自由采样。
- 推荐 TIM1 center-aligned PWM，在周期中心用 TRGO/CH4 触发 ADC。
- ADC + DMA 采一帧 `IA/IB/IC/VBUS`，FOC 使用采样完成事件运行。
- 上电先关输出采样电流 offset，再进入驱动状态。

## 角度读取

- TLI5012B 不在 FOC loop 里阻塞读取。
- 独立高优先级 angle task 用 SPI/SSC 周期读取角度，更新 latest state。
- FOC loop 只读 `latest_theta/latest_omega/timestamp`，并做预测：

```text
theta_now = theta_meas + omega_est * dt
```

- 角度读取频率建议 2kHz~10kHz，SPI 建议 4MHz~5MHz。
- 速度用角度差分加低通/PLL，不直接裸差分给控制环。

## CORDIC

- STM32G431 的 CORDIC 应用于 FOC 高频三角函数。
- 优先加速 `sin/cos`，用于 Park 和 inverse Park。
- CORDIC 资源归 FOC 独占，避免普通任务加锁抢占。
- 前期可提供 `sin_cos(f32)` 封装，后续再切 Q 格式。

## 起转和识别

- 起转、方向判定、极对数识别做成 commissioning 状态机，不混入正常 FOC loop。
- 起转流程：转子定向 -> 开环强拖低速 ramp -> 读取机械角变化。
- 方向判定：电角度正向增加时，机械角增加为正向，减少为反向。
- 极对数识别：

```text
pole_pairs = round(abs(electrical_delta / mechanical_delta))
```

- 电角度 offset：

```text
theta_e = direction * pole_pairs * theta_m + electrical_offset
```

## 模块建议

```text
src/driver/current.rs
src/driver/cordic.rs
src/driver/tli5012b.rs
src/task/angle.rs
src/task/foc.rs
src/task/commissioning.rs
src/communication.rs
```
