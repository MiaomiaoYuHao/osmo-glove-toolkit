# 磁力计融合仿真测试

在 `Core/tests/` 下运行，主机 gcc 即可，不需要板子。

    gcc -O1 -std=c99 -I ../Inc/glove -o shake  shake.c  -lm && ./shake 900
    gcc -O1 -std=c99 -I ../Inc/glove -o rest   rest.c   -lm && ./rest
    gcc -O1 -std=c99 -I ../Inc/glove -DMAG_D=0.8f -DMAG_RAMP=0.05f -DMOT_OFF=12.0f -o magnet magnet.c -lm && ./magnet 600

shake：3 Hz 往复甩动，参数是峰值角速度（deg/s）。六轴 yaw 误差按转过角度的 1.2% 累积，磁力计 0.5% 噪声。
输出运动中输出误差峰值、停止瞬间误差、停止后回到 2 度以内所需时间。

常用开关（-D）：
- `USE_TS=0` 走旧接口（无时间戳），对比用
- `TS_ERR_MS=3.0f` 时间戳系统偏差；`TS_JIT_MS=2.0f` 随机抖动
- `SKEW=2` mag 与 quat 错拍数；`WARMUP` 先做一段热身甩动
- `DRIFT_K=0.0f` 关闭六轴漂移注入；`NOISE=0.0f` 关闭噪声

magnet：世界系（或 `MAG_BODY=1` 体系）磁干扰，`MAG_D` 强度（地磁 = 1），`MAG_RAMP` 出现时长，
`MOT_ON/MOT_OFF` 甩动区间（设成 50/51 即静止）。

对比旧版本：把 `-DMF_SRC='"路径/mag_fusion.c"' -DUSE_TS=0` 指向旧文件，`-I` 指向旧头文件目录。
