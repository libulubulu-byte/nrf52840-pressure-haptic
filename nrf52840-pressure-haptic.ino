/*
 * ============================================================================
 *  脚底薄膜压力传感器 (FSR) -> BLE Notify -> DRV2605 LRA 触觉反馈
 *  目标板：Seeed XIAO nRF52840 / XIAO nRF52840 Sense
 *  核心  ：Seeed nRF52 Boards (Seeeduino:nrf52)  1.1.13
 *  Arduino IDE 中打开本文件所在目录即可；对应的 Arduino CLI 构建脚本见 build.ps1
 * ----------------------------------------------------------------------------
 *  本程序由早先的 ESP32 例程改造而来。原例程是 ESP32 的 BLE 配网 + DeepSeek
 *  云端对话，其中 WiFi.h / WiFiClientSecure.h / BLEDevice.h / ESP.restart() 在
 *  nRF52840 上**不存在**（nRF52840 没有 WiFi 硬件），故全部移除。仅保留例程的
 *  BLE 骨架与 UUID 风格：
 *    - 1 个主服务 + 多个特征（写配置 / Notify 数据 / 写命令）
 *    - UUID 前缀与例程一致，末位字母递增，便于 nRF Connect 识别
 * ----------------------------------------------------------------------------
 *  数据流：
 *    薄膜压力传感器(脚底) -> A0 采样 50Hz -> 中值+EMA 滤波 -> 阈值判定(滞回)
 *        -> BLE Notify 自定义服务 -> 手机 nRF Connect 实时曲线
 *        -> DRV2605 (I2C) -> LRA 单震 / 双震 / 长震
 *
 *  ★ 引脚（已对官方 variant.cpp / variant.h 逐条核实）
 *     A0 / D0 = P0.02    FSR 分压中点（模拟输入）
 *     D4      = P0.04    I2C SDA -> DRV2605 SDA
 *     D5      = P0.05    I2C SCL -> DRV2605 SCL
 *     ★ RGB 灯：**共阳极接 3V3，所以 HIGH = 灭 / LOW = 亮**（核心 variant.h 里
 *       LED_STATE_ON=1"高电平点亮"对这块板是**错的**，本文件所有写灯处按共阳极写）：
 *         D11 = P0.26  红灯（LED_BUILTIN = LED_RED，2.2k）
 *         D13 = P0.30  绿灯（LED_GREEN，10k）
 *         D12 = P0.06  蓝灯（LED_BLUE，2.2k）—— 库的"广播闪烁"用它
 *       （另有 D23 = P0.17 充电指示 LED ~CHG，硬件自己管，与固件无关）
 *     ⚠ P0.13 = D22 是 HICHG 充电电流配置脚，误接会干扰充电设置。
 *     D1      = P0.03    PIR 输出（OUT/VOUT）-> 数字输入，高电平触发
 *     D2      = P0.28    PIR 模块 VCC  -> GPIO 门控供电（高电平供电）
 *     D3      = P0.29    ⚠ **已关闭、不再使用**（DRV_PWR_GATE_ENABLE=0）：
 *                        DRV2605 模块 VCC 直接接 3.3V，固件不驱动 D3，
 *                        setup 里显式置成高阻输入（不供电、也不灌电流）。
 *     ⚠ 历史教训（别再按旧结论误判）：曾以为"模块 VCC 取自 D3"、把"D3 必须常高"
 *       当成铁律，并据此把"马达不震"归因于 D3 的处理方式 —— 现场最终确认是
 *       **接线接错**，与 D3 无关。所以出现"马达不震"时先量硬件（模块 VCC/GND 电压、
 *       SDA/SCL/GND 是否接对、模块电源灯亮不亮），代码侧只看日志 drv=on/off
 *       （on = I2C 探到 0x5A，芯片在线）。
 *       仍然成立的两点：① 同一个脚"一脚两用"（既当供电又输出别的电平）会干扰 I2C
 *       （实测芯片在非法地址 0x5B 应答）；② 模块被断电时 I2C 会被拉死，固件可能
 *       卡在总线等待（现象：串口一行日志都没有）。
 *       若将来要恢复 IO 门控：**马达电流绝不能走 IO**（单脚上限 15mA，
 *       LRA 额定 75mA），正解是 EN 门控或 P-MOS 高边开关。
 *
 *  ★ PIR 持续震动（新增）
 *     - PIR 输出**高电平**期间：马达**一直震动**（RTP 满强度持续输出）
 *     - PIR 变**低电平**：**立即停止**，不留余震
 *     - 触发期间红灯常亮；持续震动期间不叠加 FSR 的短震，避免互相打断
 *     - ⚠ nRF52840 的 IO 不耐 5V：PIR 模块请用 3V3 供电，或确认其输出为 3.3V
 *     - 上电预热保护：模块上电瞬间常输出一段有效电平，会误触发成
 *       "每次上电就震 2 秒"。已加保护（PIR_WARMUP_GUARD=1）：
 *       必须先读到一次无效电平，才允许接受触发。
 *
 *  ★ 开机静默（不震）
 *     - DRV2605 的自动校准会**物理驱动马达 1~2 秒**，因此默认不做
 *       （HAPTIC_AUTOCAL_ON_BOOT=0），上电时马达不动。
 *       需要校准时由手机 BLE 写 'A' 现场校一次。
 *
 *  ★ 关键设计：DRV2605 未接线时**不会阻塞**程序
 *     - I2C 扫描确认地址，失败则 drvReady=false，整体降级为纯"FSR 采集 + BLE
 *       曲线"，串口明确提示，方便先验证前半段链路
 *     - 每次振动前复查 drvReady，热插拔后敲 'I' 可在线重初始化
 *
 *  ★ 串口 115200
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
#include <Adafruit_DRV2605.h>
#include "power_config.h"        // 电源配置（广播策略 / 唤醒阈值 / 连接参数 / 指示灯开关）

/* ======================= 1. 可调参数 ======================= */

/* --- 传感器 --- */
#define FSR_PIN              A0        // P0.02
#define ADC_MAX              4095.0f   // nRF52840 ADC 12bit

/* 采样与滤波 */
/* ★ SAMPLE_HZ 曾是"固定 50Hz 连续采样"的节拍；现在采样周期改由**运行期可配的
 *   采样（RTC）唤醒周期**决定（power_config.h 的 SAMPLE_WAKE_PERIOD_MS，
 *   档位 1s/3s/5s/10s，BLE 命令 '1'/'3'/'5'/'0'）。这里仅作参考值保留
 *   （OS_RATIO/MEDIAN_WIN/EMA_ALPHA 都是"按采样点"计的，与它无关）。 */
#define SAMPLE_HZ            50        // 参考：连续采样时的等效采样率
#define OS_RATIO             8         // 硬件过采样次数（连读取平均）
#define MEDIAN_WIN           5         // 中值滤波窗口(奇数)，滤单点尖刺
#define EMA_ALPHA            0.25f     // 一阶低通系数，越小越平滑越迟钝

/* --- 阈值判定（滞回 + 去抖 + 不应期） --- */
#define TH_ON                1200      // 上升沿阈值：超过算"踩下"
#define TH_OFF               900       // 下降沿阈值：低于算"抬起"(须 < TH_ON)
#define DEBOUNCE_MS          60        // 状态需连续保持这么久才认可
#define REFRACTORY_MS        250       // 触发后不应期，防单次踩踏重复触发

/* --- 触觉反馈默认模式 --- */
/* 0 = 单震   1 = 双震   2 = 长震 */
#define HAPTIC_MODE_DEFAULT  0

/* --- DRV2605 执行器类型 --- */
#define MOTOR_ACTUATOR       0         // 0 = LRA（本次使用）  1 = ERM

/* --- DRV2605 驱动配置模式（决定"马达到底转不转"的关键开关） ---
 * 实测记录（务必保留，别再反复试错）：
 *   · ERM 参数驱动这颗 LRA（FEEDBACK bit7=0）  -> **能震，但很弱**（实测确认过）
 *   · LRA 闭环（bit7=1）+ 自动校准             -> 上电时会明显震 1~2 秒（校准必须驱动马达）
 *   · LRA 闭环 + **不做校准**                  -> 实测马达不动
 *     （闭环自动谐振引擎依赖校准产出的 A_CAL_COMP/A_CAL_BEMF 作基准）
 *
 *   0 = LRA 闭环 + 开机自动校准（**默认**，TI 参考流程）
 *       —— 最可能转得好；代价：每次上电会震 1~2 秒（校准无法静默）
 *   1 = LRA 闭环 + 不校准 —— 上电静默，但实测可能不转
 *   2 = LRA 开环（无需校准、静默；频率字段尚未核准确，仅作备选）
 *   3 = 保底回退：ERM 参数驱动（实测"能震但弱"，用来确认硬件通不通）
 *
 * ★ 现场实测补充（重要）：**模式 1（闭环、不校准）在硬件正常时是能正常驱动的**
 *   —— "VCC 接 D3" 的那两个版本就是这个配置，PIR 触发时马达确实在震。
 *   当时"不震"的真正原因是另一码事：待机断电后重新上电时 GPIO 驱动能力不足
 *   （已修：High-Drive + 重试）。所以默认用 1（开机静默，无校准噪音）；
 *   只有确实感觉震感偏弱/发闷时，才临时用 0 做一次校准对比。
 *
 * ★ 与模块供电门控的配合：模式 0 需要保住校准值，而**断电会清掉校准**，
 *   所以模式 0 下不给 DRV 模块断电（只写 STANDBY）。模式 1/2/3 都能正常断电省电。 */
#define HAPTIC_DRIVE_MODE       1

/* 开机是否自动校准：由上面的模式决定（模式 0 才校准） */
#define HAPTIC_AUTOCAL_ON_BOOT  (HAPTIC_DRIVE_MODE == 0)

/* --- BLE --- */
#define BLE_DEVICE_NAME      "XIAO-FSR-Haptic"
#define BLE_TX_POWER         4         // dBm
#define NOTIFY_HZ            20        // 通知频率，50Hz 采样每 2~3 点发一次

/* --- 串口调试输出 ---
 * 0 = 关闭全部调试打印（发布/实际使用时的默认值）
 * 1 = 打开，便于排查
 * 实现方式：代码里的 Serial.print/println/printf 已统一换成 DBG/DBGLN/DBGF
 * 三个宏，关闭时编译成空语句，**不产生任何串口输出**。
 * 置 1 时还会额外启用串口调试命令：H=紧循环5s  S=模拟PIR高电平5s
 * T=模拟PIR短脉冲300ms  P=读PIR引脚  C=读引脚配置  I=重初始化DRV。
 * 注：与 DEBUG_PLOT 相互独立；平时置 0。 */
#define DEBUG_SERIAL         0

/* 串口绘图数据开关（与上面的调试打印相互独立）
 * 1 = 只输出供绘图用的纯数值行，形如： raw:60 filt:61 peak:72
 *     —— 配合 tools/serial_plot.py 或 Arduino IDE 的"串口绘图器"看实时曲线，
 *        不会带出其它调试刷屏，适合只想看曲线的时候。
 * 0 = 不输出（生产默认）。
 * 注意：看手机端 nRF Connect 是看不到这种曲线的——BLE 发的是 9 字节二进制包，
 *       手机 App 不解析 uint16，只能按 HEX 罗列。曲线请在电脑侧看。 */
#define DEBUG_PLOT           1
#define PRINT_INTERVAL_MS    250       // 调试打开时的打印周期

#if DEBUG_SERIAL
  #define DBG(...)    Serial.print(__VA_ARGS__)
  #define DBGLN(...)  Serial.println(__VA_ARGS__)
  #define DBGF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)    do {} while (0)
  #define DBGLN(...)  do {} while (0)
  #define DBGF(...)   do {} while (0)
#endif

/* ---- [pwr]/[DRV] 电源与驱动诊断日志 ----
 * 定义必须放在文件靠前处：hapticInit() 等函数在文件前半部分也要用（比如
 * "I2C 在非法地址应答"这类关键诊断）。测量模式下连串口都不初始化，
 * 所以这里必须一起关掉，否则会对未初始化的 Serial 写数据。 */
#if PWR_LOG_ENABLE && !PWR_MEASURE_MODE
  #define PWRLOG(...)  Serial.printf(__VA_ARGS__)
#else
  #define PWRLOG(...)  do {} while (0)
#endif

/* --- PIR 人体感应触发（持续震动） ---
 * 行为需求：PIR 输出**高电平期间一直震动**，变低电平**立即停止**。
 *
 * 接线：PIR 的 OUT/VOUT -> XIAO 的 D1，VCC -> **D2**（GPIO 门控供电），GND -> GND
 * ★ 电压警告：nRF52840 的 IO **不耐 5V**。多数 HC-SR501 模块标称 5V 供电，
 *   但其 OUT 输出是 3.3V 电平，可直连 D1；若模块输出 5V（少数型号），
 *   必须加电平转换或分压后再接，否则会打坏引脚。
 *   AM312 等 3.3V 模块可直接用 3V3 供电。
 *
 * 上拉选择：默认 INPUT_PULLDOWN —— 悬空时读 LOW，不会误触发。
 *   若你的模块是开漏输出，改成 INPUT_PULLUP 并把 PIR_ACTIVE_HIGH 改为 0。 */
#define PIR_PIN              D1        // P0.03（D0/A0 已被 FSR 占用）
#define PIR_ACTIVE_HIGH      1         // 1 = 高电平触发（HC-SR501/AM312 默认）
#define PIR_DEBOUNCE_MS      30        // 状态去抖，防抖动误触发
#define PIR_HOLD_REFRESH_MS  50        // 持续震动期间重写 RTP 的周期（流式保输出）
#define PIR_MAX_HOLD_MS      0         // 0 = 不限制（按需求"直到低电平"）;
                                       // >0 则最长持续这么久，用于保护马达

/* ---------- 模块供电门控（GPIO 直接给模块上电 / 断电） ----------
 * 实测接线：D2 -> PIR 模块 VCC，D3 -> DRV2605 模块 VCC（高电平供电）。
 *
 * 为什么需要它：模块自身（电源指示灯、LDO、上拉）常有 mA 级静态电流，
 *   这部分**固件里省不掉**（芯片进 STANDBY 也没用），只能断掉它的 VCC。
 *
 * ⚠⚠ 一条硬限制，务必确认后再用：**马达电流绝不能走 IO 脚**
 *   nRF52840 单脚输出上限 15mA，而本 LRA 额定 75mA、启动瞬时 100mA。
 *   如果这颗"DRV2605 模块 VCC"同时给**输出级（H 桥）**供电，那么震动瞬间
 *   电流会从 D3 流出 → 电压塌陷、震感变弱、长期可能损伤引脚。两种正确做法：
 *     (a) 模块 VCC 直接接 3V3，把 **EN 脚**接到 D3 做门控（EN=0 时芯片 <1µA，
 *         且 EN 是输入脚，不流过马达电流）；
 *     (b) 在模块 VCC 线上串一颗 P-MOS，栅极由 D3 控制（马达电流走 MOS 不走 IO）。
 *   只有在"该 VCC 只供逻辑、马达另有独立电源脚"时，才可以像现在这样直连 IO。
 */
#define PIN_PIR_VCC          D2        // PIR 模块 VCC（高电平供电）
#define PIN_DRV_VCC          D3        /* ⚠ **已关闭、不再使用**（DRV_PWR_GATE_ENABLE=0）：
                                        *   固件不驱动它，setup 里显式置成高阻输入。
                                        *   DRV2605 模块 VCC 改由 3.3V 供电，不再需要这根线；
                                        *   保留宏定义只是方便将来若恢复 IO 门控时改回来。 */

/* 两个模块的门控**分开开关**，避免"只想验证一个，却把另一个也断电/也驱动"。
 *   PIR_PWR_GATE_ENABLE = 1：D2 给 PIR 模块供电（当前接法）
 *   DRV_PWR_GATE_ENABLE = 0：D3 **关闭**（模块 VCC 直连 3.3V，不用 IO 供电） */
/* ★★ 历史教训（保留，但结论已更正 —— 别再按旧结论误判）★★
 *   曾有一段时间认为"DRV2605 模块的 VCC 取自 D3"，于是把"D3 必须常高"写成了
 *   铁律，并据此把"马达不震"归因于 D3 的处理方式。
 *   **后来现场确认：马达不震是接线接错造成的，与 D3 无关。**
 *   教训：出现"马达不震"时，**先量硬件**（模块 VCC/GND 电压、SDA/SCL/GND 是否接对、
 *   模块电源灯亮不亮），代码侧只看日志 `drv=on/off`（on = I2C 探到 0x5A、芯片在线）。
 *   历史确实观测到过、仍值得记住的两点：
 *     · 拿这个脚兼做别的用途（既当供电又输出别的电平）时出现过 I2C 干扰
 *       （芯片在非法地址 0x5B 应答）—— 所以"一脚两用"要避免；
 *     · 模块被断电时 I2C 会被拉死，固件可能卡在总线等待（现象：零日志）。
 *   因此现在干脆**不用 D3**：模块 VCC 直连 3.3V，最干净。
 *   ⚠ 若将来要恢复 IO 门控：**马达电流绝不能走 IO**（单脚上限 15mA，
 *     LRA 额定 75mA），正解是 EN 门控或 P-MOS 高边开关。 */
#define PIR_PWR_GATE_ENABLE  1
#define DRV_PWR_GATE_ENABLE  0        /* ★ D3 已关闭：模块 VCC 直连 3.3V，固件不驱动 D3 */

/* DRV2605 模块供电是否"始终不断"（DRV_PWR_GATE_ENABLE=0 时本宏已无实际作用：
 * 模块始终由 3.3V 供电，固件只控制芯片进不进 STANDBY）：
 *   1 = **不切断模块供电**，待机时只把芯片写回 STANDBY（芯片自身 ~1µA）。
 *       好处：① 唤醒时不需要重新初始化，触发即震、无 0.1s 延迟；
 *             ② 不存在"断电-上电"导致的初始化失败/校准丢失。
 *   0 = 待机时拉低 D3 给模块断电（仅当 DRV_PWR_GATE_ENABLE=1 且确实由 D3 供电才有意义）。
 * ★ 注意：1 时省不到模块上 LDO/电源灯的 mA 级静态电流（只能靠改硬件）。 */
#define DRV_PWR_KEEP_ON      1
#define DRV_PWR_SETTLE_MS    60        // 上电后等模块 LDO/上拉/电容稳定再跑 I2C
#define DRV_PWR_RETRY        3         // 上电后初始化失败时的重试次数（每次间隔同上）

/* 待机慢广播（IDLE_ADV）期间是否也给 PIR 断电：
 *   0 = 常供电（默认）：PIR 是"有人就震"的触发源，断电就收不到触发了
 *   1 = 待机断电：省电/做电流验收时用；代价是待机期间只有 FSR 能触发 */
#define PIR_PWR_OFF_IN_IDLE  0

/* PIR 预热保护（专门防"每次上电马达就震一下/几秒"）：
 * 多数 PIR 模块上电瞬间会输出一段有效电平（HC-SR501 预热可达 30~60s，
 * AM312 约 2s 左右）。若直接采信，就会表现成"设备一上电马达自己震起来"。
 *   1 = 本次开机后**必须先读到一次无效电平**，才允许接受有效电平触发（默认）
 *   0 = 关闭保护（仅当传感器输出确实稳定干净时才关）
 * 说明：如果模块被短路成常有效电平，保护开启会导致一直不触发——
 *       但那本来就是故障状态，无保护时表现会是"马达常震不停"，更糟。 */
#define PIR_WARMUP_GUARD     1

/* 持续震动（PIR 触发）的 RTP 强度：0~0x7F（127）。
 * 0x7F = 满强度。若觉得"太剧烈"，往下调，例如 0x50（约 63%）；
 * 注意持续震动比短震更容易发热，长时间场景建议适当降低。 */
#define HAPTIC_CONT_STRENGTH 0x7F

/* PIR 触发后"最短持续震动时长"。0 = 完全跟随引脚（低电平立即停）。
 *
 * 实测结论：本工程的 PIR 检测到人后**只输出一个很短的脉冲**（不是持续高电平），
 * 所以纯跟随引脚就变成"只震一下"。设为 3000 后，行为变为
 *   "触发后至少震 3 秒；若模块本来能长期保持高电平，则继续震到变低为止"，
 * 兼顾"防抖一下就没"和"人在就一直震"。
 * 硬件侧若把模块跳线拨到 H（可重复触发）也能解决，两者可任选。 */
#define PIR_MIN_HOLD_MS      3000

/* 调试期临时功能：每次 PIR 松开后，用红灯闪烁次数把"本次持续了多久"
 * 编码输出（每 500ms 计一次，最多 20 次），无需串口即可判断模块行为。
 * 生产环境置 0 关闭（避免与"最短持续时长"叠加后每次固定闪同样次数）。 */
#define PIR_REPORT_BLINK     0

/* ======================= 2. BLE UUID（照例程风格） ======================= */
/* 例程用的前缀 4fafc201-...-c5c9c331914b / beb5483e-...-ea07361b26a8，
 * 这里保持"同一前缀 + 末位递增"的风格，改造成 FSR 系统专用的四个特征。 */

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_DATA_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // Notify 压力数据
#define CHAR_CMD_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // Write  振动命令
#define CHAR_THRESH_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26aa"  // Write  阈值设置
#define CHAR_STATUS_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26ab"  // Notify 状态文本
#define CHAR_PLOT_FILT_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ac"  // Notify 单值：滤波后压力
#define CHAR_PLOT_RAW_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26ad"  // Notify 单值：原始压力

BLEService        fsrService(SERVICE_UUID);
BLECharacteristic dataChar(CHAR_DATA_UUID);
BLECharacteristic cmdChar(CHAR_CMD_UUID);
BLECharacteristic threshChar(CHAR_THRESH_UUID);
BLECharacteristic statusChar(CHAR_STATUS_UUID);

/* ★ 单值特征（专门给手机 App 画曲线用）
 * 手机端 BLE App 的绘图功能只能画"一个数值"，而 dataChar 是 9 字节混合包
 * （raw+filt+peak+2 个标志位），App 无法把它当成曲线数据。
 * 所以这里额外提供两条**只含 2 字节小端 uint16** 的特征：
 *   末位 ac = 滤波后压力（画曲线用这条）
 *   末位 ad = 原始压力（对比用）
 * 在 App 里把该特征的值格式选成 UINT16 (little-endian) 即可看到曲线。 */
BLECharacteristic plotFiltChar(CHAR_PLOT_FILT_UUID);
BLECharacteristic plotRawChar(CHAR_PLOT_RAW_UUID);

/* 数据包：9 字节，小端，按固定长度发，nRF Connect 可直接解析 */
typedef struct __attribute__((packed)) {
  uint16_t raw;        // 原始 ADC(过采样平均)
  uint16_t filt;       // 中值+EMA 滤波值
  uint16_t peak;       // 峰值保持
  uint8_t  pressed;    // 0/1 当前按压状态
  uint8_t  drv_ok;     // 0/1 DRV2605 是否在线
  uint8_t  reserved;   // 对齐备用
} FsrPacket;

/* ======================= 3. 全局状态 ======================= */

/* 不实例化库对象：芯片地址可能不是库硬编码的 0x5A，
 * 这里只用库头文件里的寄存器宏定义，读写全部走自己的 drvWrite/drvRead。 */
bool drvReady = false;
/* 本次上电内是否已做过自动校准：校准要驱动马达 1~2 秒，只能做一次。
 * （校准值掉电即失；模式 0 下不给模块断电就是为了保住它，见 HAPTIC_DRIVE_MODE） */
bool s_drvCalDone = false;

/* 滤波缓冲 */
uint16_t medBuf[MEDIAN_WIN];
uint8_t  medIdx   = 0;
uint8_t  medCount = 0;

float    emaVal  = 0.0f;
uint16_t rawVal  = 0;
uint16_t filtVal = 0;
uint16_t peakVal = 0;

bool     pressed     = false;
bool     rawAbove    = false;
uint32_t lastChange  = 0;
uint32_t lastTrigger = 0;

uint16_t thOn  = TH_ON;
uint16_t thOff = TH_OFF;
uint8_t  hapticMode = HAPTIC_MODE_DEFAULT;

uint32_t lastSample = 0;
uint32_t lastNotify = 0;
uint32_t lastPrint  = 0;

/* PIR 状态机 */
bool     pirRaw        = false; // 去抖前的引脚状态（已按有效电平折算）
bool     pirActive     = false; // 去抖后确认的状态：true = 正在持续震动
bool     pirPendingStop = false;// 引脚已低但未到"最短持续时长"，等时到再停
uint32_t pirChangeAt  = 0;      // 引脚状态最后一次变化的时刻（用于去抖）
uint32_t pirHoldStart = 0;      // 本次持续震动的起始时刻
uint32_t lastPirRef   = 0;      // 上次刷新 RTP 强度的时刻
uint32_t lastPirDbg   = 0;      // 上次打印 PIR 监控信息的时刻（仅调试用）
uint32_t pirSimUntil  = 0;      // 调试：软件模拟 PIR 高电平到该时刻（验证 PIR 代码路径）
bool     pirSeenIdle  = false;  // 本次开机后是否读到过"无效电平"（PIR 预热保护）
bool     pirWarmupWarn = false; // 预热忽略只提示一次，避免刷屏

bool deviceConnected = false;

/* 前向声明 */
void setStatus(const char* msg);

/* 电源状态机（实现在 5.5 节；这里前向声明，供前面的触觉代码调用） */
void pwrActivity(void);
void pwrEnterState(pwr_state_t st, const char* why);
void pwrHapticBusy(bool busy);

/* 模块供电（实现在 5.5 节）：触觉函数在"待机时已断 VCC"的情况下要先上电，
 * 所以这里前向声明。已上电时调用是**幂等**的，直接返回，不会重复初始化。 */
bool drvPowerOn(void);
void drvPowerOff(void);
void pirPowerOn(void);
void pirPowerOff(void);

/* I2C 安全探测（实现在 5.5 节）：软件位翻转，永不卡死。
 * hapticInit() 里要用，所以这里前向声明。 */
static bool i2cProbeBitbang(uint8_t addr7);
static bool i2cBusIdle(void);

/* ======================= 4. DRV2605 触觉 ======================= */
/* RTP(实时播放) 模式库没封装，这里直接写寄存器。
 * 寄存器地址一律用库 Adafruit_DRV2605.h 的宏，不要自己重定义
 * （手抄极易错位：OVERDRIVE=0x0D 不是 0x0F，BREAK=0x10 不是 0x12） */
#define DRV2605_REG_MODE  0x01   // MODE 寄存器，库未定义

/* ★ DRV2605L 的 I2C 地址是**固定 0x5A**，芯片没有 ADDR/地址选择引脚。
 * VSSOP-10(DGS) 封装全部引脚只有：VREG / SCL / SDA / IN-TRIG / EN /
 * VDD / OUT+ / GND / OUT-。所以 0x5B、0x5F 都不是它的合法地址。
 *
 * 这里保留 0x5B/0x5F 的探测**不是为了兼容**，而是因为：当它们应答时，
 * 恰恰证明 I2C 传输本身不可靠（地址位/ACK 位被误判），此时所有寄存器
 * 写入都会静默失败，表现为 MODE 恒为默认值 0x40。 */
#define DRV2605_ADDR_ALT  0x5B   // 非法地址：仅用于诊断总线异常
#define DRV2605_ADDR_SUS  0x5F   // 非法地址：仅用于诊断总线异常

/* 实际地址：DRV2605L 固定 0x5A（无地址引脚），启动时仍做一次探测并
 * 保留 0x5B 兼容，避免不同批次模块造成"永远找不到"。
 *
 * 注意：Adafruit_DRV2605 库把 0x5A 硬编码在 begin() 内
 * （库源码里 new Adafruit_I2CDevice(DRV2605_ADDR, theWire)），
 * 芯片若实际在 0x5B，drv.begin() 必然失败。
 * 因此不实例化库对象，自己按探测到的地址做寄存器初始化与播放。
 *
 * ★★ IN/TRIG 引脚悬空 = 全线崩溃（本项目踩过的最大坑，务必保留此注释）★★
 * 实测现象：IN/TRIG 悬空时
 *   ① 芯片会在**非法地址 0x5F** 应答（DRV2605L 不可能有 0x5F）
 *   ② 全部 12 个寄存器**写不进**（回读恒为上电默认值，MODE 恒 0x40）
 *   ③ 读寄存器却完全正常（STATUS=0xE0、FEEDBACK=0x36 都能读）
 * 这三个现象组合极具欺骗性，极易误判为"EN 未拉高""ADDR 悬空""芯片非正品"
 * 而反复怀疑硬件。实际原因只有一个：**IN/TRIG 未按手册要求接地**。
 * 把 IN/TRIG 接 GND 后，地址立刻回到 0x5A、12/12 寄存器全部可写。
 * 结论：IN/TRIG 不使用时必须接 GND，这不是"可选建议"。 */
uint8_t drvAddr = DRV2605_ADDR;

/* 累计 I2C 写失败次数（endTransmission 返回非 0 时递增）。
 * 原实现忽略了返回值，导致"写入静默失败"极难察觉——
 * 实测遇到过一次：芯片所有寄存器都写不进，但日志上完全看不出异常。 */
uint16_t i2cWriteErr = 0;

bool drvWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(drvAddr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t e = Wire.endTransmission();
  if (e) i2cWriteErr++;
  return (e == 0);
}

uint8_t drvRead(uint8_t reg) {
  Wire.beginTransmission(drvAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)drvAddr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

/* ⚠⚠ 不要用这个函数做"总线是否可用"的探测（务必保留此警告）⚠⚠
 *   它走核心库的 Wire，而 Wire 的底层 while 循环**没有超时**：一旦 SDA 被
 *   从机拉住（典型原因：从机没供电但被 I2C 线半供电），这些循环会永远转下去，
 *   整个固件卡死（现象：串口无输出、BLE 不广播，看着像板子坏了）。
 *   初始化路径请一律使用下面的 i2cProbeBitbang()（软件位翻转，永不等待）。 */
bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

/* ---------- 执行一次 DRV2605 自动校准 ----------
 * ⚠ 这个动作**会真的驱动马达震动约 1~2 秒**，无法静默：校准是靠马达反电动势
 *   测谐振点（A_CAL_COMP / A_CAL_BEMF）的，TI 参考流程与 Linux 内核 drv260x
 *   驱动都要求在 LRA 闭环下执行。
 * 是否在开机时调用由 HAPTIC_AUTOCAL_ON_BOOT 决定（默认 0 = 上电不震），
 * 也可由手机 BLE 写 'A' 现场调用一次。 */
static void drvRunAutoCal(void) {
  PWRLOG("[DRV] 自动校准开始（此刻马达应震动 1~2 秒；不震说明驱动/马达那段有问题）\n");
  drvWrite(DRV2605_REG_CONTROL3, 0x48);        // NG_THRESH_2 | RTP_UNSIGNED_DATA
  drvWrite(DRV2605_REG_FEEDBACK, 0xB8);        // LRA_MODE | BRAKE_4X | LOOP_GAIN_HIGH
  drvWrite(DRV2605_REG_CONTROL4, 0x20);        // AUTOCAL_TIME 500ms
  drvWrite(DRV2605_REG_MODE, 0x07);            // MODE = AUTO_CAL
  delay(20);
  drvWrite(DRV2605_REG_GO, 0x01);              // 置 GO 启动校准
  /* 轮询 GO 位直到硬件清零（校准完成），上限 1.5s 兜底 */
  uint32_t t0 = millis();
  while ((drvRead(DRV2605_REG_GO) & 0x01) && (millis() - t0) < 1500UL) delay(10);
  delay(30);
  drvWrite(DRV2605_REG_MODE, 0x00);            // 退出校准模式
  PWRLOG("[DRV] 自动校准结束 A_CAL_COMP=0x%02X A_CAL_BEMF=0x%02X STATUS=0x%02X\n",
         drvRead(DRV2605_REG_AUTOCALCOMP), drvRead(DRV2605_REG_AUTOCALEMP),
         drvRead(DRV2605_REG_STATUS));
}

/* DRV2605 寄存器初始化（等价库 init()，但走可变的 drvAddr）
 *
 * ★ 关键：必须先发 DEV_RESET 软复位，再写 MODE 清 STANDBY。
 *   实测经验：上电后直接写 MODE=0x00 会**写不进去**（回读恒为 0x40，
 *   bit6 STANDBY 清不掉），现象极易被误判为"EN 引脚没拉高/芯片坏了"。
 *   原因是芯片上电后内部状态机尚未就绪，必须先软复位让它重新走一遍
 *   上电序列，之后 MODE 才可写。加上这一步后 MODE 能稳定读回 0x00。 */
static void drvRegInit() {
  drvWrite(DRV2605_REG_MODE, 0x80);        // bit7 = DEV_RESET 软复位
  delay(10);
  /* 等复位自清：bit7 由硬件清零，最长约 1ms，这里给足余量 */
  for (int i = 0; i < 10 && (drvRead(DRV2605_REG_MODE) & 0x80); i++) delay(5);
  delay(30);                               // 等内部 LDO/振荡器稳定

  drvWrite(DRV2605_REG_MODE, 0x00);        // 退出 standby
  drvWrite(DRV2605_REG_RTPIN, 0x00);       // 关闭实时播放
  drvWrite(DRV2605_REG_WAVESEQ1, 1);       // 序列槽0: strong click
  drvWrite(DRV2605_REG_WAVESEQ2, 0);       // 槽1: 0 = 序列结束
  drvWrite(DRV2605_REG_OVERDRIVE, 0x00);
  drvWrite(DRV2605_REG_SUSTAINPOS, 0x00);
  drvWrite(DRV2605_REG_SUSTAINNEG, 0x00);
  drvWrite(DRV2605_REG_BREAK, 0x00);
  drvWrite(DRV2605_REG_AUDIOMAX, 0x64);

  /* 幅度基准：电压换算按内核驱动公式 V = value * 5600mV / 255
   *   RATEDV(0x16) = 额定电压，闭环驱动幅度按它算
   *   CLAMPV(0x17) = 过驱钳位，输出幅度上限
   * 本工程 LRA 额定 3.0V（8x3.2mm、谐振 200Hz、3.3V 下 0.5g）：
   *   0x89 = 137 -> 137*5600/255 ≈ 3009 mV
   *   0x90 = 144 -> 144*5600/255 ≈ 3162 mV（略超额定，用于启动冲量） */
  drvWrite(DRV2605_REG_RATEDV, 0x89);      // 额定电压 3.0V
  drvWrite(DRV2605_REG_CLAMPV, 0x90);      // 过驱钳位 ~3.16V

  if (MOTOR_ACTUATOR == 0) {
    /* ================= LRA =================
     * ★★ 位定义以 Linux 内核 drv260x.c（TI 工程师维护）为准 ★★
     *   FEEDBACK(0x1A) bit7 = FB_REG_LRA_MODE：**1 = LRA，0 = ERM**
     *   CONTROL3(0x1D) bit0 = LRA_OPEN_LOOP
     *                  bit3 = RTP_UNSIGNED_DATA
     *                  bit5 = ERM_OPEN_LOOP
     *                  bit7:6 = NG_THRESH
     *
     * 踩过的坑（务必别再改回去）：
     *   原代码写 FEEDBACK=0x36（bit7=0）——**那是 ERM 模式**，
     *   等于把 200Hz 线性马达当转子马达驱动。症状极具欺骗性：
     *   寄存器读回全部正常、MODE 保持 RTP、STATUS 无保护位，
     *   但马达**只在起始时抽动一下，无法持续振动**（"就动了一下"）。
     *   而且当时按"LRA/ERM 两种配置"做的对比测试里，两组都写 bit7=0，
     *   所以"ERM 配置"那组毫无区别，对比结论也就失去了意义。
     *
     * 关于自动校准：TI 参考流程是"先校准再写工作参数"，闭环谐振跟踪依赖
     * 校准产出的 A_CAL_COMP/A_CAL_BEMF 作基准；校准**必然驱动马达 1~2 秒**
     * （靠反电动势测谐振点，无法静默）。是否校准由 HAPTIC_DRIVE_MODE 决定，
     * 不要为了"开机静默"随手改成不校准 —— 实测那样马达会彻底不转。 */
#if HAPTIC_DRIVE_MODE == 3
    /* ---- 保底回退：ERM 参数驱动（实测"能震但很弱"）----
     * 用来确认"硬件通路到底通不通"，不追求震感。 */
    drvWrite(DRV2605_REG_CONTROL3, 0xA0);        // 含 ERM_OPEN_LOOP
    drvWrite(DRV2605_REG_FEEDBACK, 0x36);        // bit7=0 -> ERM 模式
    drvWrite(DRV2605_REG_CONTROL1, 0x93);
    drvWrite(DRV2605_REG_CONTROL2, 0xF5);
    drvWrite(DRV2605_REG_CONTROL4, 0x20);
#else
  #if HAPTIC_DRIVE_MODE == 0
    /* 自动校准（本次上电只做一次）。★ 会驱动马达 1~2 秒，这是校准的固有代价。 */
    if (!s_drvCalDone) {
      drvRunAutoCal();
      s_drvCalDone = true;
    }
  #endif
    /* 正式工作参数（内核 lra_init 表；未置 ANALOG_IN 以免影响已接地的 IN/TRIG） */
    drvWrite(DRV2605_REG_FEEDBACK, 0x97);        // LRA_MODE | BRAKE_2X | LOOP_GAIN_MED | BEMF_GAIN_3
    drvWrite(DRV2605_REG_CONTROL1, 0x80);        // STARTUP_BOOST
    drvWrite(DRV2605_REG_CONTROL2, 0x20);        // SAMPLE_TIME 250us
  #if HAPTIC_DRIVE_MODE == 2
    drvWrite(DRV2605_REG_CONTROL3, 0x48 | 0x01); // + LRA_OPEN_LOOP(bit0)：不需校准
  #else
    drvWrite(DRV2605_REG_CONTROL3, 0x48);        // 闭环（不置 LRA_OPEN_LOOP/ERM_OPEN_LOOP）
  #endif
    drvWrite(DRV2605_REG_CONTROL4, 0x20);        // AUTOCAL_TIME 500ms
#endif
  } else {
    /* ================= ERM =================
     * ERM 模式 = FEEDBACK bit7 清零，并打开 ERM 开环（CONTROL3 bit5）。 */
    drvWrite(DRV2605_REG_CONTROL3, 0x48 | 0x20); // NG_THRESH_2 | ERM_OPEN_LOOP
    drvWrite(DRV2605_REG_FEEDBACK, 0x26);        // ERM：BRAKE_3X | LOOP_GAIN_MED | BEMF_GAIN_2
    drvWrite(DRV2605_REG_CONTROL2, 0x25);
#if HAPTIC_AUTOCAL_ON_BOOT
    drvRunAutoCal();                             // ★ 会驱动马达 1~2 秒
#endif
    drvWrite(DRV2605_REG_CONTROL1, 0x80);
  }
}

/* 初始化 DRV2605；失败返回 false，调用方降级处理 */
bool hapticInit() {
  /* ★★ 地址探测必须重试（这是"马达时好时坏"的直接原因，务必保留）★★
   *   实测同一块板子：有的开机能探测到 0x5A（drv=on），有的开机却报
   *   "0x5B 应答"（drv=off）。而 DRV2605L 的地址**固定 0x5A**，根本没有
   *   0x5B/0x5F 这两个地址 —— 也就是说那是 ACK 位被误判，属于总线边缘抖动，
   *   不是"芯片换地址了"。
   *   所以这里改成：对 0x5A 连续探测 5 次；若只看到非法地址应答，
   *   说明总线抖动但芯片大概率就在 0x5A，再给它 10 次机会，
   *   并且**始终只用 0x5A 通信**（不再把非法地址当成有效地址使用）。 */
  drvAddr = DRV2605_ADDR;                       // 唯一合法地址
  /* ★ 探测一律用 bit-bang 版本：它绝不会卡死（Wire 在脏总线上会无限等） */
  bool found = false;
  for (uint8_t i = 0; i < 5 && !found; i++) {
    found = i2cProbeBitbang(DRV2605_ADDR);
    if (!found) delay(10);
  }
  if (!found && (i2cProbeBitbang(DRV2605_ADDR_ALT) || i2cProbeBitbang(DRV2605_ADDR_SUS))) {
    PWRLOG("[DRV] !! 非法地址 0x5B/0x5F 应答 = ACK 误判（总线抖动）"
           "，仍按 0x5A 重试 10 次...\n");
    for (uint8_t i = 0; i < 10 && !found; i++) {
      delay(20);
      found = i2cProbeBitbang(DRV2605_ADDR);
    }
  }
  /* ★ 交叉验证（防止"bit-bang 探测本身有 bug 把好芯片误判成不在线"）：
   *   bit-bang 说没有设备、但总线电平是干净的（sda=scl=1）时，用核心库的 Wire
   *   再确认一次。总线干净时 Wire 不会卡死，所以这次交叉验证是安全的。
   *   若 Wire 能探测到 0x5A，说明芯片其实在，只是探测方式有差异 -> 继续初始化。 */
  if (!found && i2cBusIdle()) {
    Wire.begin();
    Wire.setClock(100000);
    const bool byWire = i2cProbe(DRV2605_ADDR);
    /* ★ 这里**绝对不能** Wire.end()：后面 drvRegInit() 还要用 Wire，
     *   而 TWIM 一旦被 disable，TASKS_STARTTX 就再也不会产生事件 ->
     *   核心库那些 while 会永远等下去（固件卡死）。 */
    if (byWire) {
      PWRLOG("[DRV] 交叉验证：bit-bang 未探测到，但 Wire 探测到 0x5A 应答"
             " -> 以 Wire 结果为准，继续初始化\n");
      found = true;
    } else {
      PWRLOG("[DRV] 交叉验证：Wire 也未探测到 0x5A（两种方式一致）\n");
    }
  }

  if (!found) {
    /* 顺带把总线电平打出来（不用万用表也能定位一大半问题）：
     *   sda/scl = 1 -> 线被上拉，正常空线
     *   sda/scl = 0 -> 该线被拉住/接到 GND/接错脚（这时模块不可能应答）
     *   sda=1 scl=1 但仍无 ACK -> 模块没供电（其内部上拉失效）或 SDA/SCL 根本没接上，
     *                              也可能是模块已损坏/没焊接好 */
    pinMode(SDA, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(50);
    const int sdaL = (int)digitalRead(SDA);
    const int sclL = (int)digitalRead(SCL);

    static bool warned = false;              // 详细提示只在本次上电打印一次，避免刷屏
    if (!warned) {
      warned = true;
      PWRLOG("[DRV] 0x5A 探测失败（总线电平 sda=%d scl=%d，1=被上拉 0=被拉住）\n",
             sdaL, sclL);
      PWRLOG("[DRV] 请依次检查：\n");
      PWRLOG("[DRV]   1) 模块 VCC/GND 电压是否 3.3V、GND 是否通"
             "（模块 VCC 应直连 3.3V；D3 已关闭，不参与供电）\n");
      PWRLOG("[DRV]   2) SDA->D4(P0.04) / SCL->D5(P0.05) 是否接对、是否松动\n");
      PWRLOG("[DRV]   3) IN/TRIG 必须接 GND；VCC 就近加 10uF\n");
      PWRLOG("[DRV]   4) 马达 OUT+/OUT- 到模块是否通（开路=马达不震）\n");
      i2cScanLog();
      PWRLOG("[DRV] 降级为纯 FSR+BLE 模式（BLE/采样照常工作）\n");
    } else {
      PWRLOG("[DRV] 探测仍失败（sda=%d scl=%d）\n", sdaL, sclL);
    }
    return false;
  }
  PWRLOG("[DRV] 0x5A 探测成功\n");

  drvRegInit();

  /* 回读校验：写不进去说明型号不对或总线异常 */
  uint8_t st = drvRead(DRV2605_REG_STATUS);
  uint8_t fb = drvRead(DRV2605_REG_FEEDBACK);
  uint8_t c3 = drvRead(DRV2605_REG_CONTROL3);
  uint8_t md = drvRead(DRV2605_REG_MODE);
  if (fb == 0xFF && c3 == 0xFF) {
    DBGF("[DRV] 0x%02X 寄存器读全 0xFF，疑似非 DRV2605 芯片\n", drvAddr);
    return false;
  }

  /* ★ 写后回读校验（关键）：用刚写入的值反查写入是否真的生效。
   * 芯片能应答 I2C、能正常读寄存器，并不代表写入能生效。
   * 尤其要查 RATEDV/CLAMPV —— 它们决定震感大小，一旦没写进就会"能震但很弱"。 */
  uint8_t rv = drvRead(DRV2605_REG_RATEDV);
  uint8_t cv = drvRead(DRV2605_REG_CLAMPV);
  uint8_t c1 = drvRead(DRV2605_REG_CONTROL1);
  /* CONTROL3 期望值随 HAPTIC_DRIVE_MODE 不同（0x48 闭环 / 0x49 开环 / 0xA0 ERM） */
  bool writeOk = (MOTOR_ACTUATOR == 0) ? ((c3 & 0x48) == 0x48) : ((c3 & 0x20) != 0);
  if (!writeOk) {
    DBGF("[DRV] !! 写入未生效: 回读 FEEDBACK=0x%02X CONTROL3=0x%02X,"
                  " 累计 I2C 写失败 %u 次\n", fb, c3, i2cWriteErr);
    /* 症状与成因（本项目实测踩过，保留备查）：
     * 写不进 + 只在非法地址 0x5F 应答 + 读寄存器正常
     *   => 99% 是 IN/TRIG 引脚悬空，不是芯片坏、不是 EN、不是 ADDR。
     * 处理：IN/TRIG 接到 GND，之后 12/12 寄存器立刻全部可写。 */
    DBGLN("[DRV] 排查顺序：① IN/TRIG 是否已接 GND（最常见）"
                   " ② EN 是否 >=1.8V ③ VDD 是否 >=2.0V");
    return false;
  }
  /* ★ 执行器模式校验：LRA 必须让 FEEDBACK bit7 = 1。
   * 这一位曾经写错（bit7=0 = ERM），导致 200Hz 线性马达被当转子马达驱动，
   * 症状是"只在起始抽动一下、无法持续振动"，且寄存器读回全正常，极难发现。 */
#if HAPTIC_DRIVE_MODE != 3
  /* 模式 3 是故意走 ERM 参数（保底验证用），所以只在其它模式下告警 */
  if (MOTOR_ACTUATOR == 0 && !(fb & 0x80)) {
    DBGF("[DRV] !! FEEDBACK=0x%02X 的 bit7=0，芯片被配成 **ERM 模式**，"
                  "而本工程是 LRA —— 会表现成\"只抽动一下/很弱\"\n", fb);
  }
#endif
  if (MOTOR_ACTUATOR == 0 && rv != 0x89) {
    DBGF("[DRV] !! 额定电压未生效: RATEDV=0x%02X(应 0x89≈3.0V)\n", rv);
  }

  if (md & 0x40) {
    /* 软复位后仍锁在 STANDBY：一般是 EN 未拉高、供电不足或马达未接 */
    DBGF("[DRV] 警告：MODE=0x%02X 仍处 STANDBY，检查 EN 引脚是否拉高、"
                  "VDD 是否 >=2.0V、LRA 是否已接\n", md);
  }
  /* LRARESON(0x22)/VBAT(0x21) 是只读回读值，仅作诊断打印：
   * 前者反映芯片实际锁定/使用的 LRA 周期（本工程马达谐振 200Hz），
   * 后者反映供电电压水平。原始值不做单位换算，避免凭记忆误读。 */
  uint8_t reson = drvRead(DRV2605_REG_LRARESON);
  uint8_t vbat  = drvRead(DRV2605_REG_VBAT);
  DBGF("[DRV] ok @0x%02X. STATUS=0x%02X MODE=0x%02X FEEDBACK=0x%02X(bit7=%d) "
                "CONTROL1=0x%02X CONTROL3=0x%02X RATEDV=0x%02X CLAMPV=0x%02X (%s)\n",
                drvAddr, st, md, fb, (fb >> 7) & 1, c1, c3, rv, cv,
                (MOTOR_ACTUATOR == 0) ? "LRA" : "ERM");
  DBGF("[DRV] 校准值 A_CAL_COMP=0x%02X A_CAL_BEMF=0x%02X\n",
                drvRead(DRV2605_REG_AUTOCALCOMP), drvRead(DRV2605_REG_AUTOCALEMP));
  DBGF("[DRV] 回读诊断: LRARESON=0x%02X(原始) VBAT=0x%02X(原始)\n",
                reson, vbat);
  return true;
}

/* RTP 实时播放：可自定义任意时长/强度的长震 */
void hapticRTP(uint8_t strength, uint16_t ms) {
  if (!drvReady && !drvPowerOn()) return;   // 待机时模块已断电 -> 先上电
  drvWrite(DRV2605_REG_RTPIN, strength);   // 0~127
  drvWrite(DRV2605_REG_MODE, 0x05);        // RTP 模式
  drvWrite(DRV2605_REG_GO, 0x01);
  delay(ms);
  drvWrite(DRV2605_REG_RTPIN, 0x00);
  drvWrite(DRV2605_REG_GO, 0x00);
  delay(5);
  drvWrite(DRV2605_REG_MODE, 0x00);        // 回内部触发模式
}

/* ROM 波形播放（单次），不依赖库，走 drvAddr */
void hapticEffect(uint8_t effect) {
  if (!drvReady && !drvPowerOn()) return;   // 待机时模块已断电 -> 先上电
  drvWrite(DRV2605_REG_MODE, 0x00);              // 内部触发模式
  drvWrite(DRV2605_REG_LIBRARY, 6);              // 库 6 = LRA 波形
  drvWrite(DRV2605_REG_WAVESEQ1, effect);
  drvWrite(DRV2605_REG_WAVESEQ2, 0);             // 序列结束
  drvWrite(DRV2605_REG_GO, 0x01);
}

/* ---------- 现场校准（BLE 命令 'A'）----------
 * HAPTIC_AUTOCAL_ON_BOOT = 0 时开机不校准（所以上电静默）。
 * 若发现震感偏弱/发闷，可连手机往 FEE2(末位a9) 写 'A' 现场校一次：
 * ⚠ 这一次会**明显震动约 1~2 秒**，属正常现象（校准必须驱动马达）。
 * 校准后把正式工作参数写回去（校准表里的 FEEDBACK/CONTROL3 不是工作值）。 */
bool hapticCalibrate(void) {
  if (!drvReady && !drvPowerOn()) {      // 待机时模块已断电 -> 先上电
    DBGLN("[DRV] 校准跳过：模块断电且上电失败");
    return false;
  }
  DBGLN("[DRV] 开始自动校准（马达会震动约 1~2 秒）");
  drvRunAutoCal();
  if (MOTOR_ACTUATOR == 0) {
    drvWrite(DRV2605_REG_FEEDBACK, 0x97);    // LRA_MODE | BRAKE_2X | LOOP_GAIN_MED | BEMF_GAIN_3
    drvWrite(DRV2605_REG_CONTROL1, 0x80);    // STARTUP_BOOST
    drvWrite(DRV2605_REG_CONTROL2, 0x20);    // SAMPLE_TIME 250us
    drvWrite(DRV2605_REG_CONTROL3, 0x48);    // 闭环
    drvWrite(DRV2605_REG_CONTROL4, 0x20);    // AUTOCAL_TIME 500ms
  } else {
    drvWrite(DRV2605_REG_CONTROL3, 0x48 | 0x20);
    drvWrite(DRV2605_REG_FEEDBACK, 0x26);
    drvWrite(DRV2605_REG_CONTROL2, 0x25);
    drvWrite(DRV2605_REG_CONTROL1, 0x80);
  }
  DBGF("[DRV] 校准完成 A_CAL_COMP=0x%02X A_CAL_BEMF=0x%02X\n",
       drvRead(DRV2605_REG_AUTOCALCOMP), drvRead(DRV2605_REG_AUTOCALEMP));
  return true;
}

void hapticSingle() { hapticEffect(1); }                          // 短促一下
void hapticDouble() { hapticEffect(1); delay(120); hapticEffect(1); }
void hapticLong()   { hapticRTP(80, 400); }

void hapticFire() {
  if (!drvReady && !drvPowerOn()) {          // 待机时模块已断电 -> 先上电
    DBGLN("[HAP] skip (DRV2605 offline)");
    return;
  }
  pwrHapticBusy(true);                       // 播放期间禁止进入深睡（需求 4）
  switch (hapticMode) {
    case 0:  hapticSingle(); break;
    case 1:  hapticDouble(); break;
    case 2:  hapticLong();   break;
    default: hapticSingle(); break;
  }
  pwrHapticBusy(false);                      // 播放结束 -> 30s 空闲计时重新开始
}

/* ---------- PIR 持续震动 ----------
 * 用 RTP（实时播放）模式实现"一直震"：
 *   RTPIN 写入非零强度 + GO=1 后，芯片**持续输出**，不需要反复触发；
 *   停止时把 RTPIN 清零并退出 RTP，马达立即断驱动。
 * 这比用 ROM 波形循环触发更干净（没有波形间隙，强度也不会一跳一跳）。 */
void hapticContinuousStart() {
  if (!drvReady && !drvPowerOn()) return;   // 待机时模块已断电 -> 先上电
  drvWrite(DRV2605_REG_RTPIN, HAPTIC_CONT_STRENGTH);
  drvWrite(DRV2605_REG_MODE, 0x05);         // RTP 模式
  drvWrite(DRV2605_REG_GO, 0x01);           // 开始
}

/* 持续期间周期性重写：长按场景下保持输出不衰减的保险动作。
 * 改版原因：实测"PIR 触发时只震了一下"，怀疑静态 RTP 会衰减/丢锁，
 * 因此这里改成**流式重写**——每 50ms 把整个 RTP 状态重新压一遍
 * （模式 + 强度 + GO），不管芯片是衰减了还是自己退出了模式，都会被拉回来。
 * 代价：每 50ms 三次 I2C 写，100kHz 下约 0.3ms，可忽略。 */
void hapticContinuousHold() {
  if (!drvReady) return;
  drvWrite(DRV2605_REG_MODE, 0x05);        // 重新确保 RTP 模式
  drvWrite(DRV2605_REG_RTPIN, HAPTIC_CONT_STRENGTH);
  drvWrite(DRV2605_REG_GO, 0x01);          // 保持输出
}

/* 立即停止震动：清 RTPIN -> 关 GO -> 回内部触发模式 */
void hapticStop() {
  if (!drvReady) return;
  drvWrite(DRV2605_REG_RTPIN, 0x00);
  drvWrite(DRV2605_REG_GO, 0x00);
  delay(2);
  drvWrite(DRV2605_REG_MODE, 0x00);
}

#if DEBUG_SERIAL
/* ---------- 自测：不依赖 PIR，直接持续驱动 N 毫秒 ----------
 * 目的：把"PIR 触发只震一下"拆成两半判断。
 *   - 如果这个自测能持续震动满 5 秒 → 芯片/驱动没问题，问题在 PIR 给了短脉冲
 *   - 如果这个自测也是震一下就停 → 芯片侧没保持住（再看下面的寄存器变化）
 * 每 200ms 回读 MODE/STATUS/RTPIN/VBAT/LRARESON：
 *   MODE 掉回 0x40     -> 芯片自己回待机了
 *   STATUS 出现保护位  -> 过流/过温保护把输出关了
 *   RTPIN 被清零       -> 写入没保持
 *   一切正常但没输出   -> 谐振没锁住/马达规格问题
 */
void debugHoldTest(uint16_t ms) {
  DBGF("\n[TEST] 持续 RTP %u ms 开始\n", ms);
  hapticContinuousStart();

  uint32_t t0 = millis();
  uint32_t last = 0;
  while (millis() - t0 < ms) {
    hapticContinuousHold();
    if ((millis() - t0) - last >= 200) {
      last = millis() - t0;
      DBGF("[TEST] t=%4lu ms MODE=0x%02X STATUS=0x%02X RTPIN=0x%02X "
           "VBAT=0x%02X RESON=0x%02X\n",
           (unsigned long)last,
           drvRead(DRV2605_REG_MODE), drvRead(DRV2605_REG_STATUS),
           drvRead(DRV2605_REG_RTPIN), drvRead(DRV2605_REG_VBAT),
           drvRead(DRV2605_REG_LRARESON));
    }
    delay(5);
  }
  hapticStop();
  DBGF("[TEST] 结束 MODE=0x%02X STATUS=0x%02X\n",
       drvRead(DRV2605_REG_MODE), drvRead(DRV2605_REG_STATUS));
}

/* 串口调试命令（仅在 DEBUG_SERIAL=1 时可用）：
 *   H = 持续驱动自测 5 秒      P = 打印 PIR 引脚电平
 *   I = 重新初始化 DRV2605     ? = 帮助 */
void debugSerialCmd() {
  while (Serial.available()) {
    char c = (char)toupper(Serial.read());
    if (c == '\n' || c == '\r' || c == ' ') continue;
    switch (c) {
      case 'H': debugHoldTest(5000); break;
      case 'S':
        /* 走 PIR 真实状态机，只把高电平用软件维持 5 秒 */
        pirSimUntil = millis() + 5000;
        DBGF("\n[SIM] 模拟 PIR 高电平 5 秒（走真实 PIR 状态机）\n");
        break;
      case 'T':
        /* 模拟"模块只给一个短脉冲"：高电平仅 300ms。
         * 用于验证 PIR_MIN_HOLD_MS（最短持续时长）是否按预期续震。 */
        pirSimUntil = millis() + 300;
        DBGF("\n[SIM] 模拟 PIR 短脉冲 300ms（验证最短持续 %u ms）\n",
             (unsigned)PIR_MIN_HOLD_MS);
        break;
      case 'P': DBGF("[PIR] 引脚当前=%d (有效电平=%d)\n",
                     (int)digitalRead(PIR_PIN), PIR_ACTIVE_HIGH); break;
      case 'C': {
        /* 直接读 nRF 的 PIN_CNF 寄存器，验证引脚到底是不是下拉输入。
         * PIN_CNF 位域：bit0=DIR(0输入/1输出)，bit[3:2]=PULL(0无/1下拉/3上拉) */
        uint32_t pin = g_ADigitalPinMap[PIR_PIN];
        volatile uint32_t *cnf = (pin < 32) ? &NRF_P0->PIN_CNF[pin]
                                            : &NRF_P1->PIN_CNF[pin - 32];
        uint32_t v = *cnf;
        uint32_t pull = (v >> 2) & 0x3;
        const char* pullStr = (pull == 0) ? "无(悬空)" :
                              (pull == 1) ? "下拉(PULLDOWN)" : "上拉(PULLUP)";
        DBGF("[PIN] D%u -> P%u.%02u  PIN_CNF=0x%08lX  DIR=%s  PULL=%s(%lu)  电平=%d\n",
             (unsigned)PIR_PIN, (pin < 32) ? 0u : 1u,
             (pin < 32) ? pin : (pin - 32), (unsigned long)v,
             (v & 1) ? "输出" : "输入", pullStr, (unsigned long)pull,
             (int)digitalRead(PIR_PIN));
        break;
      }
      case 'I': drvReady = hapticInit();
                DBGF("[DRV] 重初始化 -> %s\n", drvReady ? "ONLINE" : "OFFLINE"); break;
      case '?': DBGF("[CMD] H=紧循环5s  S=模拟PIR高电平5s  T=模拟PIR短脉冲300ms  "
                     "P=读PIR引脚  C=读引脚配置(PIN_CNF)  I=重初始化DRV\n"); break;
      default:  DBGF("[CMD] 未知命令 '%c'，敲 ? 看帮助\n", c); break;
    }
  }
}
#endif

/* ======================= 5. 滤波 ======================= */

/* 中值滤波：拷贝窗口排序取中值，滤掉单点尖刺 */
uint16_t medianFilter(uint16_t v) {
  medBuf[medIdx] = v;
  medIdx = (medIdx + 1) % MEDIAN_WIN;
  if (medCount < MEDIAN_WIN) medCount++;

  uint16_t tmp[MEDIAN_WIN];
  memcpy(tmp, medBuf, sizeof(tmp));
  for (uint8_t i = 1; i < medCount; i++) {          // 窗口很小，插入排序足够
    uint16_t key = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  return tmp[medCount / 2];
}

/* ======================= 5.5 电源状态机 =======================
 * 分场景降功耗，并打通"深睡 → 压力唤醒"这条链路。
 *
 * ★ 三个状态与功耗预期
 *   CONNECTED    BLE 连接中：SoftDevice 在连接间隙自动让 CPU 休眠，
 *                无需手动干预；平均电流由连接间隔/延迟决定。
 *                配合 interval 1000ms + latency 1（见 power_config.h），
 *                单次连接事件仅几百 µs，平均可远低于 1mA。
 *   ADVERTISING  只广播（间隔 1000ms）、不做通知，目标 < 300µA。
 *   DEEP_SLEEP   停止广播，RTC2 每 RTC_PERIOD_MS 唤醒一次快速采样 FSR，
 *                未达阈值继续睡，目标 < 20µA。
 *
 * ★ 为什么深睡不用 System OFF（重要）
 *   nRF52 进入 System OFF 后会切断 RTC 与 SAADC，**无法周期性读 ADC**，
 *   只能被 GPIO 电平跳变唤醒 —— 满足不了"ADC > WAKE_THRESHOLD 才唤醒"。
 *   故采用 System ON + WFE（RAM 保持、RTC 继续计数），
 *   这是唯一能实现本需求的休眠方式；实测可到个位数~十几 µA。
 *   若确实要极限功耗，见 power_config.h 里 PWR_USE_SYSTEM_OFF 的说明。
 *
 * ★ 事件驱动
 *   空闲时调用 sd_app_evt_wait()（不是 delay() 空转），
 *   RTC2 比较中断或 BLE 事件都会把 CPU 唤醒。
 * ========================================================================== */

static volatile bool     s_rtcTick   = false;  // RTC2 比较中断置位
static volatile uint32_t s_tickCount = 0;      // RTC2 中断累计次数（诊断）
static volatile uint32_t s_tickEvt   = 0;      // RTC2 EVENTS_TICK 累计（诊断）
/* 主循环迭代计数：用来判定"CPU 到底有没有睡"。
 * 判据：末尾调用 sd_app_evt_wait() 后，循环次数应为**个位数/秒**；
 *       若是几千~几万次/秒，说明 CPU 根本没睡，静态电流会多出 3~6mA。 */
static volatile uint32_t s_loopCount = 0;
static pwr_state_t   s_pwrState   = PWR_ST_BOOT;
static uint32_t      s_stateSince = 0;       // 进入当前状态的时刻
static uint32_t      s_lastActivity = 0;     // 最后一次"有活动"的时刻（连接/震动/唤醒）
static uint16_t      s_connHandle = BLE_CONN_HANDLE_INVALID;
static bool          s_connReqDone = false;  // 是否已发起连接参数更新
static bool          s_hapticBusy  = false;  // 震动播放中（禁止进入深睡）
static bool          s_drvStandby  = false;
static bool          s_drvPwrFail  = false;   // 门控上电后初始化失败过 -> 不再断电
static uint32_t      s_drvLastTry  = 0;       // 上次尝试上电的时刻（失败退避用）

/* ---------- 采样（RTC）唤醒周期：运行期可配（1s / 3s / 5s / 10s，默认 3s） ----------
 * ★★ 广播间隔**不由档位控制** ★★
 *   广播固定用 ADV_FAST_INTERVAL_MS（1s）连续播、永不停止、永不重配置
 *   —— 保证"任何时候手机都能很快发现并连上"。省电改由下面这个**采样唤醒周期**做：
 *   主循环不再 50Hz 连续采样，而是每 s_wakePeriodMs 才醒一次读 FSR / 更新通知。
 *
 *   ★★ 关键：手机"扫出来快不快"只取决于**广播间隔**（固定 1s），与档位无关。
 *      档位只影响压力采样/通知的密度与响应速度。
 *
 *   档位             效果
 *   3000ms（默认）   约每 3 秒采一次
 *   1000ms           响应最快（短促踩踏也抓得到）
 *   5000 / 10000ms   更省电，但触发更迟钝、短促踩踏可能完全漏掉
 *
 * 切换方式（只在主循环改，不在 BLE 回调里改）：
 *   BLE 写 FEE2：'1' -> 1s  '3' -> 3s  '5' -> 5s  '0' -> 10s
 *   （网页 fsr_dashboard.html 上有 "Wake 1/3/5/10 s" 下拉框，选中即发送） */
static uint16_t      s_wakePeriodMs   = SAMPLE_WAKE_PERIOD_MS;  // **未连接时**的采样（RTC）唤醒周期
static uint16_t      s_wakePeriodWant = 0;                      // 请求的新周期，0 = 无请求
static bool          s_newSample      = false;                  // 刚采到新样本 -> 8.3 段发一次通知

/* ---------- BLE 回调 → 主循环 的"延迟执行"标志 ----------
 * ★★ 血泪教训（务必保留）★★
 *   BLE 回调（onConnect / onCmdWrite）是在协议栈事件上下文里执行的，
 *   **绝对不能在里做阻塞操作**（I2C 初始化要 100ms+、hapticRTP 要 delay(ms)）。
 *   实测症状：链路能连上，但中心设备做**服务发现时找不到特征**，
 *   手机侧表现就是"能扫到、连不上"（电脑端写特征报
 *   "Characteristic ...a9 was not found"）。
 *   所以回调里只置标志，真正的工作挪到主循环里做。 */
static volatile bool s_needDrvOn     = false;  // 连接建立 -> 需要给 DRV 上电
static volatile bool s_needAdvRestart = false; // 断连 -> 需要重开快速广播窗口
static volatile bool s_pendingHaptic = false;  // BLE 命令 -> 需要震动
static volatile bool s_pendingCal    = false;  // BLE 命令 -> 需要自动校准
static volatile bool s_pendingReinit = false;  // BLE 命令 -> 需要重新初始化 DRV

/* PWRLOG 宏已在文件开头定义（见第 1 节后的日志宏），这里不再重复定义 */

static const char* pwrStateName(pwr_state_t st) {
  switch (st) {
    case PWR_ST_CONNECTED:   return "CONNECTED";
    case PWR_ST_ADVERTISING: return "ADVERTISING";
    case PWR_ST_IDLE_ADV:    return "IDLE_ADV";
    case PWR_ST_DEEP_SLEEP:  return "DEEP_SLEEP";
    default:                 return "BOOT";
  }
}

static const char* pwrStateTarget(pwr_state_t st) {
  switch (st) {
    case PWR_ST_CONNECTED:   return PWR_TARGET_CONNECTED;
    case PWR_ST_ADVERTISING: return PWR_TARGET_ADVERTISING;
    case PWR_ST_IDLE_ADV:    return PWR_TARGET_IDLE_ADV;
    case PWR_ST_DEEP_SLEEP:  return PWR_TARGET_DEEP_SLEEP;
    default:                 return "-";
  }
}

/* 唯一的指示灯出口：PWR_LED_HEARTBEAT=0 时所有灯都不亮。
 * ★★★ 极性（按原理图核实）：本板 RGB 灯是**共阳极接 3V3**
 *   （3V3 -> RGB 公共阳极 -> 限流电阻 -> P0.26 红 / P0.30 绿 / P0.06 蓝），
 *   所以 **LOW = 点亮，HIGH = 熄灭** —— 与核心 variant.h 里 `LED_STATE_ON=1`
 *   （"高电平点亮"）的假设**正好相反**！本文件所有写灯的地方都按共阳极来写。
 * ★ 蓝灯的"广播闪烁"由 Bluefruit 库驱动，已在 initBLE() 里用
 *   Bluefruit.autoConnLed(false) 关掉（且要注意调用顺序，见 调试记录.md C9）。 */
static inline void pwrLed(bool on) {
#if PWR_LED_HEARTBEAT
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);   // ★ 共阳极：低电平点亮
#else
  (void)on;
#endif
}

/* 每次状态转换打印一行：[pwr] state=XX, target=XXuA */
static void pwrLogState(const char* why) {
  PWRLOG("[pwr] state=%s, target=%s", pwrStateName(s_pwrState), pwrStateTarget(s_pwrState));
  if (why) PWRLOG("  (%s)", why);
  PWRLOG("\n");
}

void pwrEnterState(pwr_state_t st, const char* why) {
  if (s_pwrState == st) return;
  s_pwrState   = st;
  s_stateSince = millis();
  pwrLogState(why);
}

void pwrActivity(void) { s_lastActivity = millis(); }

/* 震动播放期间禁止进入深睡（需求 4：播放完成后重新计时 30s 空闲超时） */
void pwrHapticBusy(bool busy) {
  s_hapticBusy = busy;
  if (!busy) pwrActivity();
}

/* ---------- RTC2：毫秒级时基（RTC0 归 SoftDevice，RTC1 归 millis） ---------- */
extern "C" void RTC2_IRQHandler(void) {
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    s_rtcTick = true;
  }
}

static void pwrRtcInit(void) {
  NRF_RTC2->TASKS_STOP  = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  /* prescaler=31 -> 32768/(31+1) = 1024Hz，正好 1 tick = 1ms，无累积误差 */
  NRF_RTC2->PRESCALER = 31;
  NRF_RTC2->CC[0]     = PWR_AWAKE_TICK_MS;
  NRF_RTC2->EVTENSET  = RTC_EVTENSET_COMPARE0_Msk;
  NRF_RTC2->INTENSET  = RTC_INTENSET_COMPARE0_Msk;
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_SetPriority(RTC2_IRQn, 6);
  NVIC_EnableIRQ(RTC2_IRQn);
  NRF_RTC2->TASKS_START = 1;
}

static void pwrRtcSetPeriod(uint32_t ms) {
  NRF_RTC2->CC[0]       = (ms & 0xFFFFFFUL);
  NRF_RTC2->TASKS_CLEAR = 1;
  s_rtcTick = false;
}

/* 空闲：睡到下一个中断（BLE 事件、USB、定时器）才醒 —— 事件驱动，不空转。
 *
 * ★★ 这里曾是"待机 5.6mA"的真凶（务必保留结论）★★
 *   原先只调用 sd_app_evt_wait()，但实测在本核心 + SoftDevice S140 组合下
 *   它**每次立即返回**：主循环变成空转，实测 loop=45000 次/秒，
 *   CPU 全速运行 -> 静态电流多 3~6mA。与模块、外设都无关。
 *   改为标准 WFE 序列后 CPU 才真正进入睡眠。
 * 自检：日志 loop=xx/s 应为个位数/秒（USB 插着时约 1000/s，SOF 每 1ms 一次）。 */
static void pwrWaitEvent(void) {
#if PWR_SLEEP_USE_WFE
  /* 先把事件寄存器"喂掉"（SEV 置位 + WFE 立即消费），
   * 再执行一次 WFE，这次才会真正睡到下一个中断。 */
  __SEV();
  __WFE();
  __WFE();
#else
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);
  if (sd_en) {
    (void) sd_app_evt_wait();
  } else {
    __WFE();
    __SEV();
    __WFE();
  }
#endif
}

/* ---------- 唤醒后的外设重初始化 ---------- */
static void reinit_peripherals(void) {
  analogReadResolution(12);          // ADC
  (void) analogRead(FSR_PIN);        // 丢弃首次无效读数
  pirPowerOn();                      // PIR 模块供电（若曾被断掉）
  (void) drvPowerOn();               // DRV2605：上电 + I2C + 软复位 + LRA 配置
  PWRLOG("[pwr] periph reinit (adc=ok drv=%s)\n", drvReady ? "ok" : "offline");
}

/* ---------- 深睡唤醒源：低功耗 GPIO 电平跳变 ----------
 * ★ 为什么不用 RTC2 定时唤醒（重要，实测结论）：
 *   本核心 + SoftDevice S140 组合下，RTC2 的**计数器能跑**（1024Hz 正确），
 *   EVTENSET/INTENSET/NVIC 全部正确使能，但 **EVENTS_TICK 与 EVENTS_COMPARE
 *   都不产生**（实测 tickEvt=0、cmp=0）。nRF52 的 RTC 事件是"产生即置位"的，
 *   因此这不是中断投递问题，而是该环境下 RTC2 事件链路不可用。
 *   所以改用 GPIO PORT 事件（SENSE）作为唤醒源：
 *     · 走的是 PORT 事件机制，**不请求 HFCLK**，属于真正的低功耗唤醒
 *     · 代价：只能按**数字电平**（VIH≈0.7×VDD≈2.3V）触发，
 *       轻按（低于 VIH）不会唤醒；唤醒后再用 ADC 精确核对 WAKE_THRESHOLD
 *   若坚持要"定时唤醒采样"，需要另找可用时基（见交付说明）。 */

static volatile bool s_padWake = false;

static void pwrPadIsr(void) {       // attachInterrupt 要求 void(void)
  s_padWake = true;                 // ISR 里只置标志，判断留给主循环
}

static void pwrWakeSourceEnable(bool en, bool armFsr) {
  static bool armed = false;
  if (en && !armed) {
    /* 用核心自带的 attachInterrupt（内部走 GPIOTE IN 事件）。
     * ⚠ 功耗代价（必须知道）：
     *   GPIOTE 的 IN 事件做边沿检测需要 HFCLK 参与，**武装期间静态电流会明显抬高**，
     *   所以用这种唤醒源时"深睡 < 20µA"是达不到的，实测通常在几百 µA 量级。
     *   要真正做到 < 20µA 只有两条路：
     *     (a) System OFF + 核心自带 systemOff(pin, HIGH) —— 走 SENSE/PORT 事件，
     *         只需 LFCLK，约 1µA；代价：唤醒即复位、无法周期性采样 ADC、
     *         且只能按数字电平（VIH≈2.3V）触发。
     *     (b) 用 LPCOMP 低功耗比较器做阈值唤醒（硬件比较，约 1~2µA，
     *         阈值档位为 VDD/8 的倍数，最接近 WAKE_THRESHOLD=500 ≈ 0.44V）。
     *   两者都建议在下一轮迭代里替换本函数。 */
    if (armFsr) {
      attachInterrupt(FSR_PIN, pwrPadIsr, RISING);
    }
    attachInterrupt(PIR_PIN, pwrPadIsr, RISING);
    armed     = true;
    s_padWake = false;
    PWRLOG("[pwr] wake sources armed (fsr=%s pir=A%u, GPIOTE IN event)\n",
           armFsr ? "armed" : "SKIPPED(fsr high at rest)", (unsigned)PIR_PIN);
  } else if (!en && armed) {
    detachInterrupt(FSR_PIN);
    detachInterrupt(PIR_PIN);
    armed = false;
    PWRLOG("[pwr] wake sources disarmed\n");
  }
}

/* ---------- 进入深睡：停广播 -> 降外设 -> 循环"睡-采样" ---------- */
static void pwrEnterDeepSleep(const char* why) {
  pwrEnterState(PWR_ST_DEEP_SLEEP, why);
  PWRLOG("[pwr] deep sleep: rtc=%u ms, wake_threshold=%u\n",
         (unsigned)RTC_PERIOD_MS, (unsigned)WAKE_THRESHOLD);

  Bluefruit.Advertising.stop();

  /* 深睡期间把 DRV2605 模块整个断电（省掉模块 LDO/电源灯的 mA 级电流）。
   * ★ 但 PIR **必须保持供电**：它是深睡的唯一唤醒源（attachInterrupt 监听
   *   它的电平跳变），断了电就永远醒不来了。 */
  drvPowerOff();
  pirPowerOn();

  digitalWrite(LED_BUILTIN, HIGH);      // 红灯关（★ 共阳极：HIGH = 灭）
  pwrLed(false);                        // 统一出口：PWR_LED_HEARTBEAT=0 时空操作
  pwrRtcSetPeriod(RTC_PERIOD_MS);       // 保留调用；当前环境下 RTC 事件不可用（见上）

  /* ★ 进睡自检：若 FSR 节点"静止时就已经高于阈值"，说明传感器未接 /
   *   分压下臂电阻缺失导致引脚常高。这时**绝不能武装压力唤醒**，
   *   否则会立刻唤醒并陷入"睡-醒"打转（实测踩过：武装后 0ms 就 adc=2940 唤醒）。
   *   此情况下只保留 PIR 唤醒，并打印告警。 */
  uint32_t preAcc = 0;
  for (uint8_t i = 0; i < 4; i++) preAcc += analogRead(FSR_PIN);
  const uint16_t preV   = (uint16_t)(preAcc / 4);
  const bool     fsrSane = (preV <= (uint16_t)WAKE_THRESHOLD);
  if (!fsrSane) {
    PWRLOG("[pwr] WARN: fsr reads %u at rest (> %u) -> pressure wake DISABLED; "
           "check FSR wiring / bottom resistor\n", preV, (unsigned)WAKE_THRESHOLD);
  }
  pwrWakeSourceEnable(true, fsrSane);   // 武装唤醒源（PIR 总是武装）

  uint32_t lastSamp = millis();
  for (;;) {
    pwrWaitEvent();                     // 睡到有事件（电平跳变 / USB / SD 内部事件）

    const uint32_t t     = millis();
    const bool     byPad = s_padWake;   // 是否由电平跳变直接唤醒
    /* 非电平唤醒（如 USB 中断）时按 RTC_PERIOD_MS 节流，避免频繁开 ADC */
    if (!byPad && (t - lastSamp) < (uint32_t)RTC_PERIOD_MS) continue;
    s_padWake = false;
    lastSamp  = t;

    /* 去抖确认：3 次采样里至少 2 次超阈值才算真实按压（避免毛刺唤醒） */
    uint16_t v       = 0;
    uint8_t  hits    = 0;
    const bool pirNow = (digitalRead(PIR_PIN) == (PIR_ACTIVE_HIGH ? HIGH : LOW));
    if (fsrSane) {
      for (uint8_t i = 0; i < 3; i++) {
        v = (uint16_t)analogRead(FSR_PIN);
        if (v > (uint16_t)WAKE_THRESHOLD) hits++;
        delay(8);
      }
    }

    if (hits >= 2 || pirNow) {
      pwrWakeSourceEnable(false, false); // 先撤掉唤醒源，避免唤醒期间误触发
      if (pirNow) {
        PWRLOG("[pwr] wake by pir  (pin high)\n");
      } else {
        PWRLOG("[pwr] wake by pressure  (adc=%u > %u)\n", v, (unsigned)WAKE_THRESHOLD);
      }
      reinit_peripherals();
      pwrRtcSetPeriod(PWR_AWAKE_TICK_MS);
      Bluefruit.Advertising.start(0);
      pwrActivity();
      pwrEnterState(PWR_ST_ADVERTISING, pirNow ? "wake by pir" : "wake by pressure");
      return;
    }
  }
}

/* ---------- 连接参数更新（连接建立后延迟 CONN_UPDATE_DELAY_MS 发起） ---------- */
static void pwrRequestConnParams(void) {
  BLEConnection* conn = Bluefruit.Connection(s_connHandle);
  if (!conn) {
    PWRLOG("[pwr] conn param request skipped (no connection)\n");
    return;
  }
  /* Bluefruit 的单位（见 BLEConnection.cpp -> ble_gap_conn_params_t）：
   *   conn_interval 用 1.25ms，sup_timeout 用 10ms */
  const uint16_t interval = (uint16_t)(CONN_INTERVAL_MS / 1.25f + 0.5f);
  const uint16_t sup_to   = (uint16_t)(CONN_SUP_TIMEOUT_MS / 10);
  const bool ok = conn->requestConnectionParameter(interval, CONN_SLAVE_LATENCY, sup_to);
  PWRLOG("[pwr] conn param update: interval=%ums latency=%u sup=%ums -> %s\n",
         (unsigned)CONN_INTERVAL_MS, (unsigned)CONN_SLAVE_LATENCY,
         (unsigned)CONN_SUP_TIMEOUT_MS, ok ? "sent" : "FAILED");
}

/* ---------- 模块供电脚：必须用 High-Drive ----------
 * ★ 这是"唤醒后马达不震"的直接原因，务必保留：
 *   nRF52840 的 GPIO 默认是 **Standard-Drive（约 2mA）**，
 *   而 DRV2605 模块（电源指示灯 + LDO 静态电流）就要 ~3mA，
 *   再加上电瞬间还要给模块上的储能电容充电（10µF 用 2mA 充到 3.3V 要十几 ms）。
 *   结果：引脚电压塌陷 -> 模块内部 LDO 起不来 / I2C 电平被判错
 *   -> 初始化失败 -> 马达不震（而且日志会显示 drv=offline）。
 *   改成 High-Drive（H0H1，约 10mA）后即可正常。
 * 说明：这里直接用寄存器写 PIN_CNF，和核心 Wire_nRF52.cpp 里对 I2C 引脚
 *   的做法一致，避免依赖 nrf_gpio.h 是否被包含。 */
static inline void pwrPinHiDrive(uint32_t arduinoPin) {
  const uint32_t pin  = digitalPinToPinName(arduinoPin);
  NRF_GPIO_Type *port = (pin < 32) ? NRF_P0 : NRF_P1;
  port->PIN_CNF[pin & 31] =
      ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)   |
      ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)  |
      ((uint32_t)GPIO_PIN_CNF_DRIVE_H0H1       << GPIO_PIN_CNF_DRIVE_Pos) |
      ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
}

/* 打开模块供电：先把输出寄存器置高再切输出，避免切换瞬间产生低电平毛刺 */
static inline void pwrPinSupplyOn(uint32_t arduinoPin) {
  digitalWrite(arduinoPin, HIGH);
  pwrPinHiDrive(arduinoPin);
}

/* ---------- I2C 总线自检 / 恢复（防止固件卡死） ----------
 * ★★ 这条非常关键（实测踩过）★★
 *   nRF 的 TWIM 在一个"被从机拉死"的总线上（SDA 一直为低，典型原因：
 *   从机没供电但通过 I2C 线被半供电、或从机复位中）会**无限等待**，
 *   而 Arduino 核心的 Wire 实现里那些 while 循环**没有超时**。
 *   症状：整个固件卡死 —— 串口一行日志都没有、BLE 也不广播，
 *   看起来像"板子坏了"，其实只是卡在总线等待里。
 *   所以：进 I2C 之前先看总线是否空闲；脏了就做一次标准恢复；
 *   恢复不了就直接放弃 I2C（功能降级），绝不去调用会死等的 Wire。 */
static bool i2cBusIdle(void) {
  pinMode(SDA, INPUT_PULLUP);          // 内部上拉：悬空读高，被拉低才是真被占
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(50);
  return (digitalRead(SDA) == HIGH) && (digitalRead(SCL) == HIGH);
}

/* 用**软件位翻转**做一次最简探测：START + 地址字节 + 读 ACK + STOP。
 * ★ 为什么不用 Wire：核心库的 Wire 在脏总线上会无限等待（无超时）→ 整个固件卡死。
 *   这里时序完全由我们控制，任何情况下都会返回，所以可以放心当作"安全探测"。 */
static bool i2cProbeBitbang(uint8_t addr7) {
  /* ★ 起始前必须先"输入+上拉"去读 SDA：
   *   nRF 的输出模式会把输入缓冲断开（INPUT_Disconnect），此时 digitalRead 恒为 0，
   *   于是这个判断永远成立、探测永远失败 —— 这个坑踩过（表现为"总线上没有任何设备应答"，
   *   而实际上芯片好好的）。所以先用 INPUT_PULLUP 读，再切成输出。 */
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  if (digitalRead(SDA) == LOW) return false;      // SDA 真被拉住 -> 总线不可用

  pinMode(SDA, OUTPUT); pinMode(SCL, OUTPUT);
  digitalWrite(SDA, HIGH); digitalWrite(SCL, HIGH);
  delayMicroseconds(5);

  digitalWrite(SDA, LOW);  delayMicroseconds(5);  // START
  digitalWrite(SCL, LOW);  delayMicroseconds(5);

  const uint8_t byte = (uint8_t)(addr7 << 1);     // 地址 + 写位
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(SDA, ((byte >> i) & 1) ? HIGH : LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
  }

  pinMode(SDA, INPUT_PULLUP);                     // 释放 SDA 读 ACK
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  const bool ack = (digitalRead(SDA) == LOW);     // 从机拉低 = ACK
  digitalWrite(SCL, LOW);  delayMicroseconds(5);

  pinMode(SDA, OUTPUT);                           // STOP
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);                     // 交回给 Wire 之前保持高
  pinMode(SCL, INPUT_PULLUP);
  return ack;
}

static void i2cBusRecover(void) {
  pinMode(SDA, INPUT_PULLUP);          // 先放开 SDA
  pinMode(SCL, OUTPUT);
  for (uint8_t i = 0; i < 9; i++) {    // 9 个时钟：足够让从机把 SDA 放开
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);                // 补一个 STOP：SCL 高时 SDA 由低->高
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

/* ---------- I2C 总线自检：把"总线上到底有没有设备应答"打出来 ----------
 * 用途：区分"模块没供电/线断了"（整条总线无人应答）与"芯片在但地址不对"。
 * 这是现场最有用的一条证据：只要打印出应答地址，就说明 I2C 通着。 */
static void i2cScanLog(void) {
  char buf[96] = {0};
  int  n = 0;
  for (uint8_t a = 0x08; a < 0x78 && n < 80; a++) {
    if (i2cProbeBitbang(a)) n += snprintf(buf + n, sizeof(buf) - n, " 0x%02X", a);
  }
  if (buf[0]) PWRLOG("[DRV] I2C 扫描应答:%s\n", buf);
  else        PWRLOG("[DRV] I2C 扫描：总线上没有任何设备应答"
                     "（模块没供电 / SDA-SCL 断线 / 接线错）\n");
}

/* ---------- DRV2605 模块：初始化 / 待机 ----------
 * 当前接法：模块 VCC **直连 3.3V**，且 **D3 已关闭**（DRV_PWR_GATE_ENABLE=0），
 *   所以这里**不驱动任何供电脚**，只做：确认 I2C 总线干净 -> Wire.begin()
 *   -> 初始化芯片（能不能震就看日志 drv=on/off：on = 探到 0x5A）。
 *   （只有旧的 DRV_PWR_GATE_ENABLE=1 接法才会去拉 D3 供电并等 DRV_PWR_SETTLE_MS。）
 * 幂等：已初始化（drvReady=true）时直接返回，触觉函数可以放心随用随调。 */
bool drvPowerOn(void) {
  if (drvReady) return true;
  /* 失败退避：芯片不在线时不要每 50ms 就重试（会刷屏并反复占 I2C）。
   * 2 秒内只尝试一次；成功或失败都会更新时刻。 */
  const uint32_t tNow = millis();
  if (s_drvLastTry && (tNow - s_drvLastTry) < 2000) return false;
  s_drvLastTry = tNow;
#if DRV_PWR_GATE_ENABLE
  pwrPinSupplyOn(PIN_DRV_VCC);           // ★ High-Drive + 先置高再切输出（D3 常高）
  delay(DRV_PWR_SETTLE_MS);              // 等模块内部 LDO/上拉/储能电容稳定
#endif
  /* ★ 进 I2C 之前先确认总线是干净的，否则宁可降级也不要卡死（见上方注释） */
  if (!i2cBusIdle()) {
    PWRLOG("[pwr] !! I2C 总线不空闲（SDA/SCL 未都为高）-> 尝试总线恢复\n");
    i2cBusRecover();
    if (!i2cBusIdle()) {
      PWRLOG("[pwr] !! 总线恢复失败：请检查模块供电与 SDA/SCL/GND 接线。"
             "本次跳过 DRV2605 初始化（避免固件卡死在总线等待）\n");
      return false;
    }
    PWRLOG("[pwr] 总线恢复成功\n");
  }
  Wire.begin();                          // 核心的 begin() 会重新配置 SDA/SCL 引脚
  Wire.setClock(100000);
  /* 上电后第一次探测可能因模块尚未完全稳而失败 -> 重试几次。
   * 待机时模块是被"完全断电"的，重新上电需要时间，这一步很关键。 */
  for (uint8_t i = 0; i < DRV_PWR_RETRY && !drvReady; i++) {
    if (i) delay(DRV_PWR_SETTLE_MS);
    drvReady = hapticInit();             // 含 DEV_RESET + LRA 参数（默认不做自动校准）
  }
  s_drvStandby = false;
  if (!drvReady) {
    /* 失败就记下来：之后**不再给模块断电**，优先保证功能可用（防止反复断电-上电） */
    s_drvPwrFail = true;
    PWRLOG("[pwr] !! DRV2605 上电后初始化失败（模块欠压/驱动能力/接线）"
           " -> 以后不再断电，避免马达不震\n");
    i2cScanLog();                        // 打出总线上有谁应答，用于区分"没供电"和"地址不对"
  } else {
    PWRLOG("[pwr] DRV2605 上电 -> ok\n");
  }
  return drvReady;
}

void drvPowerOff(void) {
  /* 兜底：如果之前"门控上电后初始化失败"过，就**不再断电**。
   * 宁可多耗几 mA，也不能让马达用不了（实测踩过：唤醒后马达不震）。 */
  if (s_drvPwrFail) return;
#if DRV_PWR_KEEP_ON
  /* ★ 模块由 3.3V 常供电、D3 已关闭（固件不驱动任何供电脚），所以这里只把芯片
   *   写回 STANDBY（芯片自身 ~1µA），并保持 drvReady=true —— 唤醒时不需要重新
   *   初始化，PIR/FSR 触发即可震动，也不会有"断电-上电"造成的初始化失败。
   *   代价：省不到模块上 LDO/电源灯的 mA 级静态电流（要省只能改硬件）。 */
  if (drvReady) {
    drvWrite(DRV2605_REG_RTPIN, 0x00);
    drvWrite(DRV2605_REG_GO, 0x00);
    drvWrite(DRV2605_REG_MODE, 0x40);    // STANDBY
  }
  s_drvStandby = true;
  PWRLOG("[pwr] DRV2605 -> STANDBY（模块由 3.3V 常供电，D3 已关闭）\n");
  return;
#endif
  if (drvReady) {                        // 先确保 OUT± 停驱动，再断电
    drvWrite(DRV2605_REG_RTPIN, 0x00);
    drvWrite(DRV2605_REG_GO, 0x00);
    drvWrite(DRV2605_REG_MODE, 0x40);    // STANDBY
  }
  drvReady     = false;
  s_drvStandby = true;
  Wire.end();                            // 释放 I2C（避免上拉经 ESD 二极管倒灌）
  pinMode(SDA, INPUT);                   // D4 浮空
  pinMode(SCL, INPUT);                   // D5 浮空
#if DRV_PWR_GATE_ENABLE
  digitalWrite(PIN_DRV_VCC, LOW);        // ★ 断掉模块 VCC：电源灯/LDO 一起停
  PWRLOG("[pwr] DRV2605 断电（模块不再耗电）\n");
#else
  PWRLOG("[pwr] DRV2605 -> STANDBY（未启用 VCC 门控）\n");
#endif
}

/* ---------- PIR 模块：上电 / 断电 ---------- */
void pirPowerOn(void) {
#if PIR_PWR_GATE_ENABLE
  const bool wasOff = (digitalRead(PIN_PIR_VCC) != HIGH);   // 断电平判断要在拉高之前
  pwrPinSupplyOn(PIN_PIR_VCC);           // 同样用 High-Drive，避免模块欠压
  if (wasOff) {
    /* 重新上电后模块又要走一遍预热，可能先输出一段高电平：
     * 复位预热标志，避免被当成真实触发（否则会"莫名震一下"）。 */
    pirSeenIdle   = false;
    pirWarmupWarn = false;
  }
#endif
}

void pirPowerOff(void) {
#if PIR_PWR_GATE_ENABLE
  digitalWrite(PIN_PIR_VCC, LOW);
#endif
}

/* ---------- 运行期切换采样（RTC）唤醒周期（1s / 3s / 5s / 10s，默认 1s） ----------
 * ★ 只改一个变量，**完全不碰广播**：广播固定 1s 连续播，所以切档永远
 *   不会造成"广播空窗 / 手机连不上"（这正是旧版把档位做在广播间隔上的坑）。
 * ★ 必须在**主循环**里调用（BLE 回调里只置标志，不做任何实际动作）。 */
static void wakeApplyPeriod(uint16_t ms) {
  if (ms < 100)   ms = 100;        // 下限 100ms（再快没意义，采样不需要）
  if (ms > 60000) ms = 60000;      // 上限 60s
  s_wakePeriodMs = ms;
  PWRLOG("[pwr] 采样唤醒周期 -> %ums（广播仍固定 %ums 连续播，不影响可连接性）\n",
         (unsigned)ms, (unsigned)ADV_FAST_INTERVAL_MS);
}

/* ---------- 确保"正在广播、且处于易连状态" ----------
 * 开机 / 刚断连 / 有人交互（PIR、压力）时调用。
 *
 * ★★ 这里**不做 stop()+start()**（这是"蓝牙连接有时候会失败"的主因）★★
 *   广播一停，这一瞬间到达的连接请求就落空；而反复重启还会让中心设备
 *   在"设备列表里看得见、点连接却失败"之间来回跳。
 *   现在广播间隔**固定为 ADV_FAST_INTERVAL_MS（1s）且永不重配置** ——
 *   档位（'1'/'3'/'5'/'0'）只改采样唤醒周期，压根不碰广播（见 wakeApplyPeriod()）。
 *   交互时该做的只是"保证 PIR 有电 + 刷新活动时刻 + 更新状态"。 */
static void pwrBoostAdv(const char* why) {
  pirPowerOn();                    // 回到"可交互窗口"时确保 PIR 已供电
  if (!Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.start(0);
    PWRLOG("[pwr] adv 重新启动（之前已停止）(%s)\n", why ? why : "");
  }
  PWRLOG("[pwr] adv 保持固定 %ums 间隔、连续广播 (%s)\n",
         (unsigned)ADV_FAST_INTERVAL_MS, why ? why : "");
  pwrEnterState(PWR_ST_ADVERTISING, why);
}

/* ---------- 主循环每轮调用 ---------- */
static void pwrTick(void) {
  const uint32_t now = millis();
  if (s_lastActivity == 0) s_lastActivity = now;

  /* 诊断/兜底：轮询 RTC2 COMPARE0 事件。
   * 若这里能数到 ticks 增长，说明硬件事件是有的，问题出在中断投递；
   * 若这里也数不到，说明 RTC2 的比较事件根本没产生。 */
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    s_rtcTick = true;
    s_tickCount++;
  }

  /* ---------- 周期性状态自检（所有状态都打，每 5 秒） ----------
   * 用途：① 现场一眼看清当前状态与 DRV2605 是否在线（drv=on/off）
   *       ② 看 CPU 有没有真的睡（loop=xx/s：个位数正常，~1000/s 是 USB SOF，
   *          几万/s 说明没睡、会多耗 3~6mA）
   * 注：原先只在 ADVERTISING 状态打，导致"连上/待机"时看不到任何日志，
   *     排查"唤醒后不震"时无法判断芯片到底有没有重新上电，故改为全状态输出。 */
  static uint32_t lastDiag    = 0;
  static uint32_t lastLoopCnt = 0;
  if ((now - lastDiag) >= 5000) {
    const uint32_t dt        = now - lastDiag;
    const uint32_t loops     = s_loopCount - lastLoopCnt;
    const uint32_t loopsPerS = (dt > 0) ? (uint32_t)((uint64_t)loops * 1000ULL / dt) : 0;
    lastDiag    = now;
    lastLoopCnt = s_loopCount;
    /* ★ d2=PIR 模块供电脚电平；D3 已关闭、不再打印（浮空脚读它没有意义），
     *   drvgate=%d 就是 DRV_PWR_GATE_ENABLE（0 = 固件不驱动 D3）。
     *   马达能不能震只看 **drv=on/off**：on = I2C 探到 0x5A、芯片在线。
     *   wake=%ums 是**未连接时**用的档位周期（1000/3000/5000/10000）；
     *   samp=%ums 是**当前实际采样周期** —— 已连接恒为 SAMPLE_CONNECTED_MS(1000)，
     *             未连接则等于 wake 档位。**判断"连上后有没有变化"看这一项**。
     *   adv=%ums 是**广播间隔**，恒为 ADV_FAST_INTERVAL_MS（1000ms）—— 它决定
     *   "手机能不能很快发现/连上"，**不受档位影响**。
     *   led=R/G/B 是三个指示灯的**亮灭状态**（已按共阳极取反，**1 = 亮**）：
     *   关灯版应恒为 `0/0/0`；出现任何一个是 1 就是 mA 级耗电。 */
    PWRLOG("[pwr] state=%s drv=%s pir=%d hap=%d loop=%lu/s adv=%ums wake=%ums samp=%ums "
           "d2=%d drvgate=%d led=%d/%d/%d\n",
           pwrStateName(s_pwrState), drvReady ? "on" : "off",
           (int)pirActive, (int)s_hapticBusy, (unsigned long)loopsPerS,
           (unsigned)ADV_FAST_INTERVAL_MS, (unsigned)s_wakePeriodMs,
           (unsigned)(deviceConnected ? SAMPLE_CONNECTED_MS : s_wakePeriodMs),
           (int)digitalRead(PIN_PIR_VCC), (int)DRV_PWR_GATE_ENABLE,
           (int)!digitalRead(LED_RED), (int)!digitalRead(LED_GREEN), (int)!digitalRead(LED_BLUE));
    if (loopsPerS > 5000) {
      PWRLOG("[pwr] !! 主循环 %lu 次/秒 -> CPU 根本没睡，静态电流会多 3~6mA"
             "（检查 PWR_SLEEP_USE_WFE / 是否有中断持续产生）\n",
             (unsigned long)loopsPerS);
    }
  }

  /* ---------- 延迟执行队列 ----------
   * BLE 回调（连接/断连/写命令）里只置标志，真正的工作都在这里做：
   * 回调里做阻塞操作会卡住协议栈，导致中心设备服务发现失败（手机"连不上"）。 */
  if (s_needDrvOn)      { s_needDrvOn      = false; drvPowerOn(); }
  if (s_needAdvRestart) { s_needAdvRestart = false; pwrBoostAdv("disconnected -> fast window"); }
  if (s_pendingHaptic)  { s_pendingHaptic  = false; hapticFire(); }
  if (s_pendingCal)     { s_pendingCal     = false; (void)hapticCalibrate(); }
  if (s_pendingReinit)  { s_pendingReinit  = false; drvReady = hapticInit(); }
  if (s_wakePeriodWant) {                         // BLE/串口请求切换采样唤醒周期（1s/3s/5s/10s）
    const uint16_t ms = s_wakePeriodWant;
    s_wakePeriodWant = 0;
    wakeApplyPeriod(ms);                          // 只改变量，不碰广播（不影响连接）
  }

  if (s_pwrState == PWR_ST_BOOT) {
    pwrEnterState(PWR_ST_ADVERTISING, "boot");
  }

  if (s_pwrState == PWR_ST_CONNECTED) {
    if (!s_connReqDone && (now - s_stateSince) >= CONN_UPDATE_DELAY_MS) {
      s_connReqDone = true;
      pwrRequestConnParams();
    }
    return;                                  // 连接中不做深睡
  }

  /* 有交互（PIR 触发 / 正在震动）时：刷新活动时刻；若已降到慢广播，
   * 立刻拉回快速窗口，保证用户操作期间手机能立即连上。 */
  if (s_hapticBusy || pirActive) {
    pwrActivity();
    drvPowerOn();                            // 有交互 -> 若待机时已断电，先上电（幂等）
    if (s_pwrState == PWR_ST_IDLE_ADV) pwrBoostAdv("interaction -> fast window");
    return;
  }

  if (s_pwrState == PWR_ST_ADVERTISING) {
    /* 快速窗口倒计时（状态与功耗自检见上面的周期性日志） */
    static uint32_t lastIdleLog = 0;
    if ((now - lastIdleLog) >= 5000) {
      lastIdleLog = now;
      PWRLOG("[pwr] adv fast: idle=%lus/%us（窗口结束后转慢广播、DRV 断电）\n",
             (unsigned long)((now - s_lastActivity) / 1000),
             (unsigned)ADV_FAST_WINDOW_S);
    }
    /* 快速窗口结束：协议栈自己已经切到慢广播（ADV_SET_TERMINATED 超时后
     * 会用 slow_interval 重新启动广播），这里只更新状态与日志，不碰 radio，
     * 因此不产生额外电流。慢广播期间**仍然可以被连接**。 */
    if ((now - s_lastActivity) >= (uint32_t)ADV_FAST_WINDOW_S * 1000UL) {
      /* 无人交互 -> 把 DRV2605 模块**整个断电**。
       * 只写 STANDBY 只能省芯片自身 ~1µA，而模块上的 LDO/电源指示灯是 mA 级，
       * 只能靠断 VCC 解决（这就是 D3 门控的意义）。
       * 有震动需求时触觉函数会自己调 drvPowerOn() 重新上电。 */
      drvPowerOff();
#if PIR_PWR_OFF_IN_IDLE
      pirPowerOff();           // 可选：连 PIR 一起断（待机期间就没有人体触发了）
#endif
      PWRLOG("[pwr] 转待机广播：间隔 %u ms（每 %u 秒唤醒一次）\n",
             (unsigned)ADV_SLOW_INTERVAL_MS, (unsigned)(ADV_SLOW_INTERVAL_MS / 1000));
      pwrEnterState(PWR_ST_IDLE_ADV,
                    "fast window over -> slow adv (可连), D3 保持高电平");
    }
    return;
  }

  if (s_pwrState == PWR_ST_IDLE_ADV) {
#if DEEP_SLEEP_AFTER_S > 0
    /* 仅在显式打开时才停广播深睡：此时手机会完全搜不到设备。 */
    if ((now - s_lastActivity) >= (uint32_t)DEEP_SLEEP_AFTER_S * 1000UL) {
      pwrEnterDeepSleep("idle -> stop advertising (phone cannot discover)");
    }
#endif
    return;
  }
}

void pwrInit(void) {
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);   // DC/DC 必须开，否则休眠电流成倍
  pwrRtcInit();
  pwrRtcSetPeriod(PWR_AWAKE_TICK_MS);
  s_lastActivity = millis();
  PWRLOG("[pwr] init: dc/dc=on, adv fast=%ums/%us -> slow=%ums, deep_sleep_after=%us\n",
         (unsigned)ADV_FAST_INTERVAL_MS, (unsigned)ADV_FAST_WINDOW_S,
         (unsigned)ADV_SLOW_INTERVAL_MS, (unsigned)DEEP_SLEEP_AFTER_S);
  PWRLOG("[pwr] init: wake_threshold=%u, led_indicator=%d\n",
         (unsigned)WAKE_THRESHOLD, (int)PWR_LED_HEARTBEAT);
}

/* ======================= 6. BLE 回调 ======================= */

void setStatus(const char* msg) {
  DBGF("[STATUS] %s\n", msg);
  if (deviceConnected) {
    /* notify(uint8_t*, len) 是 Bluefruit 的正确形式；
     * 用 setMaxLen 预留的缓冲区发变长文本，无需先 write() 再 notify() */
    statusChar.notify((uint8_t*)msg, strlen(msg));
  }
}

/* ★ 注意：Bluefruit 的 setWriteCallback 要的是**函数指针**
 *   void (*)(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
 *   不是 ESP32 那种带虚函数的回调类，所以这里必须写成自由函数。 */

/* FEE2(末位a9)：'S'单震 'D'双震 'L'长震 'C'标定阈值 'P'清峰值 'I'重初始化DRV
 *                'A' 现场自动校准（会震 1~2 秒，开机默认已不做）
 *                '1' / '3' / '5' / '0' 采样（RTC）唤醒周期切 1000 / 3000 / 5000 / 10000 ms
 *                                       （默认 3000；只影响采样/上报频率，不影响可连接性，
 *                                        广播间隔恒为 ADV_FAST_INTERVAL_MS=1000ms） */
void onCmdWrite(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 0) return;
  char c = toupper(data[0]);
  DBGF("[BLE] cmd '%c'\n", c);
  char buf[48];

  /* ★ 这里只置标志、不做阻塞动作（见 s_pendingHaptic 的注释）：
   *   直接 hapticSingle()/hapticLong() 会在协议栈回调里 delay 几百毫秒，
   *   导致中心设备服务发现失败、手机报"连不上"。 */
  switch (c) {
    case 'S': hapticMode = 0; s_pendingHaptic = true; setStatus("mode: single"); break;
    case 'D': hapticMode = 1; s_pendingHaptic = true; setStatus("mode: double"); break;
    case 'L': hapticMode = 2; s_pendingHaptic = true; setStatus("mode: long");   break;
    case 'C':                                   // 用当前滤波值标定阈值
      if (filtVal > 200) {
        thOn  = (uint16_t)(filtVal * 1.15f);
        thOff = (uint16_t)(filtVal * 0.80f);
        snprintf(buf, sizeof(buf), "calibrated th=%u/%u", thOn, thOff);
        setStatus(buf);
      } else {
        setStatus("calibrate failed: value too low");
      }
      break;
    case 'P': peakVal = 0; setStatus("peak cleared"); break;
    /* 采样（RTC）唤醒周期四档（**运行期可配**，默认 3s）：1s / 3s / 5s / 10s
     *   '1' -> 1000ms   '3' -> 3000ms   '5' -> 5000ms   '0' -> 10000ms
     * 网页 fsr_dashboard.html 的 "Wake ? s" 下拉框就是发这四个字符。
     * ★ 关键：档位**只影响采样/上报频率，不影响蓝牙连接**（广播固定 1s 连续播）。
     * ★ 这里只记请求值，真正生效由主循环的延迟队列执行（回调里不做实际动作）。 */
    case '1': s_wakePeriodWant = 1000;  setStatus("wake 1000ms queued");  break;
    case '3': s_wakePeriodWant = 3000;  setStatus("wake 3000ms queued");  break;
    case '5': s_wakePeriodWant = 5000;  setStatus("wake 5000ms queued");  break;
    case '0': s_wakePeriodWant = 10000; setStatus("wake 10000ms queued"); break;
    case 'A':                                   // 现场自动校准（会震动约 1~2 秒）
      s_pendingCal = true;                      // 同样挪到主循环执行（要 1~2 秒）
      setStatus("auto-cal queued");
      break;
    case 'I':                                   // 热插拔 DRV2605 后在线重初始化
      s_pendingReinit = true;                   // 挪到主循环（I2C 初始化会阻塞）
      setStatus("re-init queued");
      break;
    default: setStatus("unknown command"); break;
  }
}

/* FEEA(末位aa)：写 4 字节小端 [thOn, thOff]；或写 2 字节单阈值自动算回差 */
void onThreshWrite(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len >= 4) {
    thOn  = (uint16_t)(data[0] | (data[1] << 8));
    thOff = (uint16_t)(data[2] | (data[3] << 8));
  } else if (len >= 2) {
    thOn  = (uint16_t)(data[0] | (data[1] << 8));
    thOff = (uint16_t)(thOn * 0.75f);
  } else {
    return;
  }
  if (thOff >= thOn) thOff = (uint16_t)(thOn * 0.75f);  // 回差必须为正
  char buf[48];
  snprintf(buf, sizeof(buf), "th=%u/%u", thOn, thOff);
  setStatus(buf);
}

/* 连接状态回调：Bluefruit 的签名是 void(uint16_t conn_hdl) */
void onConnect(uint16_t conn_hdl) {
  deviceConnected = true;
  s_connHandle    = conn_hdl;               // 电源状态机：3 秒后据此发参数更新
  s_connReqDone   = false;
  s_needDrvOn     = true;                   // ★ 只置标志：给 DRV 上电放到主循环做
  pwrLed(true);                             // PWR_LED_HEARTBEAT=1 时才是"常亮=已连接"
  setStatus("connected");
  pwrActivity();
  pwrEnterState(PWR_ST_CONNECTED, "ble connected");
}

void onDisconnect(uint16_t conn_hdl, uint8_t reason) {
  deviceConnected = false;
  /* ★ 断连原因必须**始终可见**（用 PWRLOG 而不是 DBGF）：排查"连接有时失败"
   *   时这一个字节就能区分是中心设备主动断开还是链路出问题：
   *   0x08 = 链路监督超时（连接间隔/延迟设置不被接受或信号差）
   *   0x13 = 远端主动断开（App 关了、手机走了）
   *   0x16 = 本地主动断开
   *   0x3E = 连接建立失败。 */
  PWRLOG("[pwr] BLE 断开 reason=0x%02X\n", (unsigned)reason);
  pwrLed(false);
  /* 广播由 restartOnDisconnect(true) 自动重启，且会**重新进入 fast 阶段**
   * （库内部走 start() -> _start(fast_interval, fast_timeout)），
   * 所以不需要手动 boost，只更新电源状态即可。 */
  s_needAdvRestart = true;                  // ★ 只置标志：重开广播放到主循环做
  pwrActivity();
  pwrEnterState(PWR_ST_ADVERTISING, "disconnected -> fast window");
}

/* ======================= 7. setup ======================= */

void initBLE() {
  Bluefruit.begin();

  /* ★★ 关灯版：必须在 Advertising.start() **之前**关掉库自带的"广播闪烁" ★★
   * 库的实现（bluefruit.cpp）：start() -> _startConnLed() ->
   *   if (_led_conn) xTimerStart(闪烁定时器)，而 _led_conn 默认是 true；
   * 更糟的是闪烁回调里是**裸的 digitalToggle(LED_BLUE)**（不检查标志）——
   * 定时器只要被启动过就会一直闪到连接建立（_stopConnLed()）为止。
   * 症状：未连接时蓝灯一直闪（mA 级）→"连上反而比不连省电"。详见 调试记录.md C9。 */
  Bluefruit.autoConnLed(PWR_LED_HEARTBEAT ? true : false);
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(BLE_DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  /* 首选连接参数（GAP PPCP）：中心设备参考它建立连接。
   * 真正兜底的是连接后 CONN_UPDATE_DELAY_MS 由电源状态机发起的更新请求。 */
  /* ★ 初值必须是快间隔（30ms），否则手机会判"连接失败"（原因见 power_config.h）。
   * 30 秒内由电源状态机发起参数更新，把它降到 CONN_INTERVAL_MS=1000ms 省电。 */
  Bluefruit.Periph.setConnIntervalMS(CONN_INTERVAL_FAST_MS, CONN_INTERVAL_MS);
  Bluefruit.Periph.setConnSlaveLatency(CONN_SLAVE_LATENCY);
  Bluefruit.Periph.setConnSupervisionTimeoutMS(CONN_SUP_TIMEOUT_MS);

  fsrService.begin();

  dataChar.setProperties(CHR_PROPS_NOTIFY);
  dataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataChar.setFixedLen(sizeof(FsrPacket));
  dataChar.begin();

  cmdChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  cmdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cmdChar.setMaxLen(20);
  cmdChar.setWriteCallback(onCmdWrite);      // 函数指针，不是对象
  cmdChar.begin();

  threshChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  threshChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  threshChar.setMaxLen(8);
  threshChar.setWriteCallback(onThreshWrite);
  threshChar.begin();

  statusChar.setProperties(CHR_PROPS_NOTIFY);
  statusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statusChar.setMaxLen(48);
  statusChar.begin();

  /* 单值曲线特征：固定 2 字节，小端 uint16 */
  plotFiltChar.setProperties(CHR_PROPS_NOTIFY);
  plotFiltChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  plotFiltChar.setFixedLen(2);
  plotFiltChar.begin();

  plotRawChar.setProperties(CHR_PROPS_NOTIFY);
  plotRawChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  plotRawChar.setFixedLen(2);
  plotRawChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(fsrService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  /* ★ 广播策略：**单一固定间隔、连续广播**（默认 ADV_USE_SLOW_PHASE=0）
   *   间隔固定为 ADV_FAST_INTERVAL_MS（1s），配合下面的 setFastTimeout(0)，
   *   协议栈全程**不重配置**广播 —— 任何时刻发来的连接请求都能立刻建立链路。
   * 另外保留一条**可选**的两级广播（ADV_USE_SLOW_PHASE=1）：fast 阶段保持
   *   ADV_FAST_WINDOW_S 秒，之后由 SoftDevice 自己（ADV_SET_TERMINATED 后）切到
   *   ADV_SLOW_INTERVAL_MS，全程 CPU 在睡。只用于"还想再省几 µA radio 电流"的场合，
   *   代价是慢播期间手机要多等几秒才能发现设备。
   * ⚠ 历史教训：更早的版本用"30 秒后停广播进深睡"来省电，结果是手机**再也搜不到**
   *   设备（实测症状：扫描列表里没有 XIAO-FSR-Haptic）。所以省电只能动"广播间隔"
   *   或"采样周期"，**绝不能停广播**。
   *   setIntervalMS 内部按 0.625ms 换算；setFastTimeout 单位是秒。 */
  /* ★ 广播间隔**固定**为 ADV_FAST_INTERVAL_MS（1s），运行期不可变 —— 目的就是
   *   "任何时候手机都能发现并连上"。省电改由**采样唤醒周期**（s_wakePeriodMs）
   *   负责：它只影响 CPU/ADC/通知频率，**完全不影响可连接性**。 */
#if ADV_USE_SLOW_PHASE
  Bluefruit.Advertising.setIntervalMS(ADV_FAST_INTERVAL_MS, ADV_SLOW_INTERVAL_MS);
#else
  Bluefruit.Advertising.setIntervalMS(ADV_FAST_INTERVAL_MS, ADV_FAST_INTERVAL_MS);
#endif
#if ADV_USE_SLOW_PHASE
  Bluefruit.Advertising.setFastTimeout(ADV_FAST_WINDOW_S);
#else
  /* ★ 单一固定间隔：setFastTimeout(0) 让协议栈**永不**因为"快播超时"去重配置
   *   广播（重配置瞬间的连接请求会被丢掉 → 表现为"蓝牙连接有时候会失败"）。
   *   代价只是几 µA radio 电流，换来"任何时候尝试连接都能连上"。 */
  Bluefruit.Advertising.setFastTimeout(0);
#endif
  Bluefruit.Advertising.start(0);               // 0 = 不设总时限，永久广播

  /* 兜底（幂等）：标志保持关，并把 LED_BLUE 钉在"灭"电平。 */
  Bluefruit.autoConnLed(PWR_LED_HEARTBEAT ? true : false);
  digitalWrite(LED_BLUE, HIGH);                 // ★ 共阳极：HIGH = 灭
}

void setup() {
  /* ★★ 关灯版：把板上三个指示灯全部关掉（★ 一律写 HIGH = 灭）★★
   *   硬件（原理图核实）：RGB **共阳极接 3V3**，三个阴极经限流电阻接
   *     P0.26 红(D11) / P0.30 绿(D13) / P0.06 蓝(D12)
   *   → **HIGH = 熄灭，LOW = 点亮**（核心 variant.h 的 LED_STATE_ON=1 对这块板是错的，
   *     所以核心 initVariant() 里那句 digitalWrite(HIGH) 本来就是"灭"，无需处理）。
   *   ⚠ 曾把这里写成 LOW，等于**把三个灯全点亮**（mA 级），已改正。
   *   判据：日志 `led=R/G/B`（已按共阳极取反，1 = 亮）应恒为 `0/0/0`。 */
  pinMode(LED_RED,   OUTPUT); digitalWrite(LED_RED,   HIGH);
  pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE,  OUTPUT); digitalWrite(LED_BLUE,  HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);              // ★ 共阳极：HIGH = 灭

  /* 模块供电：只给 **PIR** 模块打开 VCC（D2）。
   * ★ D3 已关闭、不再使用（DRV2605 模块 VCC 直连 3.3V）：这里显式把它置成
   *   高阻输入，保证固件任何时刻都不驱动它。 */
#if !DRV_PWR_GATE_ENABLE
  pinMode(PIN_DRV_VCC, INPUT);                   // D3 = 高阻输入（不驱动）
#endif
  pirPowerOn();                                  // D2 -> PIR 模块 VCC
  delay(10);                                     // 等模块输出电平稳定

  /* PIR 输入端。INPUT_PULLDOWN：悬空读 LOW，避免误触发。
   * 模块若是开漏输出，把这里改成 INPUT_PULLUP 并令 PIR_ACTIVE_HIGH=0。 */
#if PIR_ACTIVE_HIGH
  pinMode(PIR_PIN, INPUT_PULLDOWN);
#else
  pinMode(PIR_PIN, INPUT_PULLUP);
#endif
  pirRaw = pirActive = false;

  /* 调试或 [pwr] 日志需要时才初始化串口。
   * ★ 一旦初始化 USB 串口，DEEP_SLEEP 就达不到 < 20µA：
   *   做电流验收时请把 PWR_LOG_ENABLE 设为 0 并拔掉 USB。 */
#if (DEBUG_SERIAL || PWR_LOG_ENABLE) && !PWR_MEASURE_MODE
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);   // 最多等 3s，不卡死

  DBGLN();
  DBGLN("==============================================");
  DBGLN(" FSR -> BLE Notify + DRV2605 Haptic (PIR 持续震动)");
  DBGLN(" XIAO nRF52840 (Seeeduino:nrf52)");
  DBGLN("==============================================");
#endif

  analogReadResolution(12);                     // 显式 12bit
  (void)analogRead(FSR_PIN);                    // 丢弃首次无效读数

  /* DRV2605 初始化（模块 VCC 直连 3.3V，**不涉及 D3**）：
   * 检查 I2C 总线 -> Wire.begin + 芯片初始化。
   * 不做自动校准（会驱动马达 1~2 秒），所以上电时马达是静止的。 */
  (void)drvPowerOn();

  pwrInit();                                    // 电源状态机：DC/DC + RTC2 + 日志

  initBLE();

#if DEBUG_SERIAL
  DBGF("[BLE] advertising as \"%s\"\n", BLE_DEVICE_NAME);
  DBGLN("[BLE] 手机 nRF Connect 连接后：");
  DBGLN("      FEE1(末位a8) 点 Notify 三箭头 -> 9 字节混合包(raw/filt/peak/标志)");
  DBGLN("      末位ac/ad    Notify 单值 uint16 -> 曲线用（值格式选 UINT16）");
  DBGLN("      FEE2(末位a9) 写 'S'/'D'/'L' -> 切单震/双震/长震");
  DBGLN("      FEEA(末位aa) 写 4 字节小端 [thOn,thOff] -> 改阈值");
  DBGF("[CFG] thOn=%u thOff=%u mode=%u drv=%s  PIR=pin%u(%s)\n",
       thOn, thOff, hapticMode, drvReady ? "ONLINE" : "OFFLINE",
       PIR_PIN, PIR_ACTIVE_HIGH ? "高电平触发" : "低电平触发");
#endif
}

/* 调试辅助：把"本次 PIR 高电平持续时长"编码成红灯闪烁次数。
 * 每 500ms 计一次，至少 1 次、最多 20 次。
 * 作用：不看串口也能判断模块行为 —— 闪 1 下 ≈ 短脉冲（<0.5s）；
 *       闪 5 下 ≈ 2.5 秒；闪 10 下 ≈ 5 秒。
 * 这段闪烁发生在震动停止之后，不影响对震感的判断。 */
void reportPirDuration(uint32_t heldMs) {
#if PIR_REPORT_BLINK
  uint8_t blinks = (uint8_t)((heldMs + 499) / 500);
  if (blinks == 0)  blinks = 1;
  if (blinks > 20)  blinks = 20;
  DBGF("[PIR] 本次高电平持续 %lu ms -> 红灯闪 %u 下\n",
       (unsigned long)heldMs, blinks);
  for (uint8_t i = 0; i < blinks; i++) {
    pwrLed(true);  delay(160);
    pwrLed(false); delay(240);
  }
#else
  DBGF("[PIR] 本次高电平持续 %lu ms（红灯编码已关闭）\n", (unsigned long)heldMs);
#endif
}

/* ======================= 8. loop ======================= */

void loop() {
  uint32_t now = millis();
  s_loopCount++;                 // 功耗自检：主循环频率（见 pwrTick 的 loop=xx/s）

#if DEBUG_SERIAL
  debugSerialCmd();          // 调试命令：H=持续驱动自测 P=读PIR引脚 I=重初始化
#endif

  /* ---------- 8.0 PIR 持续震动 ----------
   * 需求：PIR 高电平期间一直震动，变低电平立即停止。
   * 流程：读引脚 -> 折算有效电平 -> 去抖确认 -> 起/停 RTP 持续输出。
   * 放在采样之前，保证"低电平立即停"的响应最快（不受采样周期影响）。 */
  bool pirLevel = (digitalRead(PIR_PIN) == (PIR_ACTIVE_HIGH ? HIGH : LOW));

#if DEBUG_SERIAL
  /* 软件模拟：命令 S 会让"高电平"维持 5 秒，走的正是下面这条真实状态机，
   * 用来把"PIR 逻辑有没有移植错"与"模块给不给高电平"彻底分开。 */
  if (pirSimUntil) {
    if (now < pirSimUntil) {
      pirLevel = true;
    } else {
      pirSimUntil = 0;
      DBGF("[SIM] 模拟高电平结束（%lu ms）\n", now);
    }
  }
#endif

  if (pirLevel != pirRaw) {
    pirRaw      = pirLevel;
    pirChangeAt = now;                     // 记录变化时刻，用于去抖
  }

#if PIR_WARMUP_GUARD
  /* 预热保护：模块上电瞬间常会输出一段有效电平（HC-SR501 可达数十秒、
   * AM312 约 2 秒），直接采信就变成"每次上电马达自己震一下/几秒"。
   * 规则：必须先读到一次无效电平，才允许接受有效电平作为触发。 */
  if (!pirLevel) pirSeenIdle = true;
  if (pirRaw && !pirSeenIdle) {
    pirRaw = false;                        // 撤回这个高电平，不进入触发流程
    if (!pirWarmupWarn) {
      pirWarmupWarn = true;
      DBGLN("[PIR] 上电预热期：模块尚未输出过无效电平，本次高电平被忽略");
    }
  }
#endif

  if (pirRaw != pirActive && (now - pirChangeAt) >= PIR_DEBOUNCE_MS) {
    if (pirRaw) {
      /* 上升沿：开始持续震动 */
      pirActive      = true;
      pirPendingStop = false;
      pirHoldStart   = now;
      hapticContinuousStart();
      pwrHapticBusy(true);               // 持续震动期间同样禁止进入深睡
      DBGF("[PIR] @%lu ms 引脚高 -> 开始持续震动\n", now);
    } else if (PIR_MIN_HOLD_MS > 0 && (now - pirHoldStart) < PIR_MIN_HOLD_MS) {
      /* 引脚已变低，但还没到"最短持续时长" -> 先记着，到点再停。
       * 这条分支专门对付"只输出一个短脉冲"的 PIR 模块。
       * 注意只打印一次：本分支在引脚为低期间会被反复命中。 */
      if (!pirPendingStop) {
        pirPendingStop = true;
        DBGF("[PIR] 引脚已低，但未到最短持续 %u ms，暂缓停止\n",
             (unsigned)PIR_MIN_HOLD_MS);
      }
    } else {
      pirActive = false;
      hapticStop();
      pwrHapticBusy(false);              // 停止后重新开始 30s 空闲计时
      reportPirDuration(now - pirHoldStart);
    }
  }

  /* 最短持续时长到点 -> 停止 */
  if (pirActive && pirPendingStop && (now - pirHoldStart) >= PIR_MIN_HOLD_MS) {
    pirActive      = false;
    pirPendingStop = false;
    hapticStop();
    pwrHapticBusy(false);
    reportPirDuration(now - pirHoldStart);
  }

  /* 可选保护：超过最长持续时长强制停止（PIR_MAX_HOLD_MS=0 表示不限制） */
#if PIR_MAX_HOLD_MS > 0
  if (pirActive && (now - pirHoldStart) >= PIR_MAX_HOLD_MS) {
    pirActive      = false;
    pirPendingStop = false;
    hapticStop();
    DBGF("[PIR] 超过最长持续 %u ms，强制停止\n", (unsigned)PIR_MAX_HOLD_MS);
  }
#endif

  /* 持续期间周期性重写 RTP（流式保输出）；调试打开时附带电平监控 */
  if (pirActive) {
    if ((now - lastPirRef) >= PIR_HOLD_REFRESH_MS) {
      lastPirRef = now;
      hapticContinuousHold();
    }
#if DEBUG_SERIAL
    if ((now - lastPirDbg) >= 200) {
      lastPirDbg = now;
      DBGF("[PIR] 持续中 %lu ms，引脚=%d\n",
           now - pirHoldStart, (int)digitalRead(PIR_PIN));
    }
#endif
  }

  /* ---------- 8.1 定时采样 + 滤波 ----------
   * ★ 采样周期由**连接状态**决定（现场要求）：
   *     · 已连接（手机/网页在看曲线）-> 固定 SAMPLE_CONNECTED_MS（1s）
   *     · 未连接（待机/广播中）      -> 运行期档位 s_wakePeriodMs（1/3/5/10s，默认 3s，省电）
   *   连上/断开都是**立即生效**（下一个判断就用新周期），不需要重启。
   *   注意：中值窗口(MEDIAN_WIN=5) 与 EMA 都是按**采样点**计的，周期变大后
   *   它们的时间常数会同比例变长（大档位下滤波会更迟钝）。 */
  const uint32_t samplePeriod = deviceConnected ? (uint32_t)SAMPLE_CONNECTED_MS
                                                : (uint32_t)s_wakePeriodMs;
  if (now - lastSample >= samplePeriod) {
    lastSample = now;
    s_newSample = true;              // ★ 采到新样本 -> 让 8.3 段发一次通知（不空发）

    /* 过采样：连读取平均，压掉 ADC 抖动 */
    uint32_t acc = 0;
    for (uint8_t i = 0; i < OS_RATIO; i++) acc += analogRead(FSR_PIN);
    rawVal = (uint16_t)(acc / OS_RATIO);

    uint16_t med = medianFilter(rawVal);                 // 中值
    emaVal += EMA_ALPHA * ((float)med - emaVal);         // EMA
    filtVal = (uint16_t)emaVal;

    if (filtVal > peakVal) peakVal = filtVal;            // 峰值保持

    /* ---------- 8.2 阈值判定（滞回 + 去抖 + 不应期） ---------- */
    bool aboveNow = rawAbove;
    if (!rawAbove && filtVal >= thOn)  aboveNow = true;
    if ( rawAbove && filtVal <= thOff) aboveNow = false;

    if (aboveNow != rawAbove) {
      rawAbove   = aboveNow;
      lastChange = now;
    } else if (aboveNow != pressed && (now - lastChange) >= DEBOUNCE_MS) {
      pressed = aboveNow;
      if (pressed) {
        /* PIR 持续震动期间不再叠加 FSR 短震，避免两种驱动互相打断 */
        if (!pirActive && (now - lastTrigger) >= REFRACTORY_MS) {
          lastTrigger = now;
          hapticFire();
          DBGF("[TRIG] filt=%u raw=%u\n", filtVal, rawVal);
        }
      } else {
        DBGF("[REL ] filt=%u\n", filtVal);
      }
    }
  }

  /* ---------- 8.3 BLE 通知：有新样本才发（不空发、不重复） ----------
   * ★ 原先是"固定每 50ms 发一次"（NOTIFY_HZ=20）：采样变慢时会**重复发同一份
   *   数据**，白白占用连接事件。现在改成跟着采样走 ——
   *     已连接：每采一次发一次（1s 档 -> 1Hz）
   *     未连接：不发（deviceConnected 为假） */
  if (s_newSample && (now - lastNotify) >= (1000 / NOTIFY_HZ)) {
    lastNotify  = now;
    s_newSample = false;

    if (deviceConnected) {
      FsrPacket p;
      p.raw      = rawVal;
      p.filt     = filtVal;
      p.peak     = peakVal;
      p.pressed  = pressed ? 1 : 0;
      p.drv_ok   = drvReady ? 1 : 0;
      p.reserved = 0;
      dataChar.notify((uint8_t*)&p, sizeof(p));

      /* 单值曲线特征：nRF52840 是小端，直接发 2 字节即为 little-endian uint16 */
      uint16_t v_filt = filtVal;
      uint16_t v_raw  = rawVal;
      plotFiltChar.notify((uint8_t*)&v_filt, 2);   // 末位 ac：滤波后压力（画曲线用）
      plotRawChar.notify((uint8_t*)&v_raw, 2);     // 末位 ad：原始压力
    }
  }

  /* ---------- 8.4 串口低频打印 ---------- */
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    DBGF("raw=%4u filt=%4u peak=%4u th=%u/%u drv=%s %s\n",
                  rawVal, filtVal, peakVal, thOn, thOff,
                  drvReady ? "on " : "off",
                  pressed ? "PRESSED" : "-");
  }

  /* ---------- 8.5 LED 指示 ----------
   * PIR 触发期间：红灯常亮（一眼看出"正在震"）
   * 无 BLE 连接：红灯 1Hz 闪烁
   * 有 BLE 连接且 PIR 未触发：由连接回调置为常亮 */
  if (pirActive) {
    pwrLed(true);                      /* PWR_LED_HEARTBEAT=0 时该调用是空操作 */
  }
#if PWR_LED_HEARTBEAT
  else if (!deviceConnected) {
    pwrLed(((now / 500) % 2) != 0);    /* 未连接：1Hz 心跳（仅调试用） */
  }
#else
  else {
    /* 省电模式：灯全灭。红灯/蓝灯都是 mA 级负载，会直接毁掉功耗指标 */
    pwrLed(false);
  }
#endif

  /* ---------- 8.6 电源状态机 ----------
   * 放在循环最后：此时 PIR 状态、震动忙标志、采样值都已更新。
   * pwrTick() 可能直接进入 DEEP_SLEEP 并阻塞，直到压力/PIR 唤醒。 */
  pwrTick();

  /* 空闲时进入系统级休眠等待事件（RTC tick / BLE 事件），
   * 不是 delay() 空转 —— 这样 CPU 在等事件期间真正睡着。 */
  pwrWaitEvent();
}
