#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

/* ============================================================================
 *  电源状态机配置（现场可调）
 *  硬件：XIAO nRF52840 + Bluefruit（SoftDevice S140）
 * ----------------------------------------------------------------------------
 *  ★ 现场最常用的三个宏（按需改这里即可）
 *      WAKE_THRESHOLD   唤醒阈值（ADC 计数，12bit -> 0~4095）
 *      IDLE_TIMEOUT_S   广播后无连接多久进深度睡眠
 *      RTC_PERIOD_MS    深度睡眠中的采样周期
 * ==========================================================================*/

/* ---------- 可直接用 -D 覆盖的宏 ---------- */
#ifndef WAKE_THRESHOLD
#define WAKE_THRESHOLD      500      /* ADC 计数超过它就唤醒（默认 ADC>500） */
#endif

/* ---------- 广播策略：两级广播（= "定时唤醒"的正确实现，默认） ----------
 * "不连蓝牙时低功耗" 与 "要能连上蓝牙" 天生矛盾：
 *   广播一停，手机再也搜不到；广播一直开，radio 就一直耗电。
 * 正确解法是让**协议栈自己定时醒来发一次广播包**（这就是定时唤醒）：
 *
 *   ① 快速窗口（ADV_FAST_INTERVAL_MS 间隔，持续 ADV_FAST_WINDOW_S 秒）
 *      开机 / 刚断连 / 有人交互（PIR、压力）时进入 —— 手机 1 秒内就能连上
 *   ② 待机慢广播（ADV_SLOW_INTERVAL_MS 间隔），窗口过后自动切换并一直保持
 *      平均电流只有几 µA 量级，**但仍然可以被连接**，只是手机"发现"它要慢几秒
 *
 * ★ 慢广播期间"能连上"是真的能连：连接请求只要在任意一次广播包之后发出
 *   即可建立链路，我们不需要做任何唤醒动作，手机也不需要等某个"窗口"。
 *   切换动作由 SoftDevice 自己完成，CPU 全程在睡，不耗额外电流。 */
/* ---------- 广播间隔（**固定**，不再运行期可切） ----------
 * 固定 1s：连续广播、永不停止、永不重配置 —— 保证"任何时候手机都能发现并连上"。
 * ★ 实测教训（务必记住）：**广播间隔越大越难被发现**。同一位置 15s 扫描：
 *   1s 档被发现 7 次、3s 档只有 2 次、10s 档几乎扫不到；手机侧表现就是"连不上"。
 *   所以**不要**为了省电把它调大 —— 省电请用下面的采样唤醒周期（不影响连接）。
 * 注：早期版本把 1s/3s/5s/10s 做成"广播间隔档位"，实测直接导致"扫不到/连不上"，
 *     现已改为：**档位只控采样唤醒周期，广播固定 1s**。 */
#ifndef ADV_FAST_INTERVAL_MS
#define ADV_FAST_INTERVAL_MS  1000    /* 固定广播间隔（改这里等于改所有档位的可连性） */
#endif

/* ---------- 采样（RTC）唤醒周期：运行期可配 1s / 3s / 5s / 10s，默认 3s ----------
 * 主循环不再 50Hz 连续采样，而是每这么久醒一次读 FSR / 更新 BLE 通知。
 *   · 越大越省电（CPU、ADC、通知都少了）
 *   · 代价：压力触发变迟钝（踩下去要等下一个采样点），短促踩踏可能漏掉
 * ★ 注意：这一项**不影响"手机能不能很快扫到它"** —— 那是广播间隔（固定 1s）决定的。
 * 运行期切换：BLE 写 FEE2(...a9) '1' -> 1s、'3' -> 3s、'5' -> 5s、'0' -> 10s
 *   （网页 fsr_dashboard.html 上有 "Wake 1/3/5/10 s" 下拉框，选中即发送） */
#ifndef SAMPLE_WAKE_PERIOD_MS
#define SAMPLE_WAKE_PERIOD_MS 3000    /* 开机默认档（3s，按现场要求） */
#endif

/* ---------- 已连接时的采样周期：固定 1s ----------
 * 只要 BLE 连着（手机/网页在看曲线），就按 1s 采一次（1Hz）；
 * **断开后回到上面的档位周期**（默认 3s）省电。
 * 实现：nrf52840-pressure-haptic.ino（主草图）的 loop() 8.1 段按 `deviceConnected` 二选一；
 *      通知也随之改成"每采到一次新样本发一次"（8.3 段），不再空发。 */
#ifndef SAMPLE_CONNECTED_MS
#define SAMPLE_CONNECTED_MS  1000
#endif
#ifndef ADV_FAST_WINDOW_S
#define ADV_FAST_WINDOW_S     15      /* 快速广播保持多少秒后降为慢广播 */
#endif
#ifndef ADV_SLOW_INTERVAL_MS
/* 待机广播间隔（仅在 ADV_USE_SLOW_PHASE=1 时才用得上）。
 * ★ 实测教训：这个值越大，手机"连不上"的概率越高 —— 连接请求必须等下一包
 *   广播，3000/5000ms 时不少手机/App 会判超时失败。现在默认不再使用慢播阶段。 */
#define ADV_SLOW_INTERVAL_MS  1000
#endif

/* ---------- 是否启用"快播窗口 -> 慢播待机"两级策略（默认关闭） ----------
 * 0 = **单一固定间隔广播，永不重配置**（默认，可靠性最好）
 * 1 = 开机/交互后快播 ADV_FAST_WINDOW_S 秒，然后自动降为 ADV_SLOW_INTERVAL_MS
 *
 * ★ 为什么默认关闭（这是"蓝牙连接有时候会失败"的主因）：
 *   ① 快播超时后协议栈要重新配置广播（ble_gap_adv_set_configure + adv_start），
 *      这一瞬间到达的连接请求会被丢弃；
 *   ② 慢播间隔大时（3000~5000ms），连接请求要等下一包广播，容易被中心设备判超时。
 *   而快/慢播之间省下的只是几 µA radio 电流，不值得牺牲连接成功率。
 * 所以默认：固定 ADV_FAST_INTERVAL_MS（默认 3000ms）一直播，setFastTimeout(0)，
 * 协议栈全程不重配置、广播连续不断 —— 任何时候尝试连接都能立刻建立链路。
 * 间隔本身在运行期用 BLE 命令切档即可，不需要重新编译。 */
#ifndef ADV_USE_SLOW_PHASE
#define ADV_USE_SLOW_PHASE    0
#endif

/* ---------- 真正的深睡（默认关闭！） ----------
 * DEEP_SLEEP_AFTER_S > 0 时：空闲该秒数后**停广播并进深睡**。
 * ⚠ 代价必须明确：一旦停广播，手机就**完全搜不到设备**，必须靠 PIR/压力
 *   （GPIO 电平跳变）唤醒或断电重启。也就是说打开它就会重新变成"连不上蓝牙"。
 *   仅在做电流验收（要把平均电流压到最低）时才建议临时设成 30。
 * 默认 0 = 不深睡，走上面的两级广播策略。 */
#ifndef DEEP_SLEEP_AFTER_S
#define DEEP_SLEEP_AFTER_S    0
#endif

#ifndef RTC_PERIOD_MS
#define RTC_PERIOD_MS       800      /* 深睡中每 800ms 醒一次采样 FSR */
#endif

/* ---------- 测量模式（做电流验收时打开） ----------
 * 1 = **完全不初始化 USB 串口**：连 Serial.begin 都不调用，日志宏也一起关掉。
 *     这是必须的，因为只要 USB 被初始化（CDC 枚举 + 每 1ms 一个 SOF 中断），
 *     外设本身就会吃掉几百 µA~mA 级，且 CPU 会被 SOF 每秒唤醒 1000 次，
 *     **根本测不到真正的低功耗**。
 * 0 = 正常（默认）：有串口日志，方便调试。
 * ⚠ 打开后：① 无法再看到任何日志；
 *          ② **烧录方式要改**：USB 不枚举成 COM 口，arduino-cli 不能按端口烧，
 *             需要**双击 XIAO 上的 RESET 键**进入 UF2 引导盘，再把生成的
 *             .uf2 文件拷进去（或烧完测量版后，要改回来时用这个方法）。
 * 建议流程：先用 0 调试/确认功能 → 把 1 烧进去 → 拔 USB、用电池供电测量。 */
#ifndef PWR_MEASURE_MODE
#define PWR_MEASURE_MODE     0
#endif

/* ---------- 空闲睡眠原语（省电关键） ----------
 * 1 = 用裸 __WFE()（**默认**）
 * 0 = 用 SoftDevice 的 sd_app_evt_wait()
 *
 * ★ 为什么默认用裸 WFE（实测结论，务必保留）：
 *   本核心（Seeeduino nRF52 1.1.13）+ SoftDevice S140 组合下，
 *   sd_app_evt_wait() **每次都立即返回**，主循环因此变成空转 ——
 *   实测 loop=45000 次/秒，CPU 全速运行，静态电流多 3~6mA。
 *   这正是现场"待机还有 5.6mA"的来源，与模块、外设都无关。
 *   换成标准 WFE 序列后 CPU 才会真正睡到下一个中断。
 *   安全性：SoftDevice 的协议栈活动（广播/连接事件、内部定时器）本身就是
 *   中断源，会把 CPU 唤醒，不会睡死。
 * 自检：日志 loop=xx/s 应为**个位数/秒**；USB 插着时约 1000/s（SOF 每 1ms
 *   一次），这也说明"插着 USB 就测不出低功耗"。 */
#ifndef PWR_SLEEP_USE_WFE
#define PWR_SLEEP_USE_WFE    1
#endif

/* ---------- 日志开关 ---------- */
/* 1 = 打印 [pwr] 日志（开发调试用）
 * 0 = 完全不打印
 * ★★ 重要：只要 USB 串口被初始化，USB 外设就会把电流抬到数百 µA 以上，
 *    此时**根本测不到 DEEP_SLEEP < 20µA**。做电流验收时必须：
 *      ① 把这里设为 0 重新烧录；② 用电池/3V3 供电、拔掉 USB；
 *      ③ 或者接受"打印时只能看状态、不能看电流"。 */
#ifndef PWR_LOG_ENABLE
#define PWR_LOG_ENABLE      1
#endif

/* ---------- 时间参数 ---------- */
#define PWR_AWAKE_TICK_MS   20       /* 清醒态 RTC tick：驱动 20ms 采样节拍 */
#define CONN_UPDATE_DELAY_MS 3000    /* 连接建立后多久发起参数更新请求 */

/* ---------- 指示灯（总开关） ----------
 * 0 = **关闭全部指示灯**（省电必须，当前默认）
 *     包括：① 未连接时的红灯 1Hz 心跳；② 连接后红灯常亮；
 *           ③ PIR 触发时红灯常亮；④ Bluefruit 库的蓝灯广播闪烁
 * 1 = 全部打开（调试/演示时方便看状态）
 * ★ 红灯（经限流电阻）常亮或闪烁都是 **mA 级**电流，蓝灯同理。
 *   只要有一个灯在亮，ADVERTISING 就不可能 < 300µA、DEEP_SLEEP 更不可能 < 20µA。
 *   所以做电流验收前务必保持 0，尤其别忘 ④ —— 那是库自带的，
 *   由 initBLE() 里的 Bluefruit.autoConnLed(false) 一并关掉。 */
#ifndef PWR_LED_HEARTBEAT
#define PWR_LED_HEARTBEAT   0
#endif

/* ---------- 连接参数 ---------- */
/* ★ 你给的 1000/2000ms + latency 2 + supervision 4000ms 是**非法组合**，
 *   中心设备一定会拒绝，原因见下面两条硬规则：
 *
 *   规则 1（BLE 规范，硬性）：
 *        sup_timeout > (1 + slave_latency) × max_interval × 2
 *        校验你的取值：(1+2) × 2000ms × 2 = 12000ms > 4000ms  → 不成立 ✗
 *   规则 2（Apple 对配件的建议，影响 iOS 是否接受）：
 *        max_interval × (slave_latency + 1) ≤ 2000ms
 *        校验你的取值：2000ms × 3 = 6000ms > 2000ms        → 不成立 ✗
 *
 *   因此默认值改用下面这组（既满足规范，也满足 Apple 约束，平均功耗同样很低）：
 *        interval 1000ms，latency 1，sup_timeout 6000ms
 *        规则1: (1+1)×1000×2 = 4000ms < 6000ms ✓
 *        规则2: 1000×(1+1) = 2000ms ≤ 2000ms ✓
 *   latency=1 意味着中心每 2 个连接间隔才必须联系一次 → 等效唤醒周期 2s，
 *   配合 1000ms 间隔，平均电流仍然很低。 */
/* ★★ 连接初期必须用**快间隔**，否则手机会"连不上" ★★
 * 实测现象：设备能扫到、但手机报连接失败。
 * 原因：原来 min=max=1000ms，手机连上后要做服务发现（十几个 ATT 往返），
 *   一个连接事件只能走一次往返 -> 1000ms×N 才能完成，多数 App 直接判超时失败。
 * 做法：PPCP 里声明 [CONN_INTERVAL_FAST_MS, CONN_INTERVAL_MS]，
 *   手机先按 30ms 快间隔连上（服务发现 200ms 内完成），
 *   3 秒后再由固件发起参数更新降到 1000ms（省电）。
 * 实测：改成快间隔后，连接正常。 */
#ifndef CONN_INTERVAL_FAST_MS
#define CONN_INTERVAL_FAST_MS 30     /* 连接建立初期（服务发现用） */
#endif

#ifndef CONN_INTERVAL_MS
#define CONN_INTERVAL_MS    1000     /* 稳定后的连接间隔（1.25ms 单位的倍数，程序内换算） */
#endif
#ifndef CONN_SLAVE_LATENCY
#define CONN_SLAVE_LATENCY  1
#endif
#ifndef CONN_SUP_TIMEOUT_MS
#define CONN_SUP_TIMEOUT_MS 6000
#endif

/* ---------- 休眠方式 ---------- */
/* 0 = System ON 低功耗（sd_app_evt_wait + RTC2 周期唤醒）——**默认，推荐**
 * 1 = System OFF（最省电，~1µA）
 *
 * ★ 为什么默认不是 System OFF：
 *   System OFF 下 **RTC 与 SAADC 都被断电**，无法"每 800ms 醒一次读 ADC"，
 *   也无法用 ADC 阈值唤醒。System OFF 只能被 GPIO DETECT（数字电平跳变）
 *   或 NFC/复位唤醒 —— 所以它**实现不了**本任务第 3 条需求。
 *   若确实要极低功耗，把这里设为 1：唤醒改为 FSR 分压节点电平拉高
 *   （需要按压力度足够大，超过 VIH≈0.7×VDD≈2.3V 才会触发），
 *   唤醒后是一次复位，配置需靠 GPREGRET 保存。 */
#ifndef PWR_USE_SYSTEM_OFF
#define PWR_USE_SYSTEM_OFF  0
#endif

/* ---------- 深睡时是否断开 I2C 负载 ---------- */
/* DRV2605 处于 STANDBY 时自身约 1~3µA，但**模块上的 LDO/电平转换会额外耗电**。
 * 若实测深睡电流偏大，可把这里设为 1：进入深睡时把 DRV2605 的 MODE 写回
 * STANDBY(0x40)，唤醒后再重新初始化。 */
#ifndef PWR_SLEEP_DRV_STANDBY
#define PWR_SLEEP_DRV_STANDBY 1
#endif

/* ---------- 状态定义 ---------- */
typedef enum {
  PWR_ST_BOOT = 0,
  PWR_ST_CONNECTED,
  PWR_ST_ADVERTISING,      /* 快速广播窗口（易被连接） */
  PWR_ST_IDLE_ADV,         /* 待机慢广播：低功耗但仍可被连接 */
  PWR_ST_DEEP_SLEEP,       /* 停广播深睡：手机完全搜不到（需 GPIO 唤醒） */
} pwr_state_t;

/* 每个状态的验收目标（仅用于打印提示，不参与逻辑） */
#define PWR_TARGET_CONNECTED      "< 1000uA"
#define PWR_TARGET_ADVERTISING    "< 300uA"
#define PWR_TARGET_IDLE_ADV       "< 100uA"   /* 慢广播待机（可连接） */
#define PWR_TARGET_DEEP_SLEEP     "< 20uA"    /* 停广播深睡（不可连接） */

/* ---------- 编译期校验连接参数（不合法直接编译报错，避免现场白测） ---------- */
#define PWR_SUP_TIMEOUT_MIN_MS  (((CONN_SLAVE_LATENCY) + 1) * (CONN_INTERVAL_MS) * 2 + 1)
#if (CONN_SUP_TIMEOUT_MS) < PWR_SUP_TIMEOUT_MIN_MS
#error "CONN_SUP_TIMEOUT_MS 太小：必须 > (1+latency)x interval x2，见 power_config.h 说明"
#endif

#endif /* POWER_CONFIG_H */
