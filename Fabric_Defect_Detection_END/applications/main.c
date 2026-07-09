#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <model_sd_start.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <rthw.h>
#include <board.h>
#include <wlan_dev.h>
#include <wlan_mgnt.h>
#include <onenet.h>

/* ==================== 蜂鸣器引脚定义 ==================== */
#define BUZZER              58

/* ==================== wifi名及密码  ==================== */
#define WIFI_SSID     "iPhone14pro"
#define WIFI_PASSWORD "12345678"

/* ==================== 串口定义 ==================== */
#define UART_OPENMV         "uart4"
#define UART_LCD            "uart2"

/* ==================== 电机参数 ==================== */
#define MICRO_STEPS         400
#define PULSE_HALF_CIRC     200
#define PULSE_FULL_CIRC     400

/* ==================== 速度等级对应的微秒周期  ==================== */
#define SPEED_LOW_US        1500
#define SPEED_MID_US        750
#define SPEED_HIGH_US       375

/* ==================== 编码器参数 ==================== */
#define ENCODER_PPR         500
#define WHEEL_CIRCUM_MM     188.4956f
#define ENCODER_4X          (ENCODER_PPR * 4)

/* ==================== 全局变量 ==================== */
static struct rt_device_pwm *pwm_dev1 = RT_NULL;
static struct rt_device_pwm *pwm_dev2 = RT_NULL;
static rt_bool_t g_motor_run = RT_FALSE;
static rt_bool_t g_direction_reverse = RT_FALSE;
static uint16_t g_motor_speed_us = SPEED_MID_US;

static struct rt_semaphore g_rx_sem;
static rt_device_t g_uart4, g_uart2;
uint8_t image_count = 0;

extern uint8_t detection_round_cnt;
extern rt_mutex_t detection_round_mutex;

/* ==================== 闭环控制全局变量 ==================== */
static rt_bool_t closed_loop_enable = RT_FALSE;
static float target_speed_mmps = 200.0f;
static float Kp = 1.0f, Ki = 0.05f, Kd = 0.0f;
static float filtered_speed = 0.0f;
static float filter_alpha = 0.5f;

/* ==================== 函数声明 ==================== */
static void motor_set_direction(rt_bool_t reverse);
static void motor_continuous_stop(void);
static void motor_move_steps(uint8_t motor_id, uint32_t steps, rt_bool_t reverse);
static void motor_set_speed_rpm_both(uint16_t rpm);
static void motor_hw_init(void);
static void motor_set_active_speed_rpm(uint16_t rpm);
static void motor_set_speed_level(uint16_t period_us);
static void motor_accel(int start_rpm, int target_rpm, int step_rpm, int step_delay_ms);

/* ==================== 编码器相关 ==================== */
int32_t encoder_get_count(void)
{
    return (int32_t)TIM3->CNT;
}

void encoder_reset_count(void)
{
    TIM3->CNT = 0;
}

static float get_line_speed_mmps(void)
{
    static int32_t last_count = 0;
    static uint32_t last_time_ms = 0;
    int32_t current = encoder_get_count();
    uint32_t now = rt_tick_get();
    float dt = (now - last_time_ms) * 1.0f / RT_TICK_PER_SECOND;
    if (dt <= 0) return filtered_speed;  // 返回上次滤波值
    int32_t delta = current - last_count;
    if (delta > 32767) delta -= 65536;
    else if (delta < -32767) delta += 65536;
//    delta = -delta;   // 方向修正
    float dist = delta * WHEEL_CIRCUM_MM / ENCODER_4X;
    float raw_speed = dist / dt;
    // 一阶低通滤波
    if (filtered_speed == 0) filtered_speed = raw_speed;
    else filtered_speed = filter_alpha * raw_speed + (1 - filter_alpha) * filtered_speed;

    // 调试打印（可选，降低频率）
    static uint32_t last_print = 0;
    if (rt_tick_get() - last_print > RT_TICK_PER_SECOND) {
        rt_kprintf("[ENC] raw=%.1f, filt=%.1f mm/s\n", raw_speed, filtered_speed);
        last_print = rt_tick_get();
    }

    last_count = current;
    last_time_ms = now;
    return filtered_speed;
}

static void speed_pid_thread(void *param)
{
    float integral = 0, prev_error = 0;
    float Ki_limit = 50.0f;
    rt_bool_t was_enabled = RT_FALSE;
    uint32_t print_tick = 0;

    // PID 系数（抖动时减小）
    float dt = 0.02f;  // 控制周期 20ms

    while (1) {
        if (closed_loop_enable) {
            if (!was_enabled) {
                integral = 0;
                prev_error = 0;
                filtered_speed = 0;
                was_enabled = RT_TRUE;
                rt_kprintf("PID started, target=%.1f mm/s, Kp=%.2f Ki=%.2f\n", target_speed_mmps, Kp, Ki);
            }
            float current_speed = get_line_speed_mmps();
            float error = target_speed_mmps - current_speed;
            integral += error * dt;
            if (integral > Ki_limit) integral = Ki_limit;
            if (integral < -Ki_limit) integral = -Ki_limit;
            float derivative = (error - prev_error) / dt;
            float output = Kp * error + Ki * integral + Kd * derivative;
            if (output > 800) output = 800;
            if (output < 0) output = 0;
            motor_set_active_speed_rpm((uint16_t)output);
            prev_error = error;

            if (rt_tick_get() - print_tick > RT_TICK_PER_SECOND) {
                rt_kprintf("[PID] cur=%.1f mm/s, err=%.2f, out=%.1f RPM\n", current_speed, error, output);
                print_tick = rt_tick_get();
            }
        } else {
            was_enabled = RT_FALSE;
            integral = 0;
            prev_error = 0;
            filtered_speed = 0;
        }
        rt_thread_mdelay(20);  // 20ms 周期
    }
}

/* ==================== 电机辅助函数 ==================== */
static void motor_set_active_speed_rpm(uint16_t rpm)
{
    if (pwm_dev1 == RT_NULL || pwm_dev2 == RT_NULL) return;
    uint32_t freq_hz = (rpm * MICRO_STEPS) / 60;
    if (freq_hz < 1) freq_hz = 1;
    uint32_t period_ns = 1000000000UL / freq_hz;
    uint32_t pulse_ns = period_ns / 2;
    if (!g_direction_reverse) {
        rt_pwm_set(pwm_dev1, 1, period_ns, pulse_ns);
        // 可选：打印电机1的设置
        // rt_kprintf("[M1] set RPM=%d, freq=%d Hz\n", rpm, freq_hz);
    } else {
        rt_pwm_set(pwm_dev2, 1, period_ns, pulse_ns);
        // rt_kprintf("[M2] set RPM=%d, freq=%d Hz\n", rpm, freq_hz);
    }
}

static void motor_set_speed_rpm_both(uint16_t rpm)
{
    if (pwm_dev1 == RT_NULL || pwm_dev2 == RT_NULL) return;
    uint32_t freq_hz = (rpm * MICRO_STEPS) / 60;
    if (freq_hz < 1) freq_hz = 1;
    uint32_t period_ns = 1000000000UL / freq_hz;
    uint32_t pulse_ns = period_ns / 2;
    rt_pwm_set(pwm_dev1, 1, period_ns, pulse_ns);
    rt_pwm_set(pwm_dev2, 1, period_ns, pulse_ns);
}

static void motor_set_direction(rt_bool_t reverse)
{
    if (reverse == RT_FALSE) {
        rt_pin_write(MOTOR1_DIR_PLUS, PIN_LOW);
        rt_pin_write(MOTOR2_DIR_PLUS, PIN_LOW);
        rt_pin_write(MOTOR1_ENA, PIN_HIGH);
        rt_pin_write(MOTOR2_ENA, PIN_LOW);
    } else {
        rt_pin_write(MOTOR1_DIR_PLUS, PIN_HIGH);
        rt_pin_write(MOTOR2_DIR_PLUS, PIN_HIGH);
        rt_pin_write(MOTOR1_ENA, PIN_LOW);
        rt_pin_write(MOTOR2_ENA, PIN_HIGH);
    }
}

static void motor_continuous_stop(void)
{
    if (g_motor_run) {
        g_motor_run = RT_FALSE;
        rt_pwm_disable(pwm_dev1, 1);
        rt_pwm_disable(pwm_dev2, 1);
        rt_kprintf("Motors stopped\n");
    } else {
        // 确保PWM输出关闭
        rt_pwm_disable(pwm_dev1, 1);
        rt_pwm_disable(pwm_dev2, 1);
    }
}

static void motor_move_steps(uint8_t motor_id, uint32_t steps, rt_bool_t reverse)
{
    // 如果闭环正在运行，先停止闭环
    if (closed_loop_enable) {
        closed_loop_enable = RT_FALSE;
        motor_continuous_stop();
        rt_thread_mdelay(10);
    }
    // 停止当前任何运行
    motor_continuous_stop();
    rt_thread_mdelay(10);

    // 固定点动频率 500Hz (对应 75 RPM, 400细分)
    uint32_t freq_hz = 500;
    uint32_t period_ns = 1000000000UL / freq_hz;
    uint32_t pulse_ns = period_ns / 2;

    if (motor_id == 1) {
        rt_pin_write(MOTOR1_DIR_PLUS, reverse ? PIN_HIGH : PIN_LOW);
        rt_pin_write(MOTOR1_ENA, PIN_HIGH);
        rt_pin_write(MOTOR2_ENA, PIN_LOW);
        rt_pwm_set(pwm_dev1, 1, period_ns, pulse_ns);
        rt_pwm_enable(pwm_dev1, 1);
    } else {
        rt_pin_write(MOTOR2_DIR_PLUS, reverse ? PIN_HIGH : PIN_LOW);
        rt_pin_write(MOTOR2_ENA, PIN_HIGH);
        rt_pin_write(MOTOR1_ENA, PIN_LOW);
        rt_pwm_set(pwm_dev2, 1, period_ns, pulse_ns);
        rt_pwm_enable(pwm_dev2, 1);
    }

    uint32_t time_ms = (steps * 1000) / freq_hz;
    if (time_ms < 20) time_ms = 20;
    rt_thread_mdelay(time_ms);

    if (motor_id == 1) {
        rt_pwm_disable(pwm_dev1, 1);
    } else {
        rt_pwm_disable(pwm_dev2, 1);
    }

    // 恢复两个电机为从动
    rt_pin_write(MOTOR1_ENA, PIN_LOW);
    rt_pin_write(MOTOR2_ENA, PIN_LOW);
    // 恢复 PWM 周期为当前速度等级（避免影响下次开环运行）
    motor_set_speed_level(g_motor_speed_us);
}

static void motor_accel(int start_rpm, int target_rpm, int step_rpm, int step_delay_ms)
{
    if (start_rpm >= target_rpm) {
        motor_set_speed_rpm_both(target_rpm);
        if (!g_motor_run) {
            g_motor_run = RT_TRUE;
            motor_set_direction(g_direction_reverse);
            rt_pwm_enable(pwm_dev1, 1);
            rt_pwm_enable(pwm_dev2, 1);
        }
        return;
    }
    motor_set_speed_rpm_both(start_rpm);
    if (!g_motor_run) {
        g_motor_run = RT_TRUE;
        motor_set_direction(g_direction_reverse);
        rt_pwm_enable(pwm_dev1, 1);
        rt_pwm_enable(pwm_dev2, 1);
    }
    rt_thread_mdelay(step_delay_ms);
    for (int rpm = start_rpm + step_rpm; rpm <= target_rpm; rpm += step_rpm) {
        motor_set_speed_rpm_both(rpm);
        rt_thread_mdelay(step_delay_ms);
    }
    motor_set_speed_rpm_both(target_rpm);
}

static void motor_set_speed_level(uint16_t period_us)
{
    g_motor_speed_us = period_us;
    // 开环模式下，如果电机正在运行，立即改变速度
    if (g_motor_run && !closed_loop_enable) {
        uint32_t freq_hz = 1000000UL / period_us;
        uint32_t rpm = (freq_hz * 60UL) / MICRO_STEPS;
        uint32_t pulse_freq = (rpm * MICRO_STEPS) / 60;
        if (pulse_freq < 1) pulse_freq = 1;
        uint32_t period_ns = 1000000000UL / pulse_freq;
        uint32_t pulse_ns = period_ns / 2;
        rt_pwm_set(pwm_dev1, 1, period_ns, pulse_ns);
        rt_pwm_set(pwm_dev2, 1, period_ns, pulse_ns);
    }
}

static void motor_hw_init(void)
{
    rt_pin_mode(MOTOR1_PULSE_NEG, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR1_PULSE_NEG, PIN_LOW);
    rt_pin_mode(MOTOR2_PULSE_NEG, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR2_PULSE_NEG, PIN_LOW);

    rt_pin_mode(MOTOR1_DIR_PLUS, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR1_DIR_MINUS, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR1_DIR_MINUS, PIN_LOW);
    rt_pin_mode(MOTOR2_DIR_PLUS, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR2_DIR_MINUS, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR2_DIR_MINUS, PIN_LOW);

    rt_pin_mode(MOTOR1_ENA, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR2_ENA, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR1_ENA, PIN_LOW);
    rt_pin_write(MOTOR2_ENA, PIN_LOW);

    pwm_dev1 = (struct rt_device_pwm *)rt_device_find("pwm1");
    pwm_dev2 = (struct rt_device_pwm *)rt_device_find("pwm2");
    if (pwm_dev1 == RT_NULL || pwm_dev2 == RT_NULL) {
        rt_kprintf("Error: PWM device not found!\n");
        return;
    }
    rt_kprintf("pwm_dev1 = %p, pwm_dev2 = %p\n", pwm_dev1, pwm_dev2);

    g_motor_speed_us = SPEED_MID_US;
}

/* ==================== WiFi/OneNET线程 ==================== */
static int wifi_auto_connect(void)
{
    rt_wlan_set_mode(RT_WLAN_DEVICE_STA_NAME, RT_WLAN_STATION);
    if (rt_wlan_connect(WIFI_SSID, WIFI_PASSWORD) == RT_EOK) {
        rt_thread_mdelay(500);
        return 0;
    } else {
        LOG_E("WiFi connect failed");
        return -1;
    }
}

static int onenet_auto_init(void)
{
    if (onenet_mqtt_init() < 0) {
        LOG_E("OneNET init failed");
        return -1;
    }
    LOG_I("OneNET MQTT connected");
    return 0;
}

static void auto_net_init_thread(void *param)
{
    rt_thread_mdelay(1000);
    if (wifi_auto_connect() == 0) {
        onenet_auto_init();
    }
}

/* ==================== 串口接收回调及处理线程 ==================== */
static rt_err_t uart_rx_cb(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&g_rx_sem);
    return RT_EOK;
}

static void uart_ctrl_thread_entry(void *param)
{
    rt_kprintf("uart_ctrl_thread started\n");

    char ch = 0;
    rt_size_t recv_len = 0;
    while (1) {
        rt_sem_take(&g_rx_sem, RT_WAITING_FOREVER);
        recv_len = rt_device_read(g_uart4, -1, &ch, 1);
        if (recv_len != 1)
            recv_len = rt_device_read(g_uart2, -1, &ch, 1);
        if (recv_len == 1) {
            switch (ch) {
                case 'b':   // 开始识别
                    if (closed_loop_enable) closed_loop_enable = RT_FALSE;
                    motor_continuous_stop();
                    rt_device_write(g_uart4, 0, "pp", 2);
                    rt_pin_write(BUZZER, PIN_HIGH);
                    rt_thread_mdelay(100);
                    rt_pin_write(BUZZER, PIN_LOW);
                    break;
                case 's':   // 停止
                    if (closed_loop_enable) closed_loop_enable = RT_FALSE;
                    motor_continuous_stop();
                    break;
                case 'u':   // 正转闭环（带软启动）
                    // 1. 停止当前所有运动
                    if (closed_loop_enable) closed_loop_enable = RT_FALSE;
                    motor_continuous_stop();
                    rt_thread_mdelay(20);

                    // 2. 设置方向和使能（主动电机使能，从动电机失能）
                    g_direction_reverse = RT_FALSE;
                    motor_set_direction(g_direction_reverse);

                    // 3. 计算目标转速（根据目标线速度和计米轮直径）
                    uint32_t target_rpm = (uint32_t)(target_speed_mmps * 60.0f / (3.14159f * 60.0f));
                    if (target_rpm < 30) target_rpm = 30;   // 最低启动转速

                    // 4. 开环加速到目标转速（梯形加速，从 30 RPM 开始）
                    motor_accel(50, target_rpm, 5, 80);

                    // 5. 加速完成后，再使能闭环（此时电机已在目标转速附近，PID 仅需微调）
                    closed_loop_enable = RT_TRUE;
                    // 注意：PID 线程中的 was_enabled 会在下一个周期重置积分，速度反馈从当前值开始
                    rt_kprintf("Soft start finished, closed-loop enabled. target speed: %.1f mm/s\n", target_speed_mmps);
                    break;
                case 'o':   // 反转闭环
                    motor_continuous_stop();
                    closed_loop_enable = RT_TRUE;
                    g_direction_reverse = RT_TRUE;
                    motor_set_direction(g_direction_reverse);
                    rt_pwm_enable(pwm_dev2, 1);
                    rt_pwm_disable(pwm_dev1, 1);
                    rt_kprintf("Closed-loop reverse started, target speed: %.1f mm/s\n", target_speed_mmps);
                    break;
                // 左电机点动
                case 'h': motor_move_steps(1, PULSE_HALF_CIRC, RT_FALSE); break;
                case 'a': motor_move_steps(1, PULSE_FULL_CIRC, RT_FALSE); break;
                case 'y': motor_move_steps(1, PULSE_HALF_CIRC, RT_TRUE);  break;
                case 'z': motor_move_steps(1, PULSE_FULL_CIRC, RT_TRUE);  break;
                // 右电机点动
                case 'd': motor_move_steps(2, PULSE_HALF_CIRC, RT_FALSE); break;
                case 'x': motor_move_steps(2, PULSE_FULL_CIRC, RT_FALSE); break;
                case 'p': motor_move_steps(2, PULSE_HALF_CIRC, RT_TRUE);  break;
                case 'q': motor_move_steps(2, PULSE_FULL_CIRC, RT_TRUE);  break;
                // 调速（修改目标线速度，闭环运行时立即生效）
                case 'w':
                    target_speed_mmps = 300.0f;
                    rt_kprintf("Target speed set to %.1f mm/s\n", target_speed_mmps);
                    break;
                case 'm':
                    target_speed_mmps = 200.0f;
                    rt_kprintf("Target speed set to %.1f mm/s\n", target_speed_mmps);
                    break;
                case 'l':
                    target_speed_mmps = 100.0f;
                    rt_kprintf("Target speed set to %.1f mm/s\n", target_speed_mmps);
                    break;
                // 开环正转测试（临时，可用于对比）
                case 't':
                    if (closed_loop_enable) closed_loop_enable = RT_FALSE;
                    motor_continuous_stop();
                    g_direction_reverse = RT_FALSE;
                    motor_set_direction(g_direction_reverse);
                    motor_accel(100, 200, 20, 50);
                    break;
                // 图像计数
                case '(':
                    image_count++;
                    break;
                case ')':
                    rt_pin_write(BUZZER, PIN_HIGH);
                    rt_thread_mdelay(80);
                    rt_pin_write(BUZZER, PIN_LOW);
                    rt_thread_mdelay(80);
                    rt_pin_write(BUZZER, PIN_HIGH);
                    rt_thread_mdelay(80);
                    rt_pin_write(BUZZER, PIN_LOW);
                    rt_kprintf("image_count = %d\n", image_count);
                    rt_mutex_take(detection_round_mutex, RT_WAITING_FOREVER);
                    detection_round_cnt = image_count;
                    rt_mutex_release(detection_round_mutex);
                    image_count = 0;
                    break;
                default: break;
            }
        }
        ch = 0;
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    rt_thread_t tid = RT_NULL;

    sd_init();
    motor_hw_init();
    encoder_hw_init();

    rt_pin_mode(BUZZER, PIN_MODE_OUTPUT);
    rt_pin_write(BUZZER, PIN_LOW);

    g_uart4 = rt_device_find(UART_OPENMV);
    g_uart2 = rt_device_find(UART_LCD);
    rt_device_open(g_uart4, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_open(g_uart2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(g_uart4, uart_rx_cb);
    rt_device_set_rx_indicate(g_uart2, uart_rx_cb);
    rt_sem_init(&g_rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    tid = rt_thread_create("uart_ctrl", uart_ctrl_thread_entry, RT_NULL, 4096, 20, 2);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("netinit", auto_net_init_thread, RT_NULL, 4096, 20, 5);
    if (tid) rt_thread_startup(tid);

    tid = rt_thread_create("pidctrl", speed_pid_thread, RT_NULL, 2048, 12, 10);
    if (tid) rt_thread_startup(tid);

    LOG_I("System init success!");
    return RT_EOK;
}
