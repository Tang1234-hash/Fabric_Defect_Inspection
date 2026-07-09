import sensor,os, machine,image
import time
from pyb import Timer
from pyb import UART
from pyb import Pin

############不能用P7#############
# 配置
W, H = 32, 32
IMAGE_SIZE = 1024
FRAME_HEAD = 0xA6
FRAME_TAIL_1 = 0x5D
FRAME_TAIL_2 = 0x5B

# uart3 电机控制 uart1 传输图像 P8 光电门 P9 舵机
uart3 = UART(3, 115200, timeout_char=200) #P4发
uart1 = UART(1, 9600, timeout_char=200) #P1收

# 🔴 修正阈值：保留完整灰度范围，避免极端二值化
thresholds = [
    (0, 127),   # 背景/暗区
    (128, 255)  # 亮区/目标
]

tim = Timer(2, freq=100)  # 定时器不变

if not "images" in os.listdir():
    os.mkdir("images")
if not "compressed" in os.listdir():
    os.mkdir("compressed")

# 定时器回调：不变
count=0
def tick(timer):
    global count
    count+=1

servo_pin = Pin('P9', Pin.OUT_PP)
door_pin = Pin('P8', Pin.IN, Pin.PULL_UP)

# 舵机函数：不变
def servo_set_angle(angle):
    angle = max(-90, min(90, angle))
    high_time_ms = 0.5 + (angle + 90) * (2.0 / 180)
    low_time_ms = 20 - high_time_ms
    servo_pin.high()
    time.sleep_us(int(high_time_ms * 1000))
    servo_pin.low()
    time.sleep_us(int(low_time_ms * 1000))

def servo_rotate(angle, delay=120):
    for _ in range(30):
        servo_set_angle(angle)
    time.sleep_ms(delay)

# ===================== 发送函数：完全修复 =====================
def send_frame(img_num_tran):
    try:
        # 🔴 读取32x32 PGM图（未压缩，直接可用）
        img = image.Image(f"compressed/{img_num_tran}.pgm", copy_to_fb=True)
    except Exception as e:
        print(f"读失败: {e}")
        return

    data = []
    # 确保是灰度图
    if img.format() != sensor.GRAYSCALE:
        img = img.to_grayscale()

    # 🔴 直接遍历32x32像素，无需手动采样（因为已经是32x32）
    for y in range(H):
        for x in range(W):
            p = img.get_pixel(x, y)
            # 映射到0-127范围（和STM32端完全匹配）
            mapped = int(round(p * 127 / 255))
            mapped = max(0, min(127, mapped))
            data.append(mapped)

    if len(data) != IMAGE_SIZE:
        print(f"长度错: {len(data)}")
        return

    try:
        while uart1.any(): uart1.read()
        time.sleep_ms(10)
        uart1.write(bytes([FRAME_HEAD]))
        uart1.write(bytes(data))
        uart1.write(bytes([FRAME_TAIL_1, FRAME_TAIL_2]))
        time.sleep_ms(20)
        print(f"Sent {img_num_tran}, OK")
    except Exception as e:
        print(f"发失败: {e}")

# ===================== 传感器初始化 =====================
def init_sensor_detection():
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度模式，用于检测
    sensor.set_framesize(sensor.QVGA)      # 320x240完整分辨率
    # 🔴 保留窗口裁剪，但记录原始尺寸，避免缩放错误
    sensor.set_windowing((150,30,50,140))
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.set_saturation(-2)
    sensor.skip_frames(time=2000)

def init_sensor_color():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)    # 彩色模式，用于保存原图
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=1000)

# ===================== 🔴 核心修复：主循环闭环逻辑 =====================
while True :
    # 🔴 【关键】每轮循环开始，彻底重置所有状态变量！
    count = 0
    flag = 1
    i = 0
    image_num = 0
    image_trans_num = 1
    stable_low = 0
    tim.callback(None)  # 强制关闭定时器，避免干扰
    uart3.read()        # 清空串口3缓存，避免残留数据
    servo_rotate(0)     # 舵机归位
    init_sensor_detection()
    clock = time.clock()

    print("\n=== 等待串口3启动信号 ===")
    # 🔴 【核心闭环】严格等待串口3传来一个字符，才执行后续程序
    while True:
        if uart3.read(1):
            print("收到启动信号，开始检测")
            break
        time.sleep_ms(10)  # 避免空转占用CPU

    time.sleep_ms(10)
    uart3.write('u')  # 回复电机运行

    # 检测循环
    while True:
        clock.tick()
        # 🔴 1. 先拍原始灰度图，再做二值化（关键！）
        img_raw = sensor.snapshot()  # 原始灰度图，保留所有细节
        img_bin = img_raw.copy()     # 复制一份做二值化，不影响原图
        img_bin.binary([thresholds[0]])  # 二值化仅用于检测，不用于缩放

        if flag == 1:
            for blob in img_bin.find_blobs([thresholds[1]], pixels_threshold=1, area_threshold=1, merge=5):
                if 5 < blob[5] < 53 and 13 < blob[6] < 157:
                    print(blob)
                    print("停下")
                    uart3.write('s')
                    image_num += 1
                    img_path = f"images/{image_num}.jpg"
                    print(f"开始保存第 {image_num} 张图片：{img_path}")

                    # 舵机动作
                    servo_rotate(-70)
                    servo_rotate(90)
                    servo_rotate(0)
                    # 1. 保存彩色原图
                    init_sensor_color()
                    img_special = sensor.snapshot()
                    img_special.save(img_path)

                    # 🔴 2. 用原始灰度图做缩放（核心修复！）
                    init_sensor_detection()
                    img_raw = sensor.snapshot()

                    scaled_img = image.Image(W, H, sensor.GRAYSCALE)
                    step_x = img_raw.width() // W
                    step_y = img_raw.height() // H

                    img_gray = img_raw.to_grayscale()
                    for y in range(H):
                        for x in range(W):
                            scaled_img.set_pixel(x, y, img_gray.get_pixel(x*step_x, y*step_y))

                    scaled_img.save(f"compressed/{image_num}.pgm")
                    print(f"压缩完成: {image_num}，尺寸{W}x{H}")

                    # 状态重置
                    flag = 0
                    uart3.write('u')
                    print("开始跳")
                    tim.callback(tick)  # 启动定时器
                    break

        # 定时器计数：80次后重置检测
        if count >= 80:
            count = 0
            flag = 1
            print("开始识别")
            tim.callback(None)

        # 光电门检测：触发后退出检测循环，进入传输
        if door_pin.value() == 0:
            stable_low += 1
            if stable_low >= 2:
                uart3.write('s')   # 停止电机
                print("光电门触发，准备传输")
                break
        else:
            stable_low = 0

        print(count)  # 仅调试用，可注释

    # 🔴 【传输完成后闭环】传输完图像，强制回到串口等待
    print("\n=== 开始图像传输 ===")
    tim.callback(None)  # 彻底关闭定时器，避免count残留
    uart3.write('s')   # 停止电机
    time.sleep_ms(100) # 等待电机停止

    # 传输图像
    while image_trans_num <= image_num:
        uart3.write('s')
        time.sleep_ms(10)
        while i<image_num :
            i+=1
            uart3.write('(')
            time.sleep_ms(10)
            if i==image_num :
                time.sleep_ms(10)
                uart3.write(')')
                time.sleep_ms(10)
        print("开始传输")
        tim.callback(None)
        send_frame(image_trans_num)
        time.sleep_ms(2000)
        image_trans_num += 1
        # 发送单张图像
        # send_frame(image_trans_num)
        # time.sleep_ms(2000)
        # image_trans_num += 1

    print("传完")
    print("传输完成！")

    # 🔴 强制清空串口缓存，避免残留数据干扰下一轮
    while uart3.any():
        uart3.read()
    print("\n=== 传输完成，回到等待状态，等待下一次启动信号 ===")
    uart3.write('s')
