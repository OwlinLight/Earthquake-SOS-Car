# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 蜂鸣器引脚定义
buzzer = 8

# 超声波引脚定义
EchoPin = 0
TrigPin = 1

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# 小车按键定义
key = 8

#  舵机引脚定义
ServoPin = 23

#  定义灭火器引脚
OutfirePin = 2

#  RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#光敏电阻引脚定义
LdrSensorLeft = 7
LdrSensorRight = 6

#  循迹红外引脚定义
#  TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


# 引脚初始化
def init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(buzzer, GPIO.OUT, initial=GPIO.HIGH)  # 初始化蜂鸣器
    GPIO.setup(OutfirePin, GPIO.OUT)  # 初始化风扇
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(AvoidSensorLeft, GPIO.IN)  # 初始化左右红外避障
    GPIO.setup(AvoidSensorRight, GPIO.IN)
    GPIO.setup(EchoPin, GPIO.IN)  # 初始化超声波
    GPIO.setup(TrigPin, GPIO.OUT)  # 初始化超声波
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    GPIO.setup(ServoPin, GPIO.OUT)
    GPIO.setup(LdrSensorLeft,GPIO.IN) #初始化光
    GPIO.setup(LdrSensorRight,GPIO.IN)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)
    # 设置舵机的频率和起始占空比
    pwm_servo = GPIO.PWM(ServoPin, 50)
    pwm_servo.start(0)


# 小车前进
def run(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车后退
def back(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车左转
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车右转
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地左转
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车原地右转
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 小车停止
def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


# 按键检测
def key_scan():
    while GPIO.input(key):
        pass
    while not GPIO.input(key):
        time.sleep(0.01)
        if not GPIO.input(key):
            time.sleep(0.01)
            while not GPIO.input(key):
                pass


# 超声波函数
def Distance_test():
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    print('distance is %d ' % (((t2 - t1) * 340 / 2) * 100))
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


# 倒车到右边车库
def right_ruku(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 倒车到左边车库
def left_ruku(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)


# 风扇启动和关闭
def outFire(status):
    if status == 1:
        print(GPIO.input(OutfirePin))
        GPIO.output(OutfirePin, not GPIO.input(OutfirePin))
        time.sleep(4)
    else:
        GPIO.output(OutfirePin, GPIO.input(OutfirePin))

# 舵机旋转到指定角度
def servo_appointed_detection(pos):
    for i in range(18):
        pwm_servo.ChangeDutyCycle(2.5 + 10 * pos / 180)


# 超声波探头向右转90度
# 检测是否有车辆
# 如果没有返回1，否则0
def right_is_empty():
    brake()
    # 舵机旋转到0度，即右侧
    servo_appointed_detection(0)
    time.sleep(1)
    # 测距
    rightdistance = Distance_test()
    if rightdistance > 40:
        return 1
    else:
        return 0


# 超声波探头向左转90度
# 检测是否有车辆
# 如果没有返回1，否则0
def left_is_empty():
    brake()
    # 因为轮胎问题，每一次停车后小车都会略微偏左，所以原地右转进行调整
    spin_right(1, 1)
    time.sleep(0.5)
    brake()
    # 舵机旋转到180度，即右侧
    servo_appointed_detection(180)
    time.sleep(1)
    # 测距
    leftdistance = Distance_test()
    if leftdistance > 40:
        return 1
    else:
        return 0


# 小车鸣笛
def whistle():
    GPIO.output(buzzer, GPIO.LOW)
    # GPIO.output(LED_R, GPIO.HIGH)
    # GPIO.output(LED_G, GPIO.LOW)
    # GPIO.output(LED_B, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(buzzer, GPIO.HIGH)
    # GPIO.output(LED_R, GPIO.LOW)
    # GPIO.output(LED_G, GPIO.LOW)
    # GPIO.output(LED_B, GPIO.LOW)
    time.sleep(0.001)


#  小车鸣笛亮红白灯
def RedAndWhite():
    # GPIO.output(buzzer, GPIO.LOW)
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)
    time.sleep(0.1)
    # GPIO.output(buzzer, GPIO.HIGH)
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.HIGH)
    GPIO.output(LED_B, GPIO.HIGH)
    time.sleep(0.001)


# 延时2s
time.sleep(2)

# try/except语句用来检测try语句块中的错误，
# 从而让except语句捕获异常信息并处理。
try:
    init()
    key_scan()
    while True:
        #  检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
        #  未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
        LdrSersorLeftValue  = GPIO.input(LdrSensorLeft)
        LdrSersorRightValue = GPIO.input(LdrSensorRight)

        TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)
        LeftSensorValue = GPIO.input(AvoidSensorLeft)  # 判断左边有没有车
        RightSensorValue = GPIO.input(AvoidSensorRight)  # 判断右边有没有车
        # 检测到黑线（停车标志）
        # 问题：
        # 解决方法：多贴黑条或小车自主减速（需要摄像头）或代码规定某黑线不够宽的话小车会因为惯性冲过去个位置让小车减速（不太符合实际）
        if TrackSensorLeftValue1 == False and TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and TrackSensorRightValue2 == False:
            # 问题：模拟倒车入库后压线
            # 解决方法：
            # 检测到左边车库没车
            if left_is_empty() == 1:
                # 直接倒车的话角度不好，会压倒线（撞墙），所以先往前开一点
                # 调整倒车位置
                run(10, 10)
                time.sleep(0.8)
                # 往左边倒，右轮启动
                left_ruku(0, 30)
                time.sleep(1)
                back(30, 30)
                time.sleep(0.30)
                brake()
                # 入库后亮绿灯
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.HIGH)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(1)
                break
            # 检测到右边车库没车，先往前开一点调整倒车位置，倒车一定角度再后退，保证小车不压线
            if right_is_empty() == 1:
                # 调整倒车位置
                run(10, 10)
                time.sleep(0.8)
                # 往右边倒，左轮启动
                right_ruku(40, 0)
                time.sleep(0.65)
                back(40, 40)
                time.sleep(0.30)
                brake()
                # 入库后亮绿灯
                GPIO.output(LED_R, GPIO.LOW)
                GPIO.output(LED_G, GPIO.HIGH)
                GPIO.output(LED_B, GPIO.LOW)
                time.sleep(1)
                break
            run(10, 10)
            time.sleep(0.2)
            continue
        # 四路循迹引脚电平状态
        # 0 0 X 0
        # 1 0 X 0
        # 0 1 X 0
        # 以上6种电平状态时小车原地右转
        # 处理右锐角和右直角的转动
        elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
            RedAndWhite()
            spin_right(30, 30)
            time.sleep(0.08)

        # 四路循迹引脚电平状态
        # 0 X 0 0       
        # 0 X 0 1 
        # 0 X 1 0       
        # 处理左锐角和左直角的转动
        elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
            RedAndWhite()
            spin_left(30, 30)
            time.sleep(0.08)
        # 0 X X X
        # 最左边检测到
        elif TrackSensorLeftValue1 == False:
            RedAndWhite()
            spin_left(30, 30)

        # X X X 0
        # 最右边检测到
        elif TrackSensorRightValue2 == False:
            RedAndWhite()
            spin_right(30, 30)

        # 四路循迹引脚电平状态
        # X 0 1 X
        # 处理左小弯
        elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
            RedAndWhite()
            left(0, 30)


        # 四路循迹引脚电平状态
        # X 1 0 X  
        # 处理右小弯
        elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
            RedAndWhite()
            right(30, 0)

        # 四路循迹引脚电平状态
        # X 0 0 X
        # 处理直线
        elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
            RedAndWhite()
            # TODO
            distance1 = Distance_test()
            time.sleep(0.2)
            distance2 = Distance_test()
            distance = distance2-distance1
            preSpeed = distance / 0.2
            if distance2 > 50:
                run(20, 20)
            elif 20 <= distance2 <= 50:
                run(5, 5)
            elif distance < 20:
                if LdrSersorLeftValue == True and LdrSersorRightValue == True:
                    brake()
                    outFire(1)
                    spin_right(50,50)
                    time.sleep(0.5)
                else:
                    brake()
                    time.sleep(0.1)
                    # 舵机旋转到0度，即右侧，测距
                    servo_appointed_detection(0)
                    time.sleep(0.8)
                    rightdistance = Distance_test()

                    # 舵机旋转到180度，即左侧，测距
                    servo_appointed_detection(180)
                    time.sleep(0.8)
                    leftdistance = Distance_test()
                    whistle()
                    if rightdistance < 20 and leftdistance > 20:
                        spin_left(50, 50)
                        time.sleep(0.25)
                        brake() # here is a test
                        # time.sleep(0.01)
                        run(10, 10)
                        time.sleep(1.5)
                        spin_left(50, 50)
                        time.sleep(0.25)
                        run(40, 40)
                        time.sleep(1)
                        spin_left(50, 50)
                        time.sleep(0.25)
                    elif leftdistance > 20 and leftdistance < 20:
                        spin_right(50, 50)
                        time.sleep(0.25)
                        brake() # here is a test
                        # time.sleep(0.01)
                        run(10, 10)
                        time.sleep(1.5)
                        spin_left(50, 50)
                        time.sleep(0.25)
                        run(40, 40)
                        time.sleep(1)
                        spin_right(50, 50)
                        time.sleep(0.25)
                    elif leftdistance > 20 and leftdistance > 20:
                        spin_right(50, 50)
                        time.sleep(0.25)
                        brake() # here is a test
                        # time.sleep(0.01)
                        run(10, 10)
                        time.sleep(1.5)
                        spin_left(50, 50)
                        time.sleep(0.25)
                        run(40, 40)
                        time.sleep(1)
                        spin_right(50, 50)
                        time.sleep(0.25)
                    else:
                        brake()
                        time.sleep(3)


        # 当检测到障碍物时避障（原地180度旋转）
        # 问题：马达动力不能太大，否则会原地转很多圈，难以寻迹
        if RightSensorValue == False and LeftSensorValue == False:
            brake()
            whistle()
            # 原地180度旋转
            spin_right(30, 30)
            time.sleep(0.5)
        # 当为1 1 1 1时小车保持 上一个小车运行状态

except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
