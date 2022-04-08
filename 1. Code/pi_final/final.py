# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import cv2
import threading
import time

exitFlag = 0


#RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

#定义蜂鸣器接口
BEE = 8;

#设置RGB三色灯为BCM编码方式
GPIO.setmode(GPIO.BCM)

#RGB三色灯设置为输出模式
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)


# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 小车按键定义
key = 8

# 循迹红外引脚定义
# TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1 = 3  # 定义左边第一个循迹红外传感器引脚为3口
TrackSensorLeftPin2 = 5  # 定义左边第二个循迹红外传感器引脚为5口
TrackSensorRightPin1 = 4  # 定义右边第一个循迹红外传感器引脚为4口
TrackSensorRightPin2 = 18  # 定义右边第二个循迹红外传感器引脚为18口
# 超声波引脚定义
EchoPin = 0
TrigPin = 1

enFRONTSERVOLEFT = 1
enFRONTSERVORIGHT = 2
enSERVOUP = 3
enSERVODOWN = 4
enSERVOUPDOWNINIT = 5
enSERVOLEFT = 6
enSERVORIGHT = 7
enSERVOSTOP = 8

FrontServoPin = 23
ServoUpDownPin = 9
ServoLeftRightPin = 11

GPIO.setup(ServoUpDownPin, GPIO.OUT)
GPIO.setup(ServoLeftRightPin, GPIO.OUT)


pwm_UpDownServo = GPIO.PWM(ServoUpDownPin, 50)
pwm_LeftRightServo = GPIO.PWM(ServoLeftRightPin, 50)
pwm_UpDownServo.start(0)
pwm_LeftRightServo.start(0)



# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


# 电机引脚初始化为输出模式
# 按键引脚初始化为输入模式
# 寻迹引脚初始化为输入模式
def init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(key, GPIO.IN)
    GPIO.setup(BEE, GPIO.OUT)
    GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
    GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
    GPIO.setup(TrackSensorRightPin1, GPIO.IN)
    GPIO.setup(TrackSensorRightPin2, GPIO.IN)
    GPIO.setup(EchoPin, GPIO.IN)
    GPIO.setup(TrigPin, GPIO.OUT)
    # 设置pwm引脚和频率为2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

def red(t):
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(t)

def dim(t):
        GPIO.output(LED_R, GPIO.LOW)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        time.sleep(t)

def sos_light():
    for i in range(0, 3):
        red(1)
        dim(0.5)
    for i in range(0, 3):
        red(2)
        dim(0.5)

def sos_bee():
    for i in range(0, 3):
        GPIO.output(BEE, GPIO.HIGH)  # 根据模块高低电平调整时间及模式
        time.sleep(1)
        GPIO.output(BEE, GPIO.LOW)  # 如是低电平触发，结果为每相隔3秒蜂鸣器鸣叫0.1秒
        time.sleep(0.5)
    for i in range(0, 3):
        GPIO.output(BEE, GPIO.HIGH)  # 根据模块高低电平调整时间及模式
        time.sleep(2)
        GPIO.output(BEE, GPIO.LOW)  # 如是低电平触发，结果为每相隔3秒蜂鸣器鸣叫0.1秒
        time.sleep(0.5)


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
    #print("distance is %d " % (((t2 - t1) * 340 / 2) * 100))
    time.sleep(0.01)
    return ((t2 - t1) * 340 / 2) * 100


#多线程函数
class myThread (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        #print ("开始线程：" + self.name)
        if(self.name == "light"):
            light(self.name, self.counter, 100)
        elif(self.name == "buzzer"):
            buzzer(self.name, self.counter, 100)
        elif(self.name == "cam"):
            cam(self.name, self.counter, 100)
        #print ("退出线程：" + self.name)

def print_time(threadName, delay, counter):
    while counter:
        if exitFlag:
            threadName.exit()
        time.sleep(delay)
        #print ("%s: %s" % (threadName, time.ctime(time.time())))
        counter -= 1

def light(threadName, delay, counter):
    while True:
        if exitFlag:
            break
        #print ("light: ...---...")
        sos_light()
        time.sleep(delay)

def buzzer(threadName, delay, counter):
    while True:
        if exitFlag:
            break
        #print("bee: ...---...")
        sos_bee()
        time.sleep(delay)


# 图片识别方法封装
def discern(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cap = cv2.CascadeClassifier(
        "/home/pi/SmartCar/code/haarcascade_frontalface_default.xml"
    )
    faceRects = cap.detectMultiScale(
        gray, scaleFactor=1.2, minNeighbors=3, minSize=(50, 50))
    if len(faceRects):
        return 1
    else:
        return 0
        # for faceRect in faceRects:
        #    x, y, w, h = faceRect
        #    cv2.rectangle(img, (x, y), (x + h, y + w), (0, 255, 0), 2)  # 框出人脸

    # cv2.imshow("Image", img)


def cam(threadName, delay, counter):
    global exitFlag
    cap = cv2.VideoCapture("http://192.168.50.1:8080/?action=stream")
    count = 0
    while True:
        if exitFlag:
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        counter-=1
        ret, img = cap.read()
        count += discern(img)
        print count
        if count == 4:
            cap.release()  # 释放摄像头
            cv2.destroyAllWindows()  # 释放窗口资源
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.LOW)
            exitFlag = 1
            break
        #print("detecting")
        count += 1
        time.sleep(delay)

def leftrightservo_appointed_detection(pos):
    for i in range(1):
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        pwm_LeftRightServo.ChangeDutyCycle(0) #归零信号


# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    for i in range(1):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        pwm_UpDownServo.ChangeDutyCycle(0)    #归零信号



# 延时2s
time.sleep(2)

# try/except语句用来检测try语句块中的错误，
# 从而让except语句捕获异常信息并处理。
try:
    init()
    key_scan()
    while True:
        # 检测到黑线时循迹模块相应的指示灯亮，端口电平为LOW
        # 未检测到黑线时循迹模块相应的指示灯灭，端口电平为HIGH
        distance = Distance_test()
        TrackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
        TrackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)


        # 当为0 0 0 0时小车保持上一个小车运行状态或者或者进入下一模块
        if TrackSensorLeftValue1 == False and TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and TrackSensorRightValue2 == False:
            run(5, 5)
            time.sleep(0.5)
            if TrackSensorLeftValue1 == False and TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False and TrackSensorRightValue2 == False:
                brake()
                leftrightservo_appointed_detection(30)
                updownservo_appointed_detection(105)
                thread1 = myThread(1, "light", 1)
                thread2 = myThread(2, "buzzer", 2)
                thread3 = myThread(3, "cam", 3)
                try:
                    thread1.start()
                    thread2.start()
                    thread3.start()
                    thread1.join()
                    thread2.join()
                    thread3.join()
                except:
                    pass
                #掉头180度
                spin_left(20, 20)
                time.sleep(0.08)
        # 四路循迹引脚电平状态
        # 0 0 X 0
        # 1 0 X 0
        # 0 1 X 0
        # 以上6种电平状态时小车原地右转
        # 处理右锐角和右直角的转动
        elif (TrackSensorLeftValue1 == False or TrackSensorLeftValue2 == False) and TrackSensorRightValue2 == False:
            spin_right(20, 20)
            time.sleep(0.08)


        # 四路循迹引脚电平状态
        # 0 X 0 0
        # 0 X 0 1
        # 0 X 1 0
        # 处理左锐角和左直角的转动
        elif TrackSensorLeftValue1 == False and (TrackSensorRightValue1 == False or TrackSensorRightValue2 == False):
            spin_left(20, 20)
            time.sleep(0.08)

        # 0 X X X
        # 最左边检测到
        elif TrackSensorLeftValue1 == False:
            spin_left(20, 20)

        # X X X 0
        # 最右边检测到
        elif TrackSensorRightValue2 == False:
            spin_right(20, 20)

        # 四路循迹引脚电平状态
        # X 0 1 X
        # 处理左小弯
        elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == True:
            left(0, 60)

        # 四路循迹引脚电平状态
        # X 1 0 X
        # 处理右小弯
        elif TrackSensorLeftValue2 == True and TrackSensorRightValue1 == False:
            right(60, 0)

        # 四路循迹引脚电平状态
        # X 0 0 X
        # 处理直线
        elif TrackSensorLeftValue2 == False and TrackSensorRightValue1 == False:
            run(20, 20)
        


except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()

