# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
#小车舵机定义
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



# 摄像头舵机左右旋转到指定角度
def leftrightservo_appointed_detection(pos):
    for i in range(1):
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
    # pwm_LeftRightServo.ChangeDutyCycle(0) #归零信号


# 摄像头舵机上下旋转到指定角度
def updownservo_appointed_detection(pos):
    for i in range(1):
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        time.sleep(0.02)  # 等待20ms周期结束
        # pwm_UpDownServo.ChangeDutyCycle(0)    #归零信号

if __name__ == "__main__":
    leftrightservo_appointed_detection(30)