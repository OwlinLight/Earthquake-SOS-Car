# -*- coding:UTF-8 -*-

import RPi.GPIO as GPIO #导入函数

import time
#定义蜂鸣器接口
BEE = 8;

GPIO.setmode(GPIO.BCM) #定义数据口

GPIO.setup(BEE, GPIO.OUT)

try:
    while True:
        for i in range(0, 3):
            GPIO.output(BEE, GPIO.LOW) #根据模块高低电平调整时间及模式
            time.sleep(1)
            GPIO.output(BEE, GPIO.HIGH) #根据模块高低电平调整时间及模式
            time.sleep(1)
        for i in range(0, 3):
            GPIO.output(BEE, GPIO.LOW) #根据模块高低电平调整时间及模式
            time.sleep(0.5)
            GPIO.output(BEE, GPIO.HIGH) #根据模块高低电平调整时间及模式
            time.sleep(0.5)
except:
    print("except")
GPIO.cleanup() #结束进程，释放GPIO引脚