# -*- coding:UTF-8 -*-

import RPi.GPIO as GPIO #导入函数

import time
#定义蜂鸣器接口
BEE = 8;

GPIO.setmode(GPIO.BOARD) #定义数据口

GPIO.setup(BEE, GPIO.OUT)

try:
    while True:
            #GPIO.output(BEE, GPIO.HIGH) #根据模块高低电平调整时间及模式
            #time.sleep(2)
            GPIO.output(BEE, GPIO.LOW) #如是低电平触发，结果为每相隔3秒蜂鸣器鸣叫0.1秒
            time.sleep(1)
except:
    print("except")
GPIO.cleanup() #结束进程，释放GPIO引脚
