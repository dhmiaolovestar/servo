import time
import machine
from board import board_info
import math
from fpioa_manager import fm
#调用fm这个类,从 fpioa_manager 包导入fm 对象，主要用于引脚和外设的映射
from Maix import GPIO
#调用GPIO这个类,从包 Maix 导入了 GPIO 这个类， GPIO 外设相关操作
from machine import UART,Timer,PWM
i=0
tim1 = Timer(Timer.TIMER0, Timer.CHANNEL2, mode=Timer.MODE_PWM)
tim2 = Timer(Timer.TIMER1, Timer.CHANNEL0, mode=Timer.MODE_PWM)
S_UP = PWM(tim1, freq=50, duty=0, pin=25)
S_DOWN = PWM(tim2, freq=50, duty=0, pin=24)

def move(start_y,start_x,end_y,end_x,slow_speed):#平滑运动
    i=0.0#循环参数
    dx=end_x-start_x
    dy=end_y-start_y#x,y改变量
    x_change=dx/slow_speed
    y_change=dy/slow_speed#达到目标时每次微小的改变量
    x_now=start_x
    y_now=start_y#指示每次微小改变后的位置
    abs_dx=math.fabs(dx)
    abs_dy=math.fabs(dy)
    abs_x_change=math.fabs(x_change)
    abs_y_change=math.fabs(y_change)#改变量与微小改变量的绝对值，用于记录是否达到目标
    t=float(abs_x_change*200000+abs_y_change*200000)#指示每个微小改变量达到所需时间，倍率可作微调
    while(abs_dx>0 or abs_dy>0):
        x_now=x_now+x_change
        y_now=y_now+y_change
        while(i<t):
            S_UP.duty(y_now)
            S_DOWN.duty(x_now)
            i=i+1
        i=0.0
        abs_dx=abs_dx-abs_x_change
        abs_dy=abs_dy-abs_y_change

while True:

'''
move(5.63,7.00,4.88,7.00,20)#中间到正上
move(4.88,7.00,4.93,6.80,40)#正上到左上

move(4.93,6.80,4.94,6.52,20)#左上到右上
move(4.94,6.52,4.95,6.25,20)

move(4.95,6.25,5.35,6.25,20)#右上到右下
move(5.35,6.25,5.75,6.25,20)

move(5.75,6.25,5.75,6.82,20)#右下到左下

move(5.75,6.82,5.35,6.82,20)#左下到左上
move(5.35,6.82,4.93,6.82,20)


move(4.93,6.80,4.88,7.00,20)#左上到正上
move(4.88,7.00,5.63,7.00,20)#正上到中间
'''
'''
while(i<100000):#右上
    S_UP.duty(4.95)
    S_DOWN.duty(6.25)
    i=i+1
i=0
while(i<100000):#长方形右下
    S_UP.duty(5.75)
    S_DOWN.duty(6.25)
    i=i+1
i=0
while(i<100000):#长方形左下
    S_UP.duty(5.75)
    S_DOWN.duty(6.83)
    i=i+1
i=0
while(i<100000):#长方形左上
    S_UP.duty(4.9)
    S_DOWN.duty(6.8)
    i=i+1
i=0
'''
'''#正方形运动
    move(5.63,7.0,4.88,7.0,20)
    move(4.88,7.0,4.95,6.25,20)
    move(4.9,6.2,6.4,6.2,20)
    move(6.4,6.2,6.4,7.82,20)
    move(6.4,7.82,4.9,7.82,20)
    move(4.9,7.82,4.88,7.0,20)
    move(4.88,7.0,5.63,7.0,20)
'''
'''
    while(i<100000):#中心
        S_UP.duty(5.63)
        S_DOWN.duty(7)
        i=i+1
    i=0
    while(i<100000):#中上
        S_UP.duty(4.88)
        S_DOWN.duty(7)
        i=i+1
    i=0
    while(i<100000):#右上
        S_UP.duty(4.9)
        S_DOWN.duty(6.2)
        i=i+1
    i=0
    while(i<100000):#右下
        S_UP.duty(6.4)
        S_DOWN.duty(6.2)
        i=i+1
    i=0
    while(i<100000):#左下
        S_UP.duty(6.4)
        S_DOWN.duty(7.82)
        i=i+1
    i=0
    while(i<100000):#左上
        S_UP.duty(4.9)
        S_DOWN.duty(7.82)
        i=i+1
    i=0
    while(i<100000):#中上
        S_UP.duty(4.88)
        S_DOWN.duty(7)
        i=i+1
    i=0
'''
    #move(5.55,8.55,4.8,8.55,10)
    #move(4.8,8.55,5.55,8.55,10)
'''
    while(i<100000):
        S_UP.duty(5.55)
        S_DOWN.duty(8.55)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(4.8)
        S_DOWN.duty(8.55)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(4.85)
        S_DOWN.duty(7.7)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(6.3)
        S_DOWN.duty(7.7)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(6.3)
        S_DOWN.duty(9.35)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(4.8)
        S_DOWN.duty(9.35)
        i=i+1
    i=0
    while(i<100000):
        S_UP.duty(4.8)
        S_DOWN.duty(8.55)
        i=i+1
    i=0

'''
'''
S_UP.duty(3.45)
S_DOWN.duty(5.95)#正中心

S_UP.duty(3.45)
S_DOWN.duty(5.2)#正中心上点

S_UP.duty(2.72)
S_DOWN.duty(5.21)#右上点

S_UP.duty(2.72)
S_DOWN.duty(6.8)#右下点

S_UP.duty(4.2)
S_DOWN.duty(6.8)#左下点

S_UP.duty(4.2)
S_DOWN.duty(5.21)


ch0.freq(2000000)
print("freq:",ch0.freq())
ch0.duty(60)
time.sleep(3)
time.sleep(3)
ch0.disable()
2.5--12.5(0--180)
'''
