import sensor#摄像头
import image#图像传感器
import lcd#屏幕
import time
import math
from fpioa_manager import fm
#调用fm这个类,从 fpioa_manager 包导入fm 对象，主要用于引脚和外设的映射
from Maix import GPIO
#调用GPIO这个类,从包 Maix 导入了 GPIO 这个类， GPIO 外设相关操作
from machine import UART,Timer,PWM
#调用UART类uart 模块主要用于驱动开发板上的异步串口，发送方发出数据后，不等接收方发回响应，接着发送下个数据包的通讯方式。
#PWM通过定时器配置，接到IO17引脚
tim1 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
tim2 = Timer(Timer.TIMER1, Timer.CHANNEL3, mode=Timer.MODE_PWM)

S_UP = PWM(tim1, freq=50, duty=0, pin=24)
S_DOWN = PWM(tim2, freq=50, duty=0, pin=25)

#UART1_T=7
#UART1_R=8
#UART2_T=9
#UART2_R=10

#uart_A = UART(UART.UART1, 9600, timeout=1000, read_buf_len=4096)
#uart_B = UART(UART.UART2, 9600, timeout=1000, read_buf_len=4096)

## maixduino board_info PIN10/PIN11/PIN12/PIN13 or other hardware IO 10/11/4/3
#fm.register(UART1_T, fm.fpioa.UART1_TX, force=True)
#fm.register(UART1_R, fm.fpioa.UART1_RX, force=True)
#fm.register(UART2_T, fm.fpioa.UART2_TX, force=True)
#fm.register(UART2_R, fm.fpioa.UART2_RX, force=True)


#GPIO
fm.register(7,fm.fpioa.GPIO1)
fm.register(8,fm.fpioa.GPIO2)

Reset_sys=GPIO(GPIO.GPIO1,GPIO.IN,GPIO.PULL_DOWN)
Reset_sig=Reset_sys.value()
Moving_Pouse_sys=GPIO(GPIO.GPIO2,GPIO.IN,GPIO.PULL_DOWN)
Moving_Pouse_sig=Moving_Pouse_sys.value()


#red (10, 80, 0, 52, 10, 51)
#(0,   80,  -70,   -10,   -0,   30)
green_threshold         = (0,   80,  -70,   -10,   -0,   30)
yellow_threshold        = (20,  80,  -40,  40,  15,  100)
red_threshold           = (70, 100, 10, 51, -14, 88)
redd2=(48,69,6,37,1,20)
read_strA="60"
read_strB="160"

angle_up=0.0
angle_down=0.0

mode=True
#待调试
CentralAngle_up=0
CentralAngle_down=0
#复位
def reset_scan():
    Reset_sig=Reset_sys.value()
    if Reset_sig ==1:
        Servo_up(CentralAngle_up)
        Servo_down(CentralAngle_down)
        mode=True
#运动目标控制
def Moving_target_control():
    Moving_Pouse_sig=Moving_Pouse_sys.value()
    if Moving_Pouse_sig ==1:
        mode=False

def err_normalization( limit:float , err:float ):
    if limit==120:
        return (err-limit)/(limit) #[-1,1]
    elif limit==180:
        return (err-limit)/(limit-60) #[-1,1]

def transmit_angle( err:float ):
    return err*90

def Servo_up(angle):

    if angle >90:
       angle=0
    if angle <-90:
       angle=0
    S_UP.duty( angle/90*5+7.5 )

    time.sleep(0.001)
def Servo_down(angle):

    if angle >90:
       angle=0
    if angle <-90:
       angle=0
    S_DOWN.duty( angle/90*5+7.5 )

    time.sleep(0.001)

def Servo_control( servo , angle ):
    angle=transmit_angle(angle)
    if servo==1:
        Servo_up(angle)
    elif (servo==0):
        Servo_down(angle)

def _control(x , y):
    global angle_up
    global angle_down
    if x < 160 :                       #2.3最左;0.7最右;1.6中间
        angle_down=angle_down+0.0625*math.fabs(x-160)#左移
    elif (160 < x < 320) :
        angle_down=angle_down-0.0625*math.fabs(x-160)#右移
    if y < 60 :                      #1.5正向上;0.4最下;0.7正向前
        angle_up=angle_up-0.166*math.fabs(y-60)    #下移
    elif (60 < y < 120) :
        angle_up=angle_up+0.166*math.fabs(y-60)    #上移
#位置式
class Pid():
    """这里定义了一个关于PID的类"""
    def __init__(self, exp_val, kp, ki, kd):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.now_val = 0
        self.sum_err = 0
        self.now_err = 0
        self.last_err = 0

    def cmd_pid(self):
        self.last_err = self.now_err
        self.now_err = self.exp_val - self.now_val


        if -8<self.now_err<8:
            self.now_err=0
        if self.now_err==0:
            self.sum_err=0
        self.sum_err += self.now_err
        # 这一块是严格按照公式来写的
        Pout=self.KP * (self.exp_val - self.now_val)
        Iout=self.KI * self.sum_err
        Dout=self.KD * (self.now_err - self.last_err)
        if Iout>0.3:
            Iout=0.3
        elif (Iout<-0.3):
            Iout=-0.3


        self.now_val = Pout + Iout + Dout
        return self.now_val






#Servo_up(0)
#Servo_down(0)

#UP     2.5-6.5-10.5
#   (占空比)-->[-1,1]*4+6.5
#2.5-上75
#6.5-正前
#10.5-下75

#DOWN   4.5-8.5-12.5
#   (占空比)-->[-1,1]*4+8.5
#4.5-右75
#8.5-正前
#12.5-左75
S_UP.duty(6.5)
S_DOWN.duty(8.5)

pid_up=  Pid( 120,0.28 , 0.0 , 0.0 )
pid_down=Pid( 160,0.28 , 0.0 , 0.0 )


lcd.init()
sensor.reset()
#初始化单目摄像头
sensor.set_pixformat(sensor.RGB565)
#设置帧格式：MaixPy开发板配置的屏幕使用的是RGB565，推荐设置为RGB565格式
sensor.set_framesize(sensor.QVGA)
#设置帧大小：MaixPy开发板配置的屏幕是320*240分辨率，推荐设置为QVGA格式
sensor.run(1)
#摄像头，启动!
#初始化
while (True):
    #reset_scan()
    while mode:
        #Moving_target_control()#
        img = sensor.snapshot()
        #img=摄像头获取的图像对象
        blobs = img.find_blobs([redd2],roi=( 80,30,150,150   ),x_stride=30,y_stride=30 )#,x_stride=100,y_stride=100)
        img.draw_rectangle( 80,30,150,150   )
        if blobs:   #如果找到了目标颜色
            for b in blobs:#迭代找到的目标颜色区域
                tmp=img.draw_rectangle(b[0:4])
                tmp=img.draw_cross(b[5], b[6])
                c=img.get_pixel(b[5], b[6])
                zz=str("%03d"%b.cx())+str("%03d"%b.cy())+'E'


                print(zz)
                #uart_B.write(zz.encode("utf-8","strict"))#write#用于使用串口发送数据,发送x坐标
                #read_dataB = uart_B.read()#read#用于读取串口缓冲中的数据
                #dataB = read_dataB
                #print(dataB)



                #误差控制
                #_control(b[5],b[6])
                #Servo_up(angle_up)
                #Servo_down(angle_down)

                #pid控制
                pid_up.now_val=b[5]#err_normalization( 120 , b[5] )#输入归一化坐标
                up=pid_up.cmd_pid( )
                Servo_control( 1 , up )#在内部进行角度扩展

                pid_down.now_val=b[6]#err_normalization( 180 , b[6] )
                down=pid_down.cmd_pid( )
                Servo_control( 0 , down )

                print( up )
                print( down )
                print( pid_up.now_val )
                print( pid_down.now_val )


        lcd.display(img)

uart_A.deinit()
uart_B.deinit()
del uart_A

