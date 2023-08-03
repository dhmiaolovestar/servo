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
tim2 = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PWM)

S_UP = PWM(tim1, freq=50, duty=0, pin=25)
S_DOWN = PWM(tim2, freq=50, duty=0, pin=24)

UART1_T=7
UART1_R=8
UART2_T=9
UART2_R=10

uart_A = UART(UART.UART1, 9600, timeout=1000, read_buf_len=4096)
uart_B = UART(UART.UART2, 9600, timeout=1000, read_buf_len=4096)

# maixduino board_info PIN10/PIN11/PIN12/PIN13 or other hardware IO 10/11/4/3
fm.register(UART1_T, fm.fpioa.UART1_TX, force=True)
fm.register(UART1_R, fm.fpioa.UART1_RX, force=True)
fm.register(UART2_T, fm.fpioa.UART2_TX, force=True)
fm.register(UART2_R, fm.fpioa.UART2_RX, force=True)

thr_light=[(70,100,50,127,-128,127)]
thrb=[(10, 90, 50, -50, -128, -10)]
thrr=[(10, 90, 8, 127, -50, 50)]
thry=[(10, 90, 50, -50, 25, 127)]
thrg=[(10, 90, -128, -10, -6, 127)]
green_threshold         = (0,   80,  -70,   -10,   -0,   30)
yellow_threshold        = (20,  80,  -40,  40,  15,  100)
red_threshold           = (10, 80, 0, 52, 10, 51)

read_strA="60"
read_strB="160"
i=0

angle_up=0.0
angle_down=0.0

def Servo_up(angle):
    angle=-angle
    if angle >90:
       angle=0
    if angle <-90:
       angle=0
    S_UP.duty((angle+90)/180*10+2.5)
    time.sleep(0.01)


def Servo_down(angle):
    if angle >90:
       angle=0
    if angle <-90:
       angle=0
    S_DOWN.duty((angle+90)/180*10+2.5)
    time.sleep(0.01)


def Servo_control( servo , angle ):
    if servo==1:
        Servo_up(angle)
    elif (servo==0):
        Servo_down(angle)


def _control(x , y):
    global angle_up
    global angle_down
    if x < 160 :                       #2.3最左;0.7最右;1.6中间
        angle_down = angle_down + 0.0625*math.fabs(x-160)#左移
    elif (160 < x < 320) :
        angle_down=angle_down-0.0625*math.fabs(x-160)#右移
    if y < 60 :                      #1.5正向上;0.4最下;0.7正向前
        angle_up=angle_up-0.166*math.fabs(y-60)    #下移
    elif (60 < y < 120) :
        angle_up=angle_up+0.166*math.fabs(y-60)    #上移



def err_normalization( limit:float , err:float ):
    return (err-limit)/limit



def transmit_angle( err:float ):
    return err*90



#位置式PID系统
class PositionalPID:
    def __init__(self, P: float, I: float, D: float):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput = 0.0  # PID控制器输出
        self.SystemOutput = 0.0  # 系统输出值
        self.LastSystemOutput = 0.0  # 系统的上一次输出

        self.PIDErrAdd = 0.0
        self.ResultValueBack = 0.0
        self.Error = 0.0
        self.LastError = 0.0

    def SetStepSignal(self, StepSignal):
        self.Error = StepSignal - self.SystemOutput

        if -0.1<self.Error<0.1:
            self.Error=0


        KpWork  = self.Kp *self.Error
        KiWork = self.Ki* self.PIDErrAdd
        if KiWork<-10:
           Kiwork=0.0
        if KiWork > 10:
           Kiwork=0.0
        KdWork = self.Kd * (self.Error- self.LastError)


        self.PIDOutput = KpWork + KiWork + KdWork
        self.PIDErrAdd += self.Error
        self.LastError = self.Error

    def control(self, servo , angle):
        angle=transmit_angle( angle )
        Servo_control( servo , angle )

        # 以一阶惯性环节为例子演示控制效果

    def SetInertiaTime(self, IntertiaTime, SampleTime):
        self.SystemOutput = (IntertiaTime * self.LastSystemOutput + SampleTime * self.PIDOutput) / (SampleTime + IntertiaTime)
        self.LastSystemOutput = self.SystemOutput

#增量式PID系统
class IncrementalPID:
    def __init__(self, P:float ,I:float ,D:float ):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.PIDOutput =0.0         #PID控制器输出
        self.SystemOutput = 0.0     #系统输出值
        self.LastSystemOutput = 0.0 #系统的上一次输出

        self.Error = 0.0
        self.LastError = 0.0
        self.LastLastError = 0.0

    #设置PID控制器参数
    def SetStepSignal(self,StepSignal):
        self.Error = StepSignal - self.SystemOutput
        #计算增量
        IncrementalValue = self.Kp*(self.Error - self.LastError)\
            + self.Ki * self.Error +self.Kd *(self.Error -2*self.LastError +self.LastLastError)
        #计算输出
        self.PIDOutput += IncrementalValue
        self.LastLastError = self.LastError
        self.LastError = self.Error

    #以一阶惯性环节为例子演示控制效果
    def SetInertiaTime(self,IntertiaTime,SampleTime):
        self.SystemOutput = (IntertiaTime*self.LastSystemOutput + SampleTime *self.PIDOutput)/(SampleTime + IntertiaTime)
        self.LastSystemOutput = self.SystemOutput




pid_up=PositionalPID( 2 , 0.01 , 0.01)
pid_down=PositionalPID( 2 , 0.01 , 0.01 )


Servo_up(1)
Servo_down(0)
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
while True:
    img = sensor.snapshot()
    #img=摄像头获取的图像对象
    blobs = img.find_blobs(thr_light,merge=True)

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

            pid_up.SetStepSignal( err_normalization( 320 , b[5] ) )
            pid_up.control( 1 , pid_up.PIDOutput )

            pid_down.SetStepSignal( err_normalization( 120 , b[6] ) )
            pid_down.control( 0 , pid_down.PIDOutput )
            print( pid_up.Error*90 )
            print( pid_down.Error*90 )

    lcd.display(img)

uart_A.deinit()
uart_B.deinit()
del uart_A

