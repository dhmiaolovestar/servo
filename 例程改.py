import time, sys
from machine import Timer,PWM

from math import pi#派
from machine import UART

red_threshold           = (70, 100, 10, 51, -14, 88)#红激光

class Servo:#舵机类
    def __init__(self, pwm, dir=50, duty_min=2.5, duty_max=12.5):
        self.value = dir#频率
        self.pwm = pwm#pwm值
        self.duty_min = duty_min#pwm最小值
        self.duty_max = duty_max#pwm最大值
        self.duty_range = duty_max -duty_min#范围
        self.enable(True)#使能
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)#中间值

    def enable(self, en):#使能或禁用舵机
        if en:#en为True则使能
            self.pwm.enable()
        else:
            self.pwm.disable()

    def dir(self, percentage):#控制舵机转动的百分值
        if percentage > 100:
            percentage = 100
        elif percentage < 0:
            percentage = 0#限幅
        self.pwm.duty(percentage/100*self.duty_range+self.duty_min)#以百分值控制舵机转动

    def drive(self, inc):
        self.value = float(inc)+self.value
        if self.value > 100:
            self.value = 100
        elif self.value < 0:
            self.value = 0
        self.pwm.duty(self.value/100*self.duty_range+self.duty_min)#增量值来实现精确的舵机转动控制

class PID:#PID类
    _kp = _ki = _kd = _integrator = _imax = 0.0
    _last_error = _last_t = 0.0
    _RC = 1/(2 * 3.1415 * 20)#初始化各参数
    def __init__(self, p=0, i=0, d=0, imax=0):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = None#微分项置0

    def get_pid(self, error, scaler):
        tnow = time.ticks_ms()#获取当前时间
        dt = tnow - self._last_t#计算时间间隔
        output = 0
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()#判断间隔时间是否过长，若过长则更新
        self._last_t = tnow#这一次时间作为下一次的上一次时间


        delta_time = float(dt) / float(1000)#时间改变量归一化
        output += error * self._kp#输出加入P项


        if abs(self._kd) > 0 and dt > 0:#判断是否启用微分项
            if self._last_derivative == None:
                derivative = 0
                self._last_derivative = 0#当刚起始时，微分为0
            else:
                derivative = (error - self._last_error) / delta_time#若非起始求微分值
            derivative = self._last_derivative + ((delta_time / (self._RC + delta_time)) * (derivative - self._last_derivative))#二次微分增强稳定性
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative#结果加上D项


        output *= scaler#调节PID效果强度

        if abs(self._ki) > 0 and dt > 0:#判断是否使用积分项
            self._integrator += (error * self._ki) * scaler * delta_time#计算积分项
            if self._integrator < -self._imax: self._integrator = -self._imax
            elif self._integrator > self._imax: self._integrator = self._imax#对积分项进行限幅
            output += self._integrator#结果加上积分项
        return output#返回PID结果

    def reset_I(self):
        self._integrator = 0
        self._last_derivative = None#将PID重启，归零积分项和

class Gimbal:#云台控制类
    def __init__(self, pitch, pid_pitch, roll=None, pid_roll=None, yaw=None, pid_yaw=None):
        self._pitch = pitch#俯仰运动的舵机对象
        self._roll = roll#横滚运动的舵机对象
        self._yaw = yaw#偏航运动的舵机对象
        self._pid_pitch = pid_pitch#俯仰运动的PID参数
        self._pid_roll = pid_roll#横滚运动的PID参数
        self._pid_yaw = pid_yaw#偏航运动的PID参数

    def set_out(self, pitch, roll, yaw=None):
        pass#设置云台输出，没写

    def run(self, pitch_err, roll_err=50, yaw_err=50, pitch_reverse=False, roll_reverse=False, yaw_reverse=False):
        out = self._pid_pitch.get_pid(pitch_err, 1)#得到俯仰运动的PID
        # print("err: {}, out: {}".format(pitch_err, out))
        if pitch_reverse:#确定是否反向输出
            out = - out
        self._pitch.drive(out)#调用drive方法进行输出，让舵机转动

        if self._roll:#判断是否需要横滚运动
            out = self._pid_roll.get_pid(roll_err, 1)#得到横滚运动PID
            if roll_reverse:#确定是否反向输出
                out = - out
            self._roll.drive(out)#调用drive方法进行输出，让舵机转动

        if self._yaw:#判断是否需要偏航运动
            out = self._pid_yaw.get_pid(yaw_err, 1)#得到偏航运动PID
            if yaw_reverse:#确定是否反向输出
                out = - out
            self._yaw.drive(out)#调用drive方法进行输出，让舵机转动


if __name__ == "__main__":#如果是主函数则执行以下主体部分
    '''
        servo:
            freq: 50 (Hz)
            T:    1/50 = 0.02s = 20ms
            duty: [0.5ms, 2.5ms] -> [0.025, 0.125] -> [2.5%, 12.5%]
        pin:
            IO24 <--> pitch
            IO25 <--> roll
    '''
    init_pitch = 80       # init position, value: [0, 100], means minimum angle to maxmum angle of servo
    init_roll = 50        # 50 means middle
    sensor_hmirror = False
    sensor_vflip = False
    lcd_rotation = 2
    lcd_mirror = True
    pitch_pid = [0.23, 0, 0.015, 0]  # P I D I_max
    roll_pid  = [0.23, 0, 0.015, 0]  # P I D I_max
    target_err_range = 10            # target error output range, default [0, 10]
    target_ignore_limit = 0.02       # when target error < target_err_range*target_ignore_limit , set target error to 0
    pitch_reverse = False # reverse out value direction
    roll_reverse = True   # ..

    import sensor,image,lcd
    import KPU as kpu
    class Target():
        def __init__(self, out_range=10, ignore_limit=0.02, hmirror=False, vflip=False, lcd_rotation=2, lcd_mirror=True):
            self.pitch = 0
            self.roll = 0
            self.out_range = out_range
            self.ignore = ignore_limit

            lcd.init()
            lcd.rotation(lcd_rotation)
            lcd.mirror(lcd_mirror)
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QVGA)
            if hmirror:
                sensor.set_hmirror(1)
            if vflip:
                sensor.set_vflip(1)

        def get_target_err(self):
            img = sensor.snapshot()
            code = img.find_blobs([red_threshold],roi=( 80,30,150,150   ))
            if code:
                max_area = 0
                max_i = 0
                for i, j in enumerate(code):
                    a = j.w()*j.h()
                    if a > max_area:
                        max_i = i
                        max_area = a

                img = img.draw_rectangle(code[max_i].rect())
                self.pitch = (code[max_i].y() + code[max_i].h() / 2)/240*self.out_range*2 - self.out_range
                self.roll = (code[max_i].x() + code[max_i].w() / 2)/320*self.out_range*2 - self.out_range
                # limit
                if abs(self.pitch) < self.out_range*self.ignore:
                    self.pitch = 0
                if abs(self.roll) < self.out_range*self.ignore:
                    self.roll = 0
                img = img.draw_cross(160, 120)
                lcd.display(img)
                return (self.pitch, self.roll)
            else:
                img = img.draw_cross(160, 120)
                lcd.display(img)
                return (0, 0)

    target = Target(target_err_range, target_ignore_limit, sensor_hmirror, sensor_vflip, lcd_rotation, lcd_mirror)

    tim0 = Timer(Timer.TIMER2, Timer.CHANNEL0, mode=Timer.MODE_PWM)
    tim1 = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PWM)
    pitch_pwm = PWM(tim0, freq=50, duty=0, pin=24)
    roll_pwm  = PWM(tim1, freq=50, duty=0, pin=25)
    pitch = Servo(pitch_pwm, dir=init_pitch)
    roll = Servo(roll_pwm, dir=init_roll)
    pid_pitch = PID(p=pitch_pid[0], i=pitch_pid[1], d=pitch_pid[2], imax=pitch_pid[3])
    pid_roll = PID(p=roll_pid[0], i=roll_pid[1], d=roll_pid[2], imax=roll_pid[3])
    gimbal = Gimbal(pitch, pid_pitch, roll, pid_roll)

    target_pitch = init_pitch
    target_roll = init_roll
    t = time.ticks_ms()
    _dir = 0
    t0 = time.ticks_ms()
    stdin = UART.repl_uart()
    while 1:
        # get target error
        err_pitch, err_roll = target.get_target_err()
        # interval limit to > 10ms
        if time.ticks_ms() - t0 < 10:
            continue
        t0 = time.ticks_ms()
        # run
        gimbal.run(err_pitch, err_roll, pitch_reverse = pitch_reverse, roll_reverse=roll_reverse)
