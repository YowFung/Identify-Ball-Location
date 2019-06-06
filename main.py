"""
项目：2017年全国电子设计竞赛·OpenMV图像定位及解算程序·主程序
版权：韶关学院参赛队·第10组
作者：张惠烽
日期：2017/8/11
版本：2.2

说明：程序正常运行是蓝色LED灯点亮，找不到目标小球时蓝色LED闪烁
"""


#引入文件和模块
import time, image, math, pyb, sensor
from pyb import UART, LED
#from control iimport *


#系统配置方案类
class Config_System(object):
    #图像相关配置
    PixFormat  = sensor.GRAYSCALE                   #色彩
    FrameSize  = sensor.VGA                         #图像大小
    WindowSize = [480, 480]                         #视图大小（应小于图像大小）
    SkipFrames = 30                                 #初始化时跳过的帧数
    Resolution = 480/65                             #分辨率(pix/cm)
    Gain       = False                              #开启自动增益
    Whitebal   = False                              #开启自动白平衡

    #串口通信相关配置
    BaudRate = 115200                               #波特率

    #小球物体信息配置
    BallColor = [5, 30, -7, 7, -6, 6]               #小球的颜色阈值
    BallBlobRedius = 1.5                            #小球色块的半径(cm)
    BallObjRedius  = [0.75, 1.8]                    #小球的半径范围(cm)


#滤波配置方案类
class Config_Filter(object):
    Area     = [0, 0]                               #面积范围[min, max](pix)
    Ratio    = [0, 0]                               #长宽比例参考标准[low, height](pix)
    Distance = 150                                  #距离偏差限制(pix)
    Margin   = [0, 0, 0, 0]                         #边缘距离[top, right, bottom, left](pix)



#全局变量
no_ball_found = 0                                   #记录连续未找到小球的次数，超过5次将作警告
position   = [-1, -1]                               #记录小球位置信息
conf_sys   = Config_System()                        #系统配置方案
conf_filt = Config_Filter()                         #滤波配置方案


#LED
led_red  = LED(1)                                   #红色LED
led_blue = LED(3)                                   #蓝色LED
led_red.on()                                        #点亮红色LED


#串口通信
uart = UART(3, conf_sys.BaudRate)
uart.init(conf_sys.BaudRate, bits = 8, parity = None, stop = 1)


#初始化sensor
def init_sensor():
    global conf_sys
    sensor.reset()
    sensor.set_pixformat(conf_sys.PixFormat)            #颜色格式
    sensor.set_framesize(conf_sys.FrameSize)            #图像像素
    sensor.set_auto_gain(conf_sys.Gain)                 #自动增益
    sensor.set_auto_whitebal(conf_sys.Whitebal)         #自动白平衡
    sensor.set_windowing(conf_sys.WindowSize)           #视图尺寸
    sensor.skip_frames(conf_sys.SkipFrames)             #跳过帧数


#初始化滤波配置方案
def init_filter(area, ratio, distance, margin):
    global conf_filt

    if type(area) != type(None):
        conf_filt.Area = area
    if type(ratio) != type(None):
        conf_filt.Ratio = ratio
    if type(distance) != type(None):
        conf_filt.Distance = distance
    if type(margin) != type(None):
        conf_filt.Margin = margin


#获取小球的坐标位置
def get_ball_info():
    global conf_sys, conf_filt, no_ball_found, position, led_red

    #获取现场图像
    img = sensor.snapshot()
    #img.lens_corr(1.8)                             #矫正畸形

    #初始化严格滤波配置方案
    Area = [get_area(cm2pix(conf_sys.BallObjRedius[0])), get_area(cm2pix(conf_sys.BallObjRedius[1]))]
    Ratio = [0.4, 0.7]
    Distance = cm2pix(15)
    Margin = [cm2pix(4), cm2pix(4), cm2pix(4), cm2pix(4)]
    init_filter(Area, Ratio, Distance, Margin)

    #寻找色块并过滤筛选
    blobs = img.find_blobs([conf_sys.BallColor], margin = 2)
    found_blob = ball_filter(blobs)

    #如果找不到目标小球，则降低滤波要求再寻找一次目标
    if type(found_blob) is type(None):
        Ratio = [0.3, 0.5]
        Distance = cm2pix(40)
        Margin = [cm2pix(2.5), cm2pix(2.5), cm2pix(2.5), cm2pix(2.5)]
        init_filter(Area, Ratio, Distance, Margin)
        found_blob = ball_filter(blobs)

    #如果还是找不到目标小球，可以考虑使用补偿滤波算法，但可能会导致不可预见的偏差

    #判断是否找到目标小球
    if type(found_blob) is type(None):
        #如果未找到小球
        no_ball_found += 1
        if no_ball_found > 5:
            led_red.on()                            #点亮红灯以警告
    else:
        #如果找到了小球
        no_ball_found = 0
        led_red.off()                               #灭掉红灯

        #画图标出小球位置
        img.draw_circle(position[0], position[1], cm2pix(conf_sys.BallBlobRedius))
        img.draw_cross(position[0], position[1])

        #写出小球坐标位置
        string = '(%d, %d)' % tuple(position)
        img.draw_string(position[0] + cm2pix(conf_sys.BallBlobRedius), position[1], string)


#小球色块滤波算法
def ball_filter(blobs):
    global position, conf_filt, conf_sys
    temp_blobs1 = []                                #面积滤波过后的结果
    temp_blobs2 = []                                #边缘滤波过后的结果
    temp_blobs3 = []                                #形状滤波过后的结果
    found_blob  = None                              #最终结果

    #面积滤波
    for blob in blobs:
        if blob.area() > conf_filt.Area[0] and blob.area() < conf_filt.Area[1]:
            temp_blobs1.append(blob)

    #形状滤波
    for blob in temp_blobs1:
        if blob.w() == blob.h():
            temp_blobs2.append(blob)
        else:
            if blob.w() < blob.h():
                temp_lw_ratio = blob.w()/blob.h()
            else:
                temp_lw_ratio = blob.h()/blob.w()

            if not len(temp_blobs2):
                ratio = conf_filt.Ratio[0]
            else:
                ratio = conf_filt.Ratio[1]
            if temp_lw_ratio > ratio:
                temp_blobs2.append(blob)

    #边缘滤波
        min_x = conf_filt.Margin[3]
        min_y = conf_filt.Margin[0]
        max_x = conf_sys.WindowSize[0] - conf_filt.Margin[1]
        max_y = conf_sys.WindowSize[1] - conf_filt.Margin[2]
    for blob in temp_blobs2:
        if blob.cx() > min_x and blob.cx() < max_x and blob.cy() > min_y and blob.cy() < max_y:
            temp_blobs3.append(blob)

    #相对位置滤波，确定最佳目标
    if position != [-1, -1]:
        #如果有上一次的位置记录，则选择距离上一次最近的那个
        limit_distance = conf_filt.Distance
        min_distance = 9999
        for blob in temp_blobs3:
            temp_distance = get_distance((blob.cx(), blob.cy()), position)
            if temp_distance < min_distance and temp_distance < limit_distance:
                distance = temp_distance
                found_blob = blob
    else:
        #如果没有上一次的位置记录，则选择长宽比例最接近正方形的那个
        min_deviation = 1
        for blob in temp_blobs3:
            if blob.w() < blob.h():
                temp_deviation = blob.w()/blob.h()
            else:
                temp_deviation = blob.h()/blob.w()

            if 1-temp_deviation < min_deviation:
                min_deviation = temp_deviation
                found_blob = blob

    #更新小球的位置信息
    if type(found_blob) != type(None):
        position = [found_blob.cx(), found_blob.cy()]

    #返回筛选后的最终的小球色块
    return found_blob


#弥补滤波算法
def ball_filter_comp(blobs):
    #当普通滤波后无法找到目标小球时，可选用弥补滤波方法
    global position, conf_filt
    found_blob = None

    #总是获取最接近上一次位置的目标
    min_distance = conf_filt.Distance
    for blob in blobs:
        now_distance = get_distance((blob.cx(), blob.cy()), position)
        if now_distance < min_distance:
            min_distance = now_distance
            found_blob = blob

    #更新小球的位置信息
    if type(found_blob) != type(None):
        position = [found_blob.cx(), found_blob.cy()]

    #返回筛选后的最终的小球色块
    return found_blob


#厘米转像素
def cm2pix(cemtimeter):
    global conf_sys
    result = int(conf_sys.Resolution*cemtimeter)
    return result


#计算圆的面积
def get_area(redius):
    area = math.pi*math.pow(redius, 2)
    return int(area)


#计算两点间的距离
def get_distance(point1, point2):
    distance = math.sqrt(math.pow(point1[0]-point2[0], 2) + math.pow(point1[1]-point2[1], 2))
    return int(distance)


 #串口发送小球坐标
def send_data():
    global uart, position
    x = position[0]
    y = position[1]
    uart_buf = bytearray([0xef, int(1), int(x)>>8, int(x), int(y)>>8, int(y), 0xfe])
    uart.write(uart_buf)


#初始化程序
init_sensor()
clock = time.clock()
led_red.off()
pyb.delay(500)
led_blue.on()


#程序大循环体
while(True):
    clock.tick()
    get_ball_info()
    send_data()
    print(position)
