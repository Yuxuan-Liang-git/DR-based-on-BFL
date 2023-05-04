
import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt
from os.path import dirname,abspath

mpl.rcParams[u'font.sans-serif'] = ['simhei']
mpl.rcParams['axes.unicode_minus'] = False

speed_f,yaw_rate_f,acc_long_f,acc_lateral_f=[],[],[],[]
speed,yaw_rate,acc_long,acc_lateral=[],[],[],[]
path = dirname(dirname(abspath(__file__)))+"/output/ekf_data.csv"
print(path)
# with open("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze-bfl/data/Const_coef_pos.csv",encoding='utf-8')as fp:
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            speed_f.append(eval(i[0]))
            yaw_rate_f.append(eval(i[1]))            
            acc_long_f.append(eval(i[2]))
            acc_lateral_f.append(eval(i[3]))

path = dirname(dirname(abspath(__file__)))+"/output/output_data.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            speed.append(eval(i[2]))
            yaw_rate.append(eval(i[3]))            
            acc_long.append(eval(i[5]))
            acc_lateral.append(eval(i[4]))
fig = plt.figure(figsize=(15,15))
plt.title("多传感器拓展卡尔曼滤波数据融合",loc='center')
ax1 = fig.add_subplot(2,2,1)
ax1.plot(speed,label='车速') 
ax1.plot(speed_f,label='滤波后的车速') 
plt.legend()

ax2 = fig.add_subplot(2,2,2)
ax2.plot(yaw_rate,label='角速度') 
ax2.plot(yaw_rate_f,label='滤波后的角速度') 
plt.legend()

ax3 = fig.add_subplot(2,2,3)
ax3.plot(acc_long,label='纵向加速度') 
ax3.plot(acc_long_f,label='滤波后的纵向加速度') 
plt.legend()

ax4 = fig.add_subplot(2,2,4)
ax4.plot(acc_lateral,label='侧向加速度') 
ax4.plot(acc_lateral_f,label='滤波后的侧向加速度') 

plt.tight_layout()

plt.legend()
path = dirname(dirname(abspath(__file__)))+"/output/raw&ekf_data.jpg"
plt.savefig(path)
plt.show()

