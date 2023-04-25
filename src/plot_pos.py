
import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt
from os.path import dirname,abspath

mpl.rcParams[u'font.sans-serif'] = ['simhei']
mpl.rcParams['axes.unicode_minus'] = False

X_pos,Y_pos,X_pos_f,Y_pos_f=[],[],[],[]
X_pos_s,Y_pos_s=[],[]
X_pos_gps,Y_pos_gps=[],[]
path = dirname(dirname(abspath(__file__)))+"/output/pos.csv"
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
            X_pos.append(eval(i[0]))
            Y_pos.append(eval(i[1]))          

path = dirname(dirname(abspath(__file__)))+"/output/ekf_data.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            X_pos_f.append(eval(i[4]))
            Y_pos_f.append(eval(i[5]))        

path = dirname(dirname(abspath(__file__)))+"/output/gps_pos.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            X_pos_gps.append(eval(i[0]))
            Y_pos_gps.append(eval(i[1]))   



fig = plt.figure(figsize=(15,15))
plt.title("多传感器拓展卡尔曼滤波航位推算（不带GPS）",loc='center')
plt.plot(X_pos,Y_pos,label='滤波前航位推算轨迹') 
plt.plot(X_pos_f,Y_pos_f,label='Odom滤波后航位推算轨迹') 
# plt.plot(X_pos_s,Y_pos_s,label='speed filtered') 
plt.plot(X_pos_gps,Y_pos_gps,label='GPS轨迹') 

plt.axis('equal') 
plt.legend()
path = dirname(dirname(abspath(__file__)))+"/output/position.jpg"
plt.savefig(path)

plt.show()
