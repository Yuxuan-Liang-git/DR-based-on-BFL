
import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt
from os.path import dirname,abspath
from math import *
Arrow_lenth = 120
dense = 10
arrow_size = 0.0015

mpl.rcParams[u'font.sans-serif'] = ['simhei']
mpl.rcParams['axes.unicode_minus'] = False

X_pos,Y_pos,PHI_pos=[],[],[]
path = dirname(dirname(abspath(__file__)))+"/output/pos.csv"
print(path)
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count%dense == 0:       # 点太多了看不清
            X_pos.append(eval(i[0]))
            Y_pos.append(eval(i[1]))    
            PHI_pos.append(eval(i[2]))    
u_pos,v_pos = [],[]
for i in range(len(X_pos)):
    u_pos.append(cos(PHI_pos[i])*Arrow_lenth)
    v_pos.append(sin(PHI_pos[i])*Arrow_lenth)

X_pos_f,Y_pos_f,PHI_pos_f=[],[],[]
path = dirname(dirname(abspath(__file__)))+"/output/ekf_data.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count%dense == 0: 
            X_pos_f.append(eval(i[4]))
            Y_pos_f.append(eval(i[5]))        
            PHI_pos_f.append(eval(i[6]))   
u_pos_f,v_pos_f = [],[]
for i in range(len(X_pos_f)):
    u_pos_f.append(cos(PHI_pos_f[i])*Arrow_lenth)
    v_pos_f.append(sin(PHI_pos_f[i])*Arrow_lenth)

X_pos_gps,Y_pos_gps,PHI_pos_gps=[],[],[]
path = dirname(dirname(abspath(__file__)))+"/output/gps_pos.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count > 1 and count %6 ==0:
            X_pos_gps.append(eval(i[0]))
            Y_pos_gps.append(eval(i[1]))   
            PHI_pos_gps.append(eval(i[2])) 
u_pos_gps,v_pos_gps = [],[]
for i in range(len(X_pos_gps)):
    u_pos_gps.append(cos(PHI_pos_gps[i])*Arrow_lenth)
    v_pos_gps.append(sin(PHI_pos_gps[i])*Arrow_lenth)

plt.figure(figsize=(15,15))
plt.quiver(X_pos, Y_pos, u_pos, v_pos,color='gray', angles='xy', scale_units='xy', label='纯Odom轨迹',scale=10,width=arrow_size)
plt.quiver(X_pos_f, Y_pos_f, u_pos_f, v_pos_f,color='red', angles='xy', scale_units='xy', label='Odom与GPS滤波后轨迹',scale=10,width=arrow_size)
plt.quiver(X_pos_gps, Y_pos_gps, u_pos_gps, v_pos_gps,color='blue', angles='xy', scale_units='xy', label='纯GPS轨迹',scale=10,width=arrow_size)
plt.axis('equal') 
plt.title("车辆轨迹与朝向对比")
plt.legend()
path = dirname(dirname(abspath(__file__)))+"/output/position.jpg"
plt.savefig(path)

plt.show()
