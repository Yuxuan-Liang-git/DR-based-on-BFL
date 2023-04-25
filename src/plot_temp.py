
import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt
from os.path import dirname,abspath

speed_f,yaw_rate_f,acc_long_f,acc_lateral_f=[],[],[],[]
speed,yaw_rate,acc_long,acc_lateral=[],[],[],[]
path = dirname(dirname(abspath(__file__)))+"/data/output_data.csv"
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

yaw_rate_acc = []
for i in range(1,len(acc_lateral)):
    if(speed[i]!=0):
        yaw_rate_acc.append(acc_lateral[i]/speed[i])
    else:
        yaw_rate_acc.append(0)

path = dirname(dirname(abspath(__file__)))+"/data/test_data.csv"
head = ['yaw_rate', 'yaw_rate_acc']
# 以自动关闭文件的方式写入
r = zip(yaw_rate,yaw_rate_acc)
with open(path, 'w', encoding='utf-8') as f:
    writer = csv.writer(f)
    for row in r:
        writer.writerow(row)

