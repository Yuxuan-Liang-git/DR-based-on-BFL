
import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt
from os.path import dirname,abspath
X_pos,Y_pos,X_pos_f,Y_pos_f=[],[],[],[]
X_pos_s,Y_pos_s=[],[]
path = dirname(dirname(abspath(__file__)))+"/output/gps_data.csv"
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
plt.axis('equal') 
plt.legend()
plt.plot(X_pos,Y_pos,label='gps position') 
path = dirname(dirname(abspath(__file__)))+"/output/gps_data.jpg"
plt.savefig(path)

plt.show()

