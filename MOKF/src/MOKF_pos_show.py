import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt

X_imu,Y_imu=[],[]
X_const,Y_const=[],[]
X_mokf,Y_mokf=[],[]
yaw_rate_const=[]
yaw_rate_imu=[]
yaw_rate_MOKF=[]

def get_pos(file_pos,x,y):
    with open(file_pos,encoding='utf-8')as fp:
        reader = csv.reader(fp)
        # 获取标题
        # 遍历数据
        count=0
        for i in reader:
            count+=1    
            if count>1:
                x.append(eval(i[0]))
                y.append(eval(i[1]))
    fp.close()



get_pos("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/MOKF/data/Integrate_pos.csv",X_imu,Y_imu)
get_pos("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/MOKF/data/Const_coef_pos.csv",X_const,Y_const)
get_pos("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/MOKF/data/MOKF_pos.csv",X_mokf,Y_mokf)

with open("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/MOKF/data/yaw_rate_data.csv",encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            yaw_rate_const.append(eval(i[0]))
            yaw_rate_imu.append(eval(i[1]))
            yaw_rate_MOKF.append(eval(i[2]))
fp.close()

fig = plt.figure(figsize=(20,5))

ax1 = fig.add_subplot(1,3,1)
# ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
ax1.plot(X_const, Y_const) 
plt.axis('equal')

# plt.axis('equal')
plt.title('Const_pos')
# plt.ylabel('Y_Pos')
ax2 = fig.add_subplot(1,3,2)
# ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
ax2.plot(X_imu, Y_imu) 
plt.axis('equal')

plt.title('Imu_pos')
ax3 = fig.add_subplot(1,3,3)
# ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
ax3.plot(X_mokf, Y_mokf) 
plt.title('MOKF_pos')


# fig = plt.figure(figsize=(20,10))
# ax1 = fig.add_subplot(4,3,1)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax1.plot(X_const, Y_const) 
# # plt.axis('equal')
# plt.title('Const_pos')
# # plt.ylabel('Y_Pos')
# ax2 = fig.add_subplot(4,3,2)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax2.plot(X_imu, Y_imu) 
# plt.title('Imu_pos')
# ax3 = fig.add_subplot(4,3,3)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax3.plot(X_mokf, Y_mokf) 
# plt.title('MOKF_pos')

# ax4 = fig.add_subplot(4,1,2)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax4.plot(yaw_rate_const) 
# plt.title('yaw_rate_const')
# ax5 = fig.add_subplot(4,1,3)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax5.plot(yaw_rate_imu) 
# plt.title('yaw_rate_imu')
# ax6 = fig.add_subplot(4,1,4)
# # ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)
# ax6.plot(yaw_rate_MOKF) 
# plt.title('yaw_rate_MOKF')

plt.tight_layout()
plt.axis('equal')
plt.show()