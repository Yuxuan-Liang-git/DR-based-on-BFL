import numpy as np
import matplotlib as mpl
import csv
import matplotlib.pyplot as plt

X,Y=[],[]
X_const_coef,Y_const_coef=[],[]
with open("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/data/Integrate_pos.csv",encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            X.append(eval(i[0]))
            Y.append(eval(i[1]))


fig = plt.figure()
# ax1 = fig.add_subplot(2,1,1) #或：plt.subplot(2,2,1)



# ax1 = plt.subplot(2,1,1)
plt.plot(X, Y) 
plt.axis('equal')

# plt.title('IMU_Filtered_Intergrate_Pos')
# # plt.ylabel('Y_Pos')

# with open("/home/yuxuanliang/horizion_ws/autodrive-arm/position-analyze/data/Const_coef_pos.csv",encoding='utf-8')as fp:
#     reader = csv.reader(fp)
#     # 获取标题
#     # 遍历数据
#     count=0
#     for i in reader:
#         count+=1    
#         if count>1:
#             X_const_coef.append(eval(i[0]))
#             Y_const_coef.append(eval(i[1]))

# ax1 = plt.subplot(2,1,2)
# plt.plot(X_const_coef, Y_const_coef) 
# plt.axis('equal')

# plt.title('Const_coef_Pos')

plt.show()