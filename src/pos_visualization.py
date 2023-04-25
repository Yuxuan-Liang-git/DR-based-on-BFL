import plotly.graph_objects as go
from os.path import dirname,abspath
import csv
import numpy as np
lat_t,lon_t = [],[]
path = dirname(dirname(abspath(__file__)))+"/output/ekf_enu_data.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1:
            lon_t.append(eval(i[0]))
            lat_t.append(eval(i[1]))       
fig = go.Figure(go.Scattermapbox(
    mode = "markers+lines",
    lat = lat_t,
    lon = lon_t,
    name='车辆底盘数据与GPS数据融合后推算出的轨迹',
    marker = {'size': 5}))

gps_lat,gps_lon = [],[]
path = dirname(dirname(abspath(__file__)))+"/output/output_data.csv"
with open(path,encoding='utf-8')as fp:
    reader = csv.reader(fp)
    # 获取标题
    # 遍历数据
    count=0
    for i in reader:
        count+=1    
        if count>1 and count < 382:
            gps_lon.append(eval(i[9]))
            gps_lat.append(eval(i[10]))  
fig.add_trace(go.Scattermapbox(
    mode = "markers",
    lat = gps_lat,
    lon = gps_lon,
    name='GPS轨迹',
    marker = {'size': 10})
)     
# fig = go.Figure(go.Scattermapbox(
#     mode = "markers+lines",
#     lat = gps_lat,
#     lon = gps_lon,
#     marker = {'size': 1}))


fig.update_layout(
    margin ={'l':0,'t':0,'b':0,'r':0},
    mapbox = {
         'center': {'lon': 121.3964, 'lat': 31.17725},
        'style': "stamen-terrain",
        # 'center': {'lon': 121.25, 'lat': 31.5},
        'zoom': 16})

fig.show()