function [pos_X,pos_Y] = get_pos(yaw_rate,time_stamp,speed)
temp_X=0
temp_Y=0
temp_PHI=0
%get_pos 输入yaw_rate 时间 速度 获得积分位置
%   此处显示详细说明
for i=2:length(time_stamp)
   dur = (time_stamp(i)-time_stamp(i-1))/1000000.0;
   if dur>40.0
       dur =dur-40.0;
   end
   temp_X=temp_X+(cos(temp_PHI)*speed(i)*dur);
   temp_Y=temp_Y+(sin(temp_PHI)*speed(i)*dur);
   temp_PHI=temp_PHI+(yaw_rate(i)*dur);
   pos_X(i-1)=temp_X;
   pos_Y(i-1)=temp_Y;
   
end

end

