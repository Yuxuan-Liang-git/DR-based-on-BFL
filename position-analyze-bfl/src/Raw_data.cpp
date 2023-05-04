#include "Raw_data.h"

Raw_data::Raw_data(const char* _data_address):data_address(_data_address)
{
    // 文件打开
    ifstream ifs;							//创建流对象
	ifs.open(data_address, ios::in);	//打开文件
	string temp;						//把文件中的一行数据作为字符串放入容器中
    while (getline(ifs, temp))          //利用getline（）读取每一行，并放入到 item 中
	{
        if (strstr(temp.c_str(), "[CAN]") != NULL)
        {
            istringstream istr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
            string str;
		    int count = 0;							 //统计一行数据中单个数据个数
            while (istr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
            {
                if (count == 0)                      
                {
                    string str_temp;
                    str_temp=str.substr(10,10);
                    long int r = atol(str_temp.c_str());    
                    time_stamp.push_back(r);
                }
                //获取第2列数据
                else if (count == 2)
                {
                    double r = atof(str.c_str())/3.6; //  km/h->m/s
                    speed.push_back(r);
                }
                //获取第2列数据
                else if (count == 3)
                {
                    double r = atof(str.c_str());
                    steering_wheel_angle.push_back(r);
                }
                count++;
            }
            count = 0;
            getline(ifs, temp);
            istringstream tstr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
            while (tstr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
            {
                // cout << "wah"<<endl;
                //获取第1列数据
                if (count == 2)                      
                {
                    double r = atof(str.c_str())*M_PI/180.0;    //deg->rad   
                    if(r>10)
                    {
                        r=0;        //  有异常值...
                    }
                    yaw_rate.push_back(r);
                }
                else if (count == 4)
                {
                    double r = atof(str.c_str())/1.12;      //  注意！这里侧向加速度除了个侧倾角增益
                    lateral_acc.push_back(r);
                }
                //获取第3列数据
                else if (count == 5)
                {
                    double r = atof(str.c_str());
                    longitude_acc.push_back(r);
                }
                count++;
            }
            for(int j=0;j<3;j++)
            {
                getline(ifs, temp);
                // 跳过后面三行
            }
        }

        if (strstr(temp.c_str(), "[GPS]") != NULL)
            {
                istringstream istr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
                string str;
                int count = 0;							 //统计一行数据中单个数据个数
                while (istr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
                {
                    if (count == 0)                      
                    {
                        string str_temp;
                        str_temp=str.substr(10,10);
                        long int r = atol(str_temp.c_str());    
                        gps_time_stamp.push_back(r);
                    }
                    else if (count == 2)
                    {
                        double r = atof(str.c_str());
                        if(r==0 || r>2)
                        {
                            gps_time_stamp.pop_back();
                            break;
                        }
                        gps_precision.push_back(r);
                    }
                    else if (count == 3)
                    {
                        double r = atof(str.c_str());
                        gps_lattitude.push_back(r);
                    }
                    //获取第2列数据
                    else if (count == 4)
                    {
                        double r = atof(str.c_str());
                        gps_longitude.push_back(r);
                    }
                    else if (count == 5)
                    {
                        double r = -atof(str.c_str())*M_PI/180.0;    //deg->rad   数据Z正方向朝下 ;
                        gps_orientation.push_back(r);
                    }
                    count++;
                }
            }
        }
    // 不知道为啥会多一位
    time_stamp.pop_back();
    speed.pop_back();
    steering_wheel_angle.pop_back();
    yaw_rate.pop_back();
    lateral_acc.pop_back();
    longitude_acc.pop_back();
    get_pos(yaw_rate,speed,&raw_pos);

    ifs.close();
}

void Raw_data::show(int i_min,int i_max)
{
    for(int i=i_min;i<i_max;i++)
    {
        cout<<"steering_wheel_angle:"<<steering_wheel_angle[i]<<'\t'<<
            "yaw_rate:"<<yaw_rate[i]<<'\t'<<
            "speed:"<<speed[i]<<'\t'<<
            "lateral_acc:"<<lateral_acc[i]<<'\t'<<
            "longitude_acc:"<<longitude_acc[i]<<endl;
        sleep(1);
    }
}

void Raw_data::save(const char* _file_path)
{
    vector<double> duration;
    double dur;
    for(int i=0;i<time_stamp.size();i++)
    {
        dur=(time_stamp[i]-time_stamp[i-1])/1000000.0;
        if(dur>40.0)
        {
            dur-=40.0;
        }
        duration.push_back(dur);
    }
    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = time_stamp.begin();
    auto p2 = steering_wheel_angle.begin();
    auto p3 = speed.begin();
    auto p4 = yaw_rate.begin();     //  deg/s
    auto p5 = lateral_acc.begin();
    auto p6 = longitude_acc.begin();
    auto p7 = duration.begin();
    outFile << fixed ;              //  浮点数输出
    outFile.precision(14) ;         //  14位有效数字
    outFile <<  "time_stamp" <<',' << "steering_wheel_angle" <<',' <<"speed"  <<',' 
        <<"yaw_rate"<<',' <<"lateral_acc" <<','<<"longitude_acc"<<','<<"duration"<<',';

    auto p8 = gps_time_stamp.begin();
    auto p9 = gps_precision.begin();
    auto p10 = gps_longitude.begin();
    auto p11 = gps_lattitude.begin();
    auto p12 = gps_orientation.begin();
    outFile << "gps_time_stamp" << ',' << "gps_precision" << ',' << "gps_longitude" << ','
        << "gps_lattitude" << ',' << "gps_orientation";

    outFile <<endl;
    for (; p1 != time_stamp.end(); )
    {
        p1++,p2++,p3++,p4++,p5++,p6++,p7++;
        outFile <<  *p1 <<',' << *p2 <<',' <<*p3  <<',' <<*p4<<',' 
            <<*p5 <<','<<*p6<<','<<*p7 << ',';
        if(p8 != gps_time_stamp.end())
        {
            
            outFile << *p8 << ',' << *p9 << ',' << *p10 << ',' << *p11 << ',' <<*p12 <<',';
 
            p8++,p9++,p10++,p11++,p12++;
        }

        outFile <<endl;
    }
    outFile.close();
}

void Raw_data::get_pos(vector <double> yaw_rate_t,vector <double> speed_t,vehicle_pos_s *pos)
{
    double X=0.0,Y=0.0,PHI=0.0;
    double beta=0.0;
    double dur;
    for(int i=1;i<time_stamp.size();i++)
    {
        dur=(time_stamp[i]-time_stamp[i-1])/1000000.0;
        if(dur>40.0)
        {
            dur-=40.0;
        }
        X+=(cos(PHI)*speed_t[i]*dur);
        Y+=(sin(PHI)*speed_t[i]*dur);
        PHI+=yaw_rate_t[i]*dur;   
    
        // if(PHI>M_PI)
        // {
        //     PHI-=2*M_PI;
        // }else if(PHI<-M_PI)
        // {
        //     PHI+=2*M_PI;
        // }
        // cout << "\t" << yaw_rate_t[i] << "\t" << X << "\t" << Y << endl;
        // sleep(1); 
        pos->X.push_back(X);
        pos->Y.push_back(Y);
        pos->PHI.push_back(PHI);
    }
}

void Raw_data::save_intergrate_pos(const char* _file_path)
{
    save_pos(raw_pos,_file_path);
}

void Raw_data::save_pos(vehicle_pos_s pos,const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = pos.X.begin();
    auto p2 = pos.Y.begin();
    auto p3 = pos.PHI.begin();
    outFile << fixed ;              //  浮点数输出
    outFile.precision(14) ;         //  14位有效数字

    outFile <<  "X" <<',' << "Y" <<',' <<"PHI" <<endl;
    for (; p1 != pos.X.end(); p1++,p2++,p3++)
    {
        outFile <<  *p1 <<',' << *p2 <<',' <<*p3  <<endl;
    }
    outFile.close();
}

void Raw_data::save_enu(vector <double> gps_lon_t,vector <double> gps_lat_t,const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = gps_lon_t.begin();
    auto p2 = gps_lat_t.begin();
    auto p3 = time_stamp.begin();
    outFile << fixed ;              //  浮点数输出
    outFile.precision(14) ;         //  14位有效数字

    outFile <<  "gps_lon" <<',' << "gps_lat" << "time_stamp" <<endl;
    for (; p1 != gps_lon_t.end(); p1++,p2++,p3++)
    {
        outFile <<  *p1 <<',' << *p2  <<',' << *p3 <<endl;
    }
    outFile.close();
}
/*
名称:my_split(const std::string& src, const char& delim,
         std::vector<std::string>& vec)

功能:用分隔符将源字符串分隔为多个子串并传出; n个分隔符, 分n+1个子串
    

参数:
     src-传入参数, 源字符串;
     delim-传入参数, 分隔符;
     vec-传出参数, 子串的集合;
  
返回值:
    0-成功;
    其它-失败;
 */

int my_split(const std::string& src, const char& delim,
		std::vector<std::string>& vec)
{
	int src_len = src.length();
	int find_cursor = 0;
	int read_cursor = 0;
	if (src_len <= 0) return -1;
	vec.clear();
	while (read_cursor < src_len){
		find_cursor = src.find(delim, find_cursor);
		//1.找不到分隔符
		if (-1 == find_cursor){
			if (read_cursor <= 0) return -1;
			//最后一个子串, src结尾没有分隔符
			if (read_cursor < src_len){
				vec.push_back(src.substr(read_cursor, src_len - read_cursor));
				return 0;
			}
		}
		//2.有连续分隔符的情况
		else if (find_cursor == read_cursor){
			//字符串开头为分隔符, 也按空子串处理, 如不需要可加上判断&&(read_cursor!=0)
			vec.push_back(std::string(""));
		}
		//3.找到分隔符
		else
			vec.push_back(src.substr(read_cursor, find_cursor - read_cursor));
		read_cursor = ++find_cursor;
		if (read_cursor == src_len){
			//字符串以分隔符结尾, 如不需要末尾空子串, 直接return
			vec.push_back(std::string(""));
			return 0;
		} 
	}//end while()
	return 0;
}
