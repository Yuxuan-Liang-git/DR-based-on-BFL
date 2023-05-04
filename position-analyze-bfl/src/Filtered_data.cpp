#include "Filtered_data.h"

void Filtered_data::get_ekf_data(void)
{
  /****************************
   * NonLinear system model      *
   ***************************/

  // create gaussian
  ColumnVector sys_noise_Mu(7);
  sys_noise_Mu = 0.0;

  SymmetricMatrix sys_noise_Cov(7);
  for (unsigned int i = 1; i <= 4; i++)
  {
      sys_noise_Cov(i, i) = pow(0.5, 2);
  }
  for (unsigned int i = 5; i <= 7; i++)
  {
      sys_noise_Cov(i, i) = pow(1, 2);
  }
  sys_noise_Cov(2, 2) = pow(0.1, 2);


  Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

  // create the model
  NonLinearAnalyticConditionalGaussianMobile sys_pdf(system_Uncertainty);
  AnalyticSystemModelGaussianUncertainty sys_model(&sys_pdf);

  /*********************************
   * Initialise Odometry measurement model *
   ********************************/

// create matrix H for linear measurement model
//  state(1)    speed
//  state(2)    yaw_rate
//  state(3)    longitude_acc
//  state(4)    lateral_acc
//  state(5)    x_pos
//  state(6)    y_pos
//  state(7)    PHI
  Matrix Hodom(4,7);
  Hodom = 0.0;
  for (unsigned int i = 1; i <= 4; i++)
  {
      Hodom(i, i) = 1;
  }
  // Construct the measurement noise (a scalar in this case)
  ColumnVector odom_meas_noise_Mu(4);
  SymmetricMatrix odom_meas_noise_Cov(4);
  for (unsigned int i = 1; i <= 4; i++)
  {
    odom_meas_noise_Mu(i) = 0;
  }
  odom_meas_noise_Cov(1, 1) = 100;
  odom_meas_noise_Cov(2, 2) = 20;   //  YAW_RATE 基本不用滤了 
  odom_meas_noise_Cov(3, 3) = 50;
  odom_meas_noise_Cov(4, 4) = 10000;  //  侧向加速度误差太大了，不能用  在系统中加了这玩意，yaw_rate就滤不动了...

  Gaussian odom_measurement_Uncertainty(odom_meas_noise_Mu, odom_meas_noise_Cov);

  // create the measurement model
  LinearAnalyticConditionalGaussian odom_meas_pdf(Hodom, odom_measurement_Uncertainty);
  LinearAnalyticMeasurementModelGaussianUncertainty odom_meas_model(&odom_meas_pdf);


  /*********************************
   * Initialise GPS measurement model *
   ********************************/
//  state(1)    speed
//  state(2)    yaw_rate
//  state(3)    longitude_acc
//  state(4)    lateral_acc
//  state(5)    x_pos
//  state(6)    y_pos
//  state(7)    PHI
  Matrix Hgps(3,7);
  Hgps = 0.0;
  for (unsigned int i = 1; i <= 3; i++)
  {
      Hgps(i, i+4) = 1;
  }
  ColumnVector gps_meas_noise_Mu(3);
  SymmetricMatrix gps_meas_noise_Cov(3);
  for (unsigned int i = 1; i <= 3; i++)
  {
    gps_meas_noise_Mu(i) = 0;
  }
  gps_meas_noise_Cov(1, 1) = 5e-1;   //  X_pos
  gps_meas_noise_Cov(2, 2) = 5e-1;   //  Y_pos
  gps_meas_noise_Cov(3, 3) = 1e-5;   //  PHI

  Gaussian gps_measurement_Uncertainty(gps_meas_noise_Mu, gps_meas_noise_Cov);

  // create the measurement model
  LinearAnalyticConditionalGaussian gps_meas_pdf(Hgps, gps_measurement_Uncertainty);
  LinearAnalyticMeasurementModelGaussianUncertainty gps_meas_model(&gps_meas_pdf);

  /****************************
   * Linear prior DENSITY     *
   ***************************/
   // Continuous Gaussian prior (for Kalman filters)
  ColumnVector prior_Mu(7);
  SymmetricMatrix prior_Cov(7);
  prior_Mu = 0;
  for(int i=1;i<=7;i++)
  {
    prior_Cov(i,i) = pow(0.1,2);
  }

  Gaussian prior_cont(prior_Mu,prior_Cov);

  /******************************
   * Construction of the Filter *
   ******************************/
  ExtendedKalmanFilter filter(&prior_cont);

  /***************************
   * initialise MOBILE ROBOT *
   **************************/
  // Model of mobile robot in world with one wall
  // The model is used to simultate the distance measurements.

  /*******************
   * ESTIMATION LOOP *
   *******************/
  cout << "MAIN: Starting estimation" << endl;
  double filter_time_old_ = time_stamp[0];
  double filter_time;
  double process;
  bool get_offset_flag = true;
  int gps_index=0;

  ofstream outFile;
  outFile.open("../output/temp_data.csv", ios::out);
  outFile <<  "gps_measurement"<<',' <<"PHI_f"<<endl;

  for(int i = 1 ;i<time_stamp.size();i++)
    {
      // process = i/time_stamp.size();
      // cout << process  << endl;

      // only update filter for time later than current filter time 
      filter_time = time_stamp[i];
      double dt = (filter_time - filter_time_old_)/1000000.0;
      if (dt == 0)
      {
          cerr << "dt is zero! " << fixed << setprecision(0) << filter_time <<"\t" <<filter_time_old_ << endl;
          continue;
          // return false;
      }
      if (dt < 0)
      {
          cerr << "Will not update robot pose with time" << dt << "sec in the past" << endl;
          // return false;
      }
      if (dt > 40)
      {
          dt-=40;
      }
      filter_time_old_ = filter_time;

      ColumnVector input(1);
      input(1) = dt;
      // UPDATE FILTER
      sys_pdf.AdditiveNoiseSigmaSet(sys_noise_Cov * pow(dt, 2));
      filter.Update(&sys_model,input);

      // DO ONE MEASUREMENT
      ColumnVector odom_measurement(4);
      odom_measurement(1) = speed[i];    //  speed
      odom_measurement(2) = yaw_rate[i];    //  yaw_rate
      odom_measurement(3) = longitude_acc[i];    //  longitude_acc
      if(speed[i]!=0)
      {
        odom_measurement(4) = lateral_acc[i];    //  有速度的时候才更新侧向加速度
      }
      else{
        odom_measurement(4) = 0;    //  lateral_acc 
      }
      odom_meas_pdf.AdditiveNoiseSigmaSet(odom_meas_noise_Cov* pow(dt, 2));

      filter.Update(&odom_meas_model,odom_measurement);

      //filter.Update(&sys_model,input);
      ColumnVector state(7);
      state = filter.PostGet()->ExpectedValueGet();

      speed_f.push_back(state(1));
      yaw_rate_f.push_back(state(2));       
      longitude_acc_f.push_back(state(3));
      lateral_acc_f.push_back(state(4));
      pos_f.X.push_back(state(5));
      pos_f.Y.push_back(state(6));
      pos_f.PHI.push_back(state(7));

      //  获取GPS坐标到相对坐标的旋转与映射关系
      if((time_stamp[i]>gps_time_stamp[gps_index])&&(gps_index<gps_time_stamp.size()))
      {
        //  假设第一个点朝向是对的，如果第一个GPS点错了后面的轨迹全都会错 没空改了
        if(get_offset_flag)
        {
          get_offset_flag = false;
          gps_enu2utm(gps_precision,gps_longitude,
              gps_lattitude,&gps_pos);
          gps_X_offset = pos_f.X.back();
          gps_Y_offset = pos_f.Y.back();
          delta_PHI = pos_f.PHI.back() - gps_orientation[0]-M_PI/2;
          R_Matrix=Eigen::AngleAxisd(delta_PHI,Eigen::Vector3d(0,0,1)).toRotationMatrix();

          cout << "delta_PHI" << '\t' << delta_PHI << endl;
        }
        else
        {
          // cout << gps_index << '\t' << gps_pos.X.size() << endl;
          ColumnVector gps_measurement(3);
          double X,Y;
          X = gps_pos.X[gps_index]-gps_pos.X[0];  
          Y = gps_pos.Y[gps_index]-gps_pos.Y[0];
          // cout << X << '\t' << Y << endl;

          Eigen::Vector3d v(X,Y,0);
          Eigen::Vector3d v_hat =  R_Matrix * v;

          gps_measurement(1) = v_hat[0]+gps_X_offset;
          gps_measurement(2) = v_hat[1]+gps_Y_offset;
          // gps_measurement(3) = pos_f.PHI[i-2];

          gps_measurement(3) = delta_PHI + gps_orientation[gps_index]+M_PI/2;

          //  保证是劣弧
          if(gps_measurement(3)>M_PI)
          {
              gps_measurement(3)-=2*M_PI;
          }else if(gps_measurement(3)<-M_PI)
          {
              gps_measurement(3)+=2*M_PI;
          }
          outFile <<  gps_measurement(3) <<',' <<pos_f.PHI.back() <<endl;

          double temp_delta = gps_measurement(3) - pos_f.PHI.back();
          if(abs(temp_delta)>(M_PI/2))    //  GPS朝向角有时会解算出奇怪的值...就不用gps的朝向角了
          {
            gps_measurement(3) = pos_f.PHI.back();
          }
          gps_meas_pdf.AdditiveNoiseSigmaSet(gps_meas_noise_Cov); 
          filter.Update(&gps_meas_model,gps_measurement);
        }
        gps_index ++ ;
      }


    } // estimation loop
    outFile.close();
  cout << "======================================================" << endl
       << "End of the Kalman filter for mobile robot localisation" << endl
       << "======================================================"
       << endl;

}
void Filtered_data::save_ekf_data(const char* _file_path)
{
  ofstream outFile;
  outFile.open(_file_path, ios::out);
  auto p1 = speed_f.begin();
  auto p2 = yaw_rate_f.begin();
  auto p3 = longitude_acc_f.begin();
  auto p4 = lateral_acc_f.begin();
  auto p5 = pos_f.X.begin();
  auto p6 = pos_f.Y.begin();
  auto p7 = pos_f.PHI.begin();

  outFile <<  "speed_f"<<',' <<"yaw_rate_f"<<',' <<"longitude_acc_f" << "," << "lateral_acc_f" <<"," << "X_f"<<"," << "Y_f"<<"," << "PHI_f"<<endl;
  for (; p1 != speed_f.end(); p1++,p2++,p3++,p4++,p5++,p6++,p7++)
  {
      outFile <<  *p1<<',' <<*p2<<',' <<*p3<<',' <<*p4 <<','<<*p5<<','<<*p6<<','<<*p7<<endl;
  }
  outFile.close();
}
void Filtered_data::save_speed_f_pos(const char* _file_path)
{
    get_pos(yaw_rate,speed_f,&pos_speed_f);
    save_pos(pos_speed_f,_file_path);
}
void Filtered_data::save_filtered_pos(const char* _file_path)
{
    get_pos(yaw_rate_f,speed_f,&pos_f);
    save_pos(pos_f,_file_path);
}

void Filtered_data::save_filtered_pos_enu(const char* _file_path)
{
    pos2gps_transform();
    save_enu(gps_ekf_lon,gps_ekf_lat,_file_path);

}

void Filtered_data::save_gps_data(const char* _file_path)
{
    save_pos(gps_pos,_file_path);
}

void Filtered_data::save_gps_pos(const char* _file_path)
{
  gps2pos_transform();
  save_pos(gps_pos_tf,_file_path);
}



//  将gps坐标与相对坐标对齐
void Filtered_data::gps2pos_transform(void)
{
  double X,Y;
  //  对齐后的坐标
  gps_pos_tf.X.push_back(gps_X_offset);
  gps_pos_tf.Y.push_back(gps_Y_offset);
  gps_pos_tf.PHI.push_back(delta_PHI+gps_orientation[0]+M_PI/2);
  
  for(int j = 1;j<gps_time_stamp.size();j++)
  {
    X = gps_pos.X[j]-gps_pos.X[0];
    Y = gps_pos.Y[j]-gps_pos.Y[0];
    Eigen::Vector3d v(X,Y,0);
    Eigen::Vector3d v_hat =  R_Matrix * v;
    
    gps_pos_tf.X.push_back(v_hat[0]+gps_X_offset);
    gps_pos_tf.Y.push_back(v_hat[1]+gps_Y_offset);
    gps_pos_tf.PHI.push_back(delta_PHI+gps_orientation[j]+M_PI/2);
  }
}


//  将相对坐标与GPS坐标对齐
void Filtered_data::pos2gps_transform(void)
{
  // vector <double> gps_ekf_lon,gps_ekf_lat,gps_odom_lon,gps_odom_lat;
  vector <double> gps_enu_e,gps_enu_n;
  R_Matrix=Eigen::AngleAxisd(-delta_PHI,Eigen::Vector3d(0,0,1)).toRotationMatrix();
  Eigen::Vector3d pos_offset(gps_X_offset,gps_Y_offset,0);
  Eigen::Vector3d gps_offset =  R_Matrix * pos_offset;

  for(int j = 0;j<time_stamp.size();j++)
  {
    Eigen::Vector3d v(pos_f.X[j],pos_f.Y[j],0);
    Eigen::Vector3d v_hat =  R_Matrix * v;
    
    gps_enu_e.push_back(v_hat[0]+gps_pos.X[0]-gps_offset[0]);
    gps_enu_n.push_back(v_hat[1]+gps_pos.Y[0]-gps_offset[1]);
  }
  gps_utm2enu(gps_enu_e,gps_enu_n,&gps_ekf_lon,&gps_ekf_lat);

}



//  经纬度转东北坐标系下的位移
int Filtered_data::gps_enu2utm(vector <double> gps_precision_t,
    vector <double>gps_longitude_t,vector <double> gps_lattitude_t,
    vehicle_pos_s *pos)
{
    PJ_CONTEXT *C;
    PJ *P;
    PJ *norm;
    PJ_COORD a, b;
    /* or you may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                          */
    C = proj_context_create();
    P = proj_create_crs_to_crs (C,
                                "EPSG:4326",
                                "+proj=utm +zone=51 +datum=WGS84", /* or EPSG:32632 */
                                NULL);
    if (0 == P) {
        fprintf(stderr, "Failed to create transformation object.\n");
        return 1;
    }
    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    norm = proj_normalize_for_visualization(C, P);
    if (0 == norm) {
        fprintf(stderr, "Failed to normalize transformation object.\n");
        return 1;
    }
    proj_destroy(P);
    P = norm;
    

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    for (int i=0;i<gps_time_stamp.size();i++)
    {
        a = proj_coord(gps_longitude_t[i], gps_lattitude_t[i], 0, 0);
    /* transform to UTM zone 51, then back to geographical */
        // cout << gps_longitude_t[i] << '\t' << gps_lattitude_t[i] << endl;
        b = proj_trans(P, PJ_FWD, a);

//         b = proj_trans(P, PJ_INV, b);

        // printf("easting: %.3f, northing: %.3f\n", b.enu.e, b.enu.n);
        // sleep(1);

        pos->X.push_back(b.enu.e);
        pos->Y.push_back(b.enu.n);
        pos->PHI.push_back(0);
    }

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}


//  东北坐标系转经纬度
int Filtered_data::gps_utm2enu(vector <double> gps_enu_e_t,
    vector <double>gps_enu_n_t,vector <double> *gps_lon_t,vector <double> *gps_lat_t)
{
    PJ_CONTEXT *C;
    PJ *P;
    PJ *norm;
    PJ_COORD a, b;
    /* or you may set C=PJ_DEFAULT_CTX if you are sure you will     */
    /* use PJ objects from only one thread                          */
    C = proj_context_create();
    P = proj_create_crs_to_crs (C,
                                "EPSG:4326",
                                "+proj=utm +zone=51 +datum=WGS84", /* or EPSG:32632 */
                                NULL);
    if (0 == P) {
        fprintf(stderr, "Failed to create transformation object.\n");
        return 1;
    }
    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    norm = proj_normalize_for_visualization(C, P);
    if (0 == norm) {
        fprintf(stderr, "Failed to normalize transformation object.\n");
        return 1;
    }
    proj_destroy(P);
    P = norm;
    

    /* a coordinate union representing Copenhagen: 55d N, 12d E    */
    /* Given that we have used proj_normalize_for_visualization(), the order of
    /* coordinates is longitude, latitude, and values are expressed in degrees. */
    for (int i=0;i<time_stamp.size();i++)
    {
        a = proj_coord(gps_enu_e_t[i], gps_enu_n_t[i], 0, 0);
        b = proj_trans(P, PJ_INV, a);
        // printf("easting: %.3f, northing: %.3f\n", b.enu.e, b.enu.n);
        // sleep(1);

        gps_lon_t->push_back(b.lp.lam);
        gps_lat_t->push_back(b.lp.phi);
    }

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
    return 0;
}